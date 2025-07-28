clear; clc; close all;

%% 0. Parameter settings and file names
leo_out_file = 'leo_satellite_l1500_49_orbit800.txt';
dopp_out_file = 'perturb_l1500_49_doppler_file_orbit800.txt';
gs_out_file = 'perturb_leo_file_l1500_49_orbit800.txt';

signal_freq = 1.626e9;               % Signal frequency [Hz]
orbit_height = 800e3;            % Satellite orbit height [m]
range_km = 750; interval_km = 250; % Grid area and interval [km]
doppler_noise_std = 0;            % Doppler noise standard deviation [m/s]
int_pos_noise = 0; int_vel_noise = 0; % Initial orbit perturbation
pr_noise = 0; vr_noise = 0;       % Ground station noise

% Center geodetic coordinates
centerLatitude = 30.183; 
centerLongitude = 129.830;

%% 1. Generate satellite grid and compute ECEF position/velocity
[latitude, longitude] = getGridPoints(centerLatitude, centerLongitude, range_km, interval_km);
wgs84 = wgs84Ellipsoid('meter');
R_E = 6371e3;
G = 6.67430e-11; M = 5.972e24;
R = R_E + orbit_height;
v_norm = sqrt(G * M / R); % Orbital velocity [m/s]

% Initialization
index = 1;
sats = []; % Store satellite parameters
fid = fopen(leo_out_file, 'w');
% 
% directions = [1 0; 0 -1; -1 0; 0 1]; % East, South, West, North
% 
% for i = 1:length(latitude)
%     for j = 1:length(longitude)
%         [X, Y, Z] = geodetic2ecef(wgs84, latitude(i,j), longitude(i,j), orbit_height);
%         r = [X; Y; Z]; r_hat = r / norm(r);
% 
%         % Compute local ENU axes
%         [e, n, u] = enu_axes(latitude(i,j), longitude(i,j));
% 
%         % Randomly choose from East, South, West, North
%         dir_idx = randi(4);
%         enu_direction = directions(dir_idx, 1)*e + directions(dir_idx, 2)*n;
%         v_dir = enu_direction / norm(enu_direction);
%         v = v_norm * v_dir;
% 
%         sats(index,:) = [index, 0, X, Y, Z, v(1), v(2), v(3)];
%         fprintf(fid, 'L%04d %14.3f %14.3f %14.3f %10.3f %10.3f %10.3f\r\n', ...
%             index, X, Y, Z, v(1), v(2), v(3));
%         index = index + 1;
%     end
% end

for i = 1:length(latitude)
    for j = 1:length(longitude)
        [X, Y, Z] = geodetic2ecef(wgs84, latitude(i,j), longitude(i,j), orbit_height);
        r = [X; Y; Z]; r_hat = r / norm(r);

        % Generate tangential velocity direction
        if abs(dot(r_hat, [0; 0; 1])) < 0.99
            temp = [0; 0; 1];
        else
            temp = [1; 0; 0];
        end
        t1 = cross(r_hat, temp); t1 = t1 / norm(t1);
        t2 = cross(r_hat, t1);
        theta = 2*pi*rand(); v_dir = cos(theta)*t1 + sin(theta)*t2;
        v = v_norm * v_dir;

        sats(index,:) = [index, 0, X, Y, Z, v(1), v(2), v(3)];
        fprintf(fid, 'L%04d %14.3f %14.3f %14.3f %10.3f %10.3f %10.3f\r\n', ...
            index, X, Y, Z, v(1), v(2), v(3));
        index = index + 1;
    end
end
fclose(fid);

%% 1.5 Plot satellite locations using m_map
figure;
m_proj('Equidistant Cylindrical','lon',[centerLongitude-5 centerLongitude+5],'lat',[centerLatitude-5 centerLatitude+5]);
m_coast('color','k'); m_grid('box','fancy','tickdir','in');
hold on;

% Flatten latitude and longitude matrices
lat_vec = latitude(:);
lon_vec = longitude(:);
m_plot(lon_vec, lat_vec, 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
title('Satellite Positions in Grid');
xlabel('Longitude'); ylabel('Latitude');

%% 2. Define center point as the receiver
mean_lat = mean(latitude(:));
mean_lon = mean(longitude(:));
[position_r(1), position_r(2), position_r(3)] = geodetic2ecef(wgs84, mean_lat, mean_lon, 0);
velocity_r = zeros(3,1); % Receiver is static

%% 3. Calculate Doppler measurements and apply perturbation
fid = fopen(dopp_out_file, 'w');

% Output receiver header only ONCE
fprintf(fid,'# freq =  %14.3f; dopp_noise = %14.3f \r\n', signal_freq, doppler_noise_std);
fprintf(fid,'# week sow sat_num init_pos_X init_pos_Y init_pos_Z init_vel_X init_vel_Y init_vel_Z \r\n');

% Add noise to receiver (could be zero)
rx_pos_noise = int_pos_noise * (-0.5 + rand(3,1));
rx_vel_noise = int_vel_noise * (-0.5 + rand(3,1));
init_rx_pos = position_r(:) + rx_pos_noise;
init_rx_vel = velocity_r(:) + rx_vel_noise;

% Write receiver init position and velocity
fprintf(fid,'$ %5d %18.3f %3d %14.3f %14.3f %14.3f %14.3f %14.3f %14.3f \r\n', ...
    0, 0.0, size(sats,1), init_rx_pos(1), init_rx_pos(2), init_rx_pos(3), ...
    init_rx_vel(1), init_rx_vel(2), init_rx_vel(3));

% Then write Doppler for all satellites
for k = 1:size(sats,1)
    sat_pos = sats(k,3:5).'; % Satellite true position
    sat_vel = sats(k,6:8).'; % Satellite true velocity

    % Apply perturbation
    pert_sat_pos = sat_pos + int_pos_noise * (-0.5 + rand(3,1));
    pert_sat_vel = sat_vel + int_vel_noise * (-0.5 + rand(3,1));

    % Calculate Doppler using perturbed values
    [dopp, sagnac] = doppler(pert_sat_pos, init_rx_pos, pert_sat_vel, init_rx_vel, signal_freq, doppler_noise_std);
    dopp = dopp + doppler_noise_std*rand(1,1); % Add constant system bias

    % Output satellite Doppler measurement
    fprintf(fid,'L%03d  %14.3f\r\n', k, dopp);
end

fclose(fid);

%% 4. Output perturbed satellite position and velocity (instead of ground station)
fid = fopen(gs_out_file, 'w');
fprintf(fid, '# freq =  %14.3f; noise: init_pos = %14.3f, init_vel = %14.3f \r\n', ...
    signal_freq, int_pos_noise, int_vel_noise);
fprintf(fid, '# prn pos_X pos_Y pos_Z vel_X vel_Y vel_Z clk_shift \r\n');

for k = 1:size(sats, 1)
    sat_pos = sats(k, 3:5).';  % True position
    sat_vel = sats(k, 6:8).';  % True velocity

    % Apply perturbation
    pert_sat_pos = sat_pos + int_pos_noise * ( rand(3,1));
    pert_sat_vel = sat_vel + int_vel_noise * ( rand(3,1));

    % Output perturbed satellite state
    fprintf(fid, 'L%03d %14.3f %14.3f %14.3f %14.3f %14.3f %14.3f 0\r\n', ...
        k, pert_sat_pos(1), pert_sat_pos(2), pert_sat_pos(3), ...
        pert_sat_vel(1), pert_sat_vel(2), pert_sat_vel(3));
end

fclose(fid);


function [e, n, u] = enu_axes(lat, lon)
    % Convert degrees to radians
    lat = deg2rad(lat);
    lon = deg2rad(lon);
    
    % East, North, Up unit vectors in ECEF
    e = [-sin(lon); cos(lon); 0];
    n = [-sin(lat)*cos(lon); -sin(lat)*sin(lon); cos(lat)];
    u = [cos(lat)*cos(lon); cos(lat)*sin(lon); sin(lat)];
end
