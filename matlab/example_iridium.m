%% Example: Local search v.s. Semidefinite relaxation for satellite Doppler positioning (Iridium dataset)
clc; clear; close all;  % start clean

%% Input measurements and ephemeris
filename = 'data/iridium/Iridium_Doppler_measurements.csv';
freq = 1626270833; % Hz

% Read the CSV file using readtable
data = readtable(filename);

% Initialize a cell array
num_rows = height(data);
satellite_data = cell(num_rows, 1);
measurements = [];
for i = 1:num_rows
    measured_doppler = -data{i, 3} * 299792458 / freq; % Column 3: Measured Doppler shift
    sat_position = data{i, 4:6}; % Columns 4 to 6: Satellite position
    sat_velocity = data{i, 7:9}; % Columns 7 to 9: Satellite velocity
    measurements = [measurements; Measurement(measured_doppler, sat_velocity, sat_position)];
end

%% Initialize parameters
MAX_ITER = 1000;
VEL_GT = [0; 0; 0];
gt = [-2418244.984840921; 5385836.046258101; 2405675.159335429]; % Ground truth
X = gt; % Initial guess
% X = X+8e5;   %% add large initial position error

%% State estimation
% Gauss-Newton
[X_gn, iter_gn] = gauss_newton(measurements, VEL_GT, X, MAX_ITER);

% Dog-Leg
[X_dl, iter_dl] = dog_leg(measurements, VEL_GT, X, MAX_ITER);

% Semidefinite Programming (SDP)
SCALE_P = 1e7;
SCALE_V = 1e3;

[X_sdp] = sdp_solver_clk(measurements, VEL_GT, SCALE_P, SCALE_V);


%% Display results
fprintf('Gauss-Newton result:\n');
disp(X_gn);
disp(X_gn - gt);
fprintf('iteration: %d\n\n', iter_gn);

fprintf('Dog-Leg result:\n');
disp(X_dl);
disp(X_dl - gt);
fprintf('iteration: %d\n', iter_dl);

fprintf('SDP result:\n');
disp(X_sdp);
disp(X_sdp - gt);


