%% Example: Local search v.s. Semidefinite relaxation for satellite Doppler positioning (Simulation)
clc; clear; close all;  % start clean

%% Input measurements and ephemeris
doppler_file_path =   'data/simulation/perturb_l1500_49_doppler_file_orbit800.txt';
ephemeris_file_path = 'data/simulation/perturb_leo_file_l1500_49_orbit800.txt';

data_reader = SatelliteDataReader(doppler_file_path, ephemeris_file_path);
data_reader=data_reader.read_doppler_data();
data_reader=data_reader.read_ephemeris_data();

%% Initialize parameters
[measurements, init_pv] = data_reader.get_measurement_for_epoch(1);  % Get measurements for epoch 1
MAX_ITER = 1000;
gt = init_pv(1:3)';
VEL_GT = init_pv(4:6)';
X= gt;
% X = X+5.5e5;   %% add large initial position error

%% State estimation
% Gaussian-Newton
[X_gn, iter_gn] = gauss_newton(measurements, VEL_GT, X, MAX_ITER);

% Dog-Leg
[X_dl, iter_dl] = dog_leg(measurements, VEL_GT, X, MAX_ITER);

% Semidefinite Programming (SDP)
SCALE_P = 1e7;
SCALE_V = 1e3;
[X_sdp] = sdp_solver_clk(measurements, VEL_GT, SCALE_P, SCALE_V);

%% Disp results
fprintf('Gauss-Newton result:\n');
disp(X_gn);
disp(X_gn-gt);
fprintf('iteration: %d\n\n', iter_gn);
fprintf('Dog-Leg result:\n');
disp(X_dl);
disp(X_dl-gt);
fprintf('iteration: %d\n', iter_dl);
fprintf('SDP result:\n');
disp(X_sdp);
disp(X_sdp-gt);



