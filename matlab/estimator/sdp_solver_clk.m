function position = sdp_solver_clk(measurements, VEL_GT, SCALE_P, SCALE_V)
thres_hold = 1e-4;

% Initial CVX optimization
cvx_begin sdp
cvx_solver sedumi
cvx_precision low
% cvx_quiet(0)

% Variable definitions
N_COM = 4;
len_meas = length(measurements);
N = N_COM + 2 * len_meas;
variable y(N);
variable Y(N, N) symmetric;

% Construct coefficient matrix
A = zeros(len_meas, N);
k = zeros(len_meas, 1);

for i = 1:len_meas
    doppler = measurements(i).doppler ./ SCALE_V;
    satellite_velocity = measurements(i).satellite_velocity ./ SCALE_V;
    satellite_position = measurements(i).satellite_position ./ SCALE_P;

    delta_v = satellite_velocity - VEL_GT';
    A(i, 1:3) = delta_v;  % Assume position corresponds to the first 3 elements in y
    A(i, N_COM + i) = doppler;

    k(i) = -1 * (delta_v * satellite_position');
end

% Objective function
minimize(trace(A' * A * Y) + 2 * (k' * A * y));

% Constraints
subject to
[Y, y; y', 1] >= 0;  % Semidefinite constraint on lifted matrix

y(1+N_COM:N) >= 0;

for i = 1:len_meas
    satellite_position = measurements(i).satellite_position ./ SCALE_P;
    Y(N_COM + i, N_COM + i) == ...
        trace(Y(1:3, 1:3)) ...
        - 2 * (satellite_position * y(1:3)) ...
        + (satellite_position * satellite_position');

    Y(N_COM, N_COM + i) == y(N_COM + len_meas + i);
end

cvx_end

init_pos = y(1:3);

% Iterative refinement with updated weighting
Q_old = zeros(len_meas);
Q = ones(len_meas);
while abs(trace(Q - Q_old)) > thres_hold

    Q_old = Q;

    % CVX optimization with updated Q
    cvx_begin sdp
    cvx_solver sedumi
    cvx_precision best
    cvx_quiet(0)

    variable y(N);
    variable Y(N, N) symmetric;

    A = zeros(len_meas, N);
    k = zeros(len_meas, 1);

    for i = 1:len_meas
        doppler = measurements(i).doppler ./ SCALE_V;
        satellite_velocity = measurements(i).satellite_velocity ./ SCALE_V;
        satellite_position = measurements(i).satellite_position ./ SCALE_P;

        delta_v = satellite_velocity - VEL_GT';
        A(i, 1:3) = delta_v;
        A(i, N_COM + i) = doppler;

        k(i) = -1 * (delta_v * satellite_position');

        range = norm(init_pos - satellite_position);
        Q(i, i) = range^2;
    end

    Q_inv = inv(Q);

    minimize(trace(A' * Q_inv * A * Y) + 2 * (k' * Q_inv * A * y));

    subject to
    [Y, y; y', 1] >= 0;

    y(1+N_COM:N) >= 0;

    for i = 1:len_meas
        satellite_position = measurements(i).satellite_position ./ SCALE_P;
        Y(N_COM + i, N_COM + i) == ...
            trace(Y(1:3, 1:3)) ...
            - 2 * (satellite_position * y(1:3)) ...
            + (satellite_position * satellite_position');

        Y(N_COM, N_COM + i) == y(N_COM + len_meas + i);
    end

    cvx_end

    init_pos = y(1:3);
end

% Check rank tightness
eigY = eig(Y(1:3, 1:3));  % Eigenvalue decomposition of submatrix
eigY = sort(eigY, 'descend');
d_lambda = eigY(1) / eigY(2);

if strcmp(cvx_status, 'Solved') && d_lambda > 1e5
    disp(cvx_status);
    disp(d_lambda);
    position = y(1:3) * SCALE_P;
else
    disp(cvx_status);
    disp(d_lambda);
    disp('Rank tightness failed!');
    position = zeros(3, 1);
end

end
