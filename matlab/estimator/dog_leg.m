function [X, iter] = dog_leg(measurements, VEL_GT, init_X, max_iter)
    % Initialize state and previous cost
    X = init_X;
    prev_cost = inf;
    delta = 1e3;  % Trust region radius in meters (adjustable)

    for iter = 1:max_iter
        H = [];
        Z = [];

        % Construct Jacobian matrix H and residual vector Z
        for i = 1:length(measurements)
            delta_p = X(1:3) - (measurements(i).satellite_position)';
            delta_v = VEL_GT - (measurements(i).satellite_velocity)';
            D = measurements(i).doppler;
            rho = norm(delta_p);

            z = D - delta_p' * delta_v / rho;
            h = (-delta_v') * (eye(3) / rho - delta_p * delta_p' / (rho^3));

            H = [H; h];
            Z = [Z; z];
        end

        % Gradient and Hessian approximation
        g = H' * Z;
        HtH = H' * H;

        % Newton step
        delta_x_newton = - (HtH \ g);

        % Steepest descent step (scaled negative gradient)
        delta_x_gradient = - (g / norm(g));

        % Apply Dog-Leg trust region method
        if norm(delta_x_newton) <= delta
            delta_X = delta_x_newton;
        elseif norm(delta_x_gradient) >= delta
            % Scale gradient step to fit within trust region
            delta_X = (delta / norm(delta_x_gradient)) * delta_x_gradient;
        else
            % Compute tau for combination: delta_X = a + tau * (b - a)
            a = delta_x_gradient;
            b = delta_x_newton;
            d = b - a;

            % Solve quadratic equation ||a + τd|| = delta
            A = dot(d, d);
            B = 2 * dot(a, d);
            C = dot(a, a) - delta^2;
            tau = (-B + sqrt(B^2 - 4*A*C)) / (2*A);
            delta_X = a + tau * d;
        end

        % Update the state
        X = X + delta_X;

        % Compute current cost (sum of squared residuals)
        current_cost = 0;
        for i = 1:length(measurements)
            delta_p = X(1:3) - (measurements(i).satellite_position)';
            delta_v = VEL_GT - (measurements(i).satellite_velocity)';
            D = measurements(i).doppler;
            rho = norm(delta_p);
            z = D - delta_p' * delta_v / rho;
            current_cost = current_cost + z^2;
        end

        % Convergence check
        if norm(delta_X) < 1e-6 || abs(current_cost - prev_cost) < 1e-6
            break;
        end

        % Update previous cost
        prev_cost = current_cost;
    end
end
