function [X, iter] = gauss_newton(measurements, VEL_GT, init_X, max_iter)
    X = init_X;
    for iter = 1:max_iter
        H = []; 
        Z = [];
        
        for i = 1:length(measurements)
            delta_p = X(1:3) - measurements(i).satellite_position';
            delta_v = VEL_GT - measurements(i).satellite_velocity';
            D = measurements(i).doppler;
            rho = norm(delta_p);

            z = D - delta_p' * delta_v / rho;
            h = (-delta_v') * (eye(3) / rho - delta_p * delta_p' / (rho^3));
            % 
            % % scheme 2
            % z = rho*D - delta_p' * delta_v;
            % h = delta_p'*D/rho - delta_v';

            H = [H; h];
            Z = [Z; z];
        end
        
        delta_X = (H' * H) \ (H' * Z);
        X = X - delta_X;
        
        if norm(delta_X) < 1e-6
            break;
        end
    end
end