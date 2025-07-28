function [dopp,earth_error] = doppler(position_s,position_r,velocity_s,velocity_r,signal_freq, noise)
%DOPPLER Summary of this function goes here
% method: f_d = 1/labmda*(v_s-v_r)*(p_s-p_r)/||p_s-p_r|| 
% input:
%   satellite's and receiver's trajectory and velocity (ECEF)
%   signal frequency (Hz),
%   noise (m/s)
% output:
%   doppler measurement
% author: Song Baoshan, 20240313

delta_p = position_s - position_r;
delta_v = velocity_s - velocity_r;
norm_p = norm(delta_p,2);
c_speed = 299792458.0;
earth_speed = 7.2921151467E-5;    % earth angular velocity (IS-GPS) (rad/s) 

% earth rotation error
% earth_error = earth_speed / c_speed * (velocity_s(2) * position_r(2) + position_s(2) * velocity_r(2) - velocity_s(1) * position_r(1) - position_s(1) * velocity_r(1));
earth_error = 0.0;
% doppler 
dopp = (signal_freq/c_speed) * ((delta_v'*delta_p)/norm_p + earth_error + noise); % Hz

end

