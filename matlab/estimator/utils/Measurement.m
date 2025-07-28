
classdef Measurement
    properties
        doppler
        satellite_velocity
        satellite_position
    end
    
    methods
        function obj = Measurement(doppler, satellite_velocity, satellite_position)
            obj.doppler = doppler;
            obj.satellite_velocity = satellite_velocity;
            obj.satellite_position = satellite_position;
        end
    end
end