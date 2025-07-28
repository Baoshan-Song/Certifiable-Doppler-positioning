function [lat, lon] = getGridPoints(centerLat, centerLon, range, interval)
    % centerLat: Center latitude in degrees
    % centerLon: Center longitude in degrees
    % range: Range in km
    % interval: Interval in km

    % Earth's radius in meters
    R = 6378137;

    % Convert range and interval to radians
    range_rad = range / R;
    interval_rad = interval / R;

    % Generate grid points
    [Y, X] = meshgrid(-range:interval:range, -range:interval:range);
    distances = sqrt(X.^2 + Y.^2) * interval_rad;
    bearings = atan2(X, Y);

    % Calculate new latitude and longitude
    lat_new_rad = asin(sin(centerLat*pi/180) .* cos(distances) + ...
                      cos(centerLat*pi/180) .* sin(distances) .* cos(bearings));
    lon_new_rad = centerLon*pi/180 + atan2(sin(bearings) .* sin(distances) .* cos(centerLat*pi/180), ...
                                          cos(distances) - sin(centerLat*pi/180) .* sin(lat_new_rad));

    % Convert back to degrees
    lat = lat_new_rad * 180/pi;
    lon = lon_new_rad * 180/pi;
end