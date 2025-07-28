function [gpsWeek, gpsSecond] = datetime2gps(year, month, day, hour, minute, second)
% DATETIME2GPS Convert datetime to GPS week and seconds
%   [gpsWeek, gpsSecond] = DATETIME2GPS(year, month, day, hour, minute, second)
%   Converts the given datetime to GPS week and seconds.
%
%   Inputs:
%       year, month, day, hour, minute, second - Datetime components
%
%   Outputs:
%       gpsWeek - GPS week number
%       gpsSecond - Seconds within the GPS week

% GPS epoch (January 6, 1980)
gpsEpoch = datenum(1980, 1, 6, 0, 0, 0);

% Convert datetime to datenum
dt = datenum(year, month, day, hour, minute, second);

% Calculate the number of days since GPS epoch
daysSinceEpoch = dt - gpsEpoch;

% Convert days to GPS week and seconds
gpsWeek = floor(daysSinceEpoch / 7);
gpsSecond = rem(daysSinceEpoch, 7) * 86400;
end