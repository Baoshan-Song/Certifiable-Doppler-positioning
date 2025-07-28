function unixTime = gps2unix(gpsWeek, gpsSecond)
% GPS2UNIX Convert GPS time to Unix time
%   unixTime = GPS2UNIX(gpsWeek, gpsSecond)
%   Converts GPS week and seconds to Unix time (seconds since 1970-01-01).
%
%   Inputs:
%       gpsWeek - GPS week number
%       gpsSecond - Seconds within the GPS week
%
%   Outputs:
%       unixTime - Unix time (seconds since 1970-01-01)

% GPS epoch (January 6, 1980)
gpsEpoch = datenum(1980, 1, 6, 0, 0, 0);

% Convert GPS time to MATLAB datenum
dtNum = gpsEpoch + gpsWeek * 7 + gpsSecond / 86400;

% Convert MATLAB datenum to Unix time
unixTime = (dtNum - datenum(1970, 1, 1)) * 86400;
end