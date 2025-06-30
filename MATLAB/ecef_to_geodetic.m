function [lat_deg, lon_deg, alt] = ecef_to_geodetic(x, y, z)
%ECEF_TO_GEODETIC Convert ECEF coordinates to geodetic latitude, longitude, 
% and altitude (WGS-84).
%
%   [lat_deg, lon_deg, alt] = ecef_to_geodetic(x, y, z)
%
%   Inputs:
%       x, y, z - ECEF coordinates in meters.
%   Outputs:
%       lat_deg, lon_deg - Latitude and Longitude in degrees.
%       alt - Altitude in meters.

    % WGS-84 ellipsoid constants
    a = 6378137.0;          % Semi-major axis (meters)
    e_sq = 6.69437999014e-3; % First eccentricity squared
    
    % Calculations
    p = sqrt(x^2 + y^2);
    theta = atan2(z * a, p * (1 - e_sq));
    
    lon_rad = atan2(y, x);
    lat_rad = atan2(z + e_sq * a * sin(theta)^3 / (1 - e_sq), ...
                    p - e_sq * a * cos(theta)^3);
                 
    N = a / sqrt(1 - e_sq * sin(lat_rad)^2);
    alt = p / cos(lat_rad) - N;
    
    % Convert radians to degrees for output
    lat_deg = rad2deg(lat_rad);
    lon_deg = rad2deg(lon_rad);
end
