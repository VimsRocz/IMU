function [lat, lon, h] = ecef2lla_wgs84_fallback(x, y, z)
%ECEF2LLA_WGS84_FALLBACK Simple ECEF->LLA conversion without toolboxes
%   Uses WGS-84 parameters.

% constants
A = 6378137.0;        % semi-major axis
E2 = 6.69437999014e-3; % eccentricity squared
B = sqrt(A^2*(1-E2));

p = sqrt(x.^2 + y.^2);
th = atan2(A*z, B*p);
lon = atan2(y, x);
lat = atan2(z + (sqrt(A^2-B^2).^2/B).*sin(th).^3, ...
            p - E2*A*cos(th).^3);
N = A ./ sqrt(1 - E2*sin(lat).^2);
h = p./cos(lat) - N;

lat = rad2deg(lat);
lon = rad2deg(lon);
end
