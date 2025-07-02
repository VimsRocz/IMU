function [lat_deg, lon_deg, alt] = ecef2geodetic(x, y, z)
%ECEF2GEODETIC Convert ECEF coordinates to geodetic latitude, longitude and altitude.
%   [LAT_DEG, LON_DEG, ALT] = ECEF2GEODETIC(X, Y, Z) converts Earth-Centred
%   Earth-Fixed coordinates (metres) into geodetic latitude and longitude in
%   degrees and altitude in metres. The implementation follows the WGS-84
%   conversion used in src/utils.py so results match the Python code.

    a = 6378137.0;          % semi-major axis (metres)
    e_sq = 6.69437999014e-3; % first eccentricity squared

    p = sqrt(x.^2 + y.^2);
    b = a * sqrt(1 - e_sq);
    theta = atan2(z .* a, p .* b);
    ep_sq = (a^2 - b^2) / b^2;
    lon_rad = atan2(y, x);
    lat_rad = atan2(z + ep_sq .* b .* sin(theta).^3, ...
                    p - e_sq .* a .* cos(theta).^3);
    N = a ./ sqrt(1 - e_sq .* sin(lat_rad).^2);
    alt = p ./ cos(lat_rad) - N;

    lat_deg = rad2deg(lat_rad);
    lon_deg = rad2deg(lon_rad);
end
