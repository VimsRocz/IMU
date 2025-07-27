function [lat_deg, lon_deg, h] = ecef_to_geodetic(x, y, z)
%ECEF_TO_GEODETIC  Convert ECEF coordinates to geodetic latitude, longitude and height.
%% Converts ECEF coordinates to geodetic coordinates (lat, lon, h) using WGS84.
%   [LAT_DEG, LON_DEG, H] = ECEF_TO_GEODETIC(X, Y, Z) converts Earth-Centered
%   Earth-Fixed (ECEF) coordinates ``X``, ``Y`` and ``Z`` (in metres) to the
%   corresponding geodetic latitude, longitude (degrees) and height above the
%   WGS‑84 ellipsoid (metres).  Inputs may be scalars or arrays of equal size and
%   the outputs will match that size.
%
%   This implementation does not require the Mapping Toolbox.  When the toolbox
%   is available, the built-in ``ecef2geodetic`` could be used instead.

% WGS‑84 ellipsoid parameters
    a = 6378137.0;               % semi-major axis [m]
    e = 0.081819190842622;       % first eccentricity

    e_sq = e^2;
    p = sqrt(x.^2 + y.^2);

    b = a * sqrt(1 - e_sq);
    theta = atan2(z .* a, p .* b);
    ep_sq = (a^2 - b^2) / b^2;

    lon = atan2(y, x);
    lat = atan2( ...
        z + ep_sq * b .* sin(theta).^3, ...
        p - e_sq * a .* cos(theta).^3);

    N = a ./ sqrt(1 - e_sq * sin(lat).^2);
    h = p ./ cos(lat) - N;

    lat_deg = rad2deg(lat);
    lon_deg = rad2deg(lon);
end

