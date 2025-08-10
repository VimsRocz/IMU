function [lat_deg, lon_deg, h] = ecef_to_geodetic(x, y, z)
%ECEF_TO_GEODETIC Convert ECEF coordinates to geodetic latitude, longitude and altitude.
%   [LAT_DEG, LON_DEG, H] = ECEF_TO_GEODETIC(X, Y, Z) converts Earth-Centred
%   Earth-Fixed coordinates (metres) into geodetic latitude and longitude in
%   degrees and height above the WGS-84 ellipsoid. If the Mapping Toolbox is
%   available the function calls ``ecef2geodetic`` for maximum accuracy,
%   otherwise it falls back to a lightweight implementation shared with the
%   Python version.

    if exist('ecef2geodetic', 'file') == 2 && exist('wgs84Ellipsoid', 'file') == 2
        try
            wgs84 = wgs84Ellipsoid('meter');
            [lat_deg, lon_deg, h] = ecef2geodetic(wgs84, x, y, z);
            return;
        catch %#ok<CTCH>
            % fall through to the custom implementation if the call fails
        end
    end

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
    h = p ./ cos(lat_rad) - N;

    lat_deg = rad2deg(lat_rad);
    lon_deg = rad2deg(lon_rad);
end
