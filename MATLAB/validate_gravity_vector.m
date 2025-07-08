function g_ned = validate_gravity_vector(lat_deg, h)
%VALIDATE_GRAVITY_VECTOR Print gravity magnitude and return vector in NED.
%   g_ned = VALIDATE_GRAVITY_VECTOR(LAT_DEG, H) computes the gravity
%   magnitude using the WGS-84 normal gravity formula and prints the
%   formatted message shown by the Python version of the pipeline.

if nargin < 2
    h = 0;
end
lat_rad = deg2rad(lat_deg);
sin_lat = sin(lat_rad);

% WGS-84 normal gravity with height correction
g = 9.7803253359 * (1 + 0.00193185265241 * sin_lat^2) / ...
    sqrt(1 - 0.00669437999013 * sin_lat^2);

% height above ellipsoid correction
g = g - 3.086e-6 * h;

fprintf(['[Gravity Validation] Latitude: %.3f deg, altitude: %.1f m --> ' ...
        'Gravity: %.6f m/s^2 (NED +Z is down)\n'], lat_deg, h, g);

g_ned = [0; 0; g];
end
