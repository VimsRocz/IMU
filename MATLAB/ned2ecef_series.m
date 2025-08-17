function [pos_ecef, vel_ecef] = ned2ecef_series(pos_ned, vel_ned, lat_rad, lon_rad, r0_ecef)
%NED2ECEF_SERIES Convert NED position/velocity arrays to ECEF.
%   [POS_ECEF, VEL_ECEF] = NED2ECEF_SERIES(POS_NED, VEL_NED, LAT_RAD, LON_RAD,
%   R0_ECEF) converts the Nx3 matrices POS_NED and VEL_NED from North-East-Down
%   coordinates to Earth-Centered Earth-Fixed. LAT_RAD and LON_RAD specify the
%   reference latitude and longitude in radians. R0_ECEF is the 3-element ECEF
%   reference position vector.
%
%   This mirrors the helper used in the Python pipeline.

N = size(pos_ned,1);
pos_ecef = zeros(N,3);
vel_ecef = zeros(N,3);
for i = 1:N
    pos_ecef(i,:) = ned2ecef_vector(pos_ned(i,:), lat_rad, lon_rad) + r0_ecef(:).';
    vel_ecef(i,:) = ned2ecef_vector(vel_ned(i,:), lat_rad, lon_rad);
end
end
