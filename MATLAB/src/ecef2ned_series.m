function [pos_ned, vel_ned] = ecef2ned_series(pos_ecef, vel_ecef, lat_rad, lon_rad, r0_ecef)
%ECEF2NED_SERIES Convert ECEF position/velocity arrays to NED.
%   [POS_NED, VEL_NED] = ECEF2NED_SERIES(POS_ECEF, VEL_ECEF, LAT_RAD, LON_RAD,
%   R0_ECEF) converts the Nx3 matrices POS_ECEF and VEL_ECEF from
%   Earth-Centred Earth-Fixed to the local North-East-Down frame defined by
%   LAT_RAD and LON_RAD (radians). R0_ECEF is the reference ECEF origin.
%
%   This mirrors the helper used in the Python pipeline.

N = size(pos_ecef,1);
pos_ned = zeros(N,3);
vel_ned = zeros(N,3);
C = compute_C_ECEF_to_NED(lat_rad, lon_rad);
for i = 1:N
    pos_ned(i,:) = (C * (pos_ecef(i,:).' - r0_ecef(:))).';
    vel_ned(i,:) = (C * vel_ecef(i,:).').';
end
end
