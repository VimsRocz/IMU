function [pos_ecef, vel_ecef] = ned2ecef_series(pos_ned, vel_ned, lat_rad, lon_rad, r0_ecef)
%NED2ECEF_SERIES Convert NED position/velocity arrays to ECEF.
%   [POS_ECEF, VEL_ECEF] = NED2ECEF_SERIES(POS_NED, VEL_NED, LAT_RAD, LON_RAD,
%   R0_ECEF) converts the Nx3 matrices POS_NED and VEL_NED from North-East-Down
%   coordinates to Earth-Centered Earth-Fixed. LAT_RAD and LON_RAD specify the
%   reference latitude and longitude in radians. R0_ECEF is the 3-element ECEF
%   reference position vector.
%
%   This mirrors the helper used in the Python pipeline.

C_e_n = compute_C_ECEF_to_NED(lat_rad, lon_rad);
C_n_e = C_e_n';
pos_ecef = (C_n_e * pos_ned')' + r0_ecef(:)';
vel_ecef = (C_n_e * vel_ned')';
end
