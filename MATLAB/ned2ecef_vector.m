function vec_ecef = ned2ecef_vector(vec_ned, lat_rad, lon_rad)
%NED2ECEF_VECTOR Rotate NED vectors to ECEF.
%   VEC_ECEF = NED2ECEF_VECTOR(VEC_NED, LAT_RAD, LON_RAD) rotates the Nx3 matrix
%   VEC_NED from North-East-Down to Earth-Centered Earth-Fixed using the
%   reference latitude and longitude in radians.

C_e_n = compute_C_ECEF_to_NED(lat_rad, lon_rad);
vec_ecef = (C_e_n' * vec_ned')';
end
