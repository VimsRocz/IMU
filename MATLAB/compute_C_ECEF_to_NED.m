function C = compute_C_ECEF_to_NED(lat_rad, lon_rad)
%COMPUTE_C_ECEF_TO_NED Rotation matrix from ECEF to NED.
%   C = COMPUTE_C_ECEF_TO_NED(LAT_RAD, LON_RAD) returns the 3x3 rotation
%   matrix that transforms vectors from the Earth-Centered Earth-Fixed
%   frame to the local North-East-Down frame at the specified latitude and
%   longitude (given in radians).

s_lat = sin(lat_rad); c_lat = cos(lat_rad);
s_lon = sin(lon_rad); c_lon = cos(lon_rad);

C = [-s_lat * c_lon, -s_lat * s_lon,  c_lat;
     -s_lon,          c_lon,         0;
     -c_lat * c_lon, -c_lat * s_lon, -s_lat];
end
