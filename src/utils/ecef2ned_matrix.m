function C = ecef2ned_matrix(lat_rad, lon_rad)
%ECEF2NED_MATRIX  Rotation matrix from ECEF to NED.
%   C = ECEF2NED_MATRIX(LAT_RAD, LON_RAD) returns the 3x3 rotation matrix
%   that transforms vectors from the Earth-Centred Earth-Fixed frame to the
%   local North-East-Down frame at the given latitude and longitude
%   (radians).

    s_lat = sin(lat_rad); c_lat = cos(lat_rad);
    s_lon = sin(lon_rad); c_lon = cos(lon_rad);
    C = [-s_lat * c_lon, -s_lat * s_lon,  c_lat;
         -s_lon,          c_lon,         0;
         -c_lat * c_lon, -c_lat * s_lon, -s_lat];
end
