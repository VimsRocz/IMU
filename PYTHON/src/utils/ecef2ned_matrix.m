function C = ecef2ned_matrix(lat_rad, lon_rad)
%ECEF2NED_MATRIX Rotation matrix from ECEF to NED frame.
%   C = ECEF2NED_MATRIX(LAT_RAD, LON_RAD) returns a 3x3 matrix that
%   rotates a vector from the Earth-Centred Earth-Fixed frame to the
%   local North-East-Down frame.

    s_lat = sin(lat_rad); c_lat = cos(lat_rad);
    s_lon = sin(lon_rad); c_lon = cos(lon_rad);
    C = [
        -s_lat * c_lon, -s_lat * s_lon,  c_lat;
        -s_lon,          c_lon,         0;
        -c_lat * c_lon, -c_lat * s_lon, -s_lat
    ];
end
