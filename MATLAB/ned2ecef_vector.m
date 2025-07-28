function ecef = ned2ecef_vector(ned, lat_rad, lon_rad)
%NED2ECEF_VECTOR Rotate a single NED vector to ECEF.
%   ECEF = NED2ECEF_VECTOR(NED, LAT_RAD, LON_RAD) converts the 1x3 vector NED
%   from the local North-East-Down frame to Earth-Centred Earth-Fixed.

R_n2e = [ -sin(lat_rad)*cos(lon_rad), -sin(lon_rad), -cos(lat_rad)*cos(lon_rad);
          -sin(lat_rad)*sin(lon_rad),  cos(lon_rad), -cos(lat_rad)*sin(lon_rad);
           cos(lat_rad)            ,             0 , -sin(lat_rad)           ];

ecef = R_n2e * ned(:);
ecef = ecef.';  % return as row vector
end
