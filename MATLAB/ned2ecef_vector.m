function ecef = ned2ecef_vector(ned, lat, lon)
%NED2ECEF_VECTOR Rotate a single NED vector to ECEF.
%   ECEF = NED2ECEF_VECTOR(NED, LAT, LON) converts the 1x3 vector NED from
%   the local North-East-Down frame to Earth-Centred Earth-Fixed. Mirrors
%   the Python helper ``frames.ned_to_ecef_vector``.

R = [ -sin(lat)*cos(lon), -sin(lon),          -cos(lat)*cos(lon);
      -sin(lat)*sin(lon),  cos(lon),          -cos(lat)*sin(lon);
       cos(lat)        ,  0       ,          -sin(lat)        ];

ecef = (R * ned(:)).';
end
