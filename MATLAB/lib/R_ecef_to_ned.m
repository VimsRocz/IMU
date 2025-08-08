function R = R_ecef_to_ned(lat, lon)
%R_ECEF_TO_NED Rotation matrix from ECEF to NED frame.
%   R = R_ECEF_TO_NED(LAT, LON) returns the 3x3 rotation matrix that
%   converts ECEF vectors to the local North-East-Down frame at latitude LAT
%   and longitude LON (radians).

sphi = sin(lat); cphi = cos(lat);
slam = sin(lon); clam = cos(lon);
R = [ -sphi*clam, -sphi*slam,  cphi;
           -slam,       clam,  0;
      -cphi*clam, -cphi*slam, -sphi ];
end
