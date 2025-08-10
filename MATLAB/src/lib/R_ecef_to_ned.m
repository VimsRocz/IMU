function R = R_ecef_to_ned(lat, lon)
%R_ECEF_TO_NED Rotation matrix from ECEF to NED frame.
%   R = R_ECEF_TO_NED(lat, lon) returns the 3x3 rotation matrix that maps
%   Earth-Centered Earth-Fixed (ECEF) vectors to North-East-Down (NED)
%   coordinates. Latitude and longitude are given in radians.
%
%   The rows of R correspond to unit vectors of the NED frame expressed in
%   ECEF coordinates. This implementation mirrors the Python version in
%   src/utils/frames.py.

sphi = sin(lat);  cphi = cos(lat);
slam = sin(lon);  clam = cos(lon);
R = [ -sphi*clam, -sphi*slam,  cphi;
           -slam,       clam,  0;
      -cphi*clam, -cphi*slam, -sphi ];
end
