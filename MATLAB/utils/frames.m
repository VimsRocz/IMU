function [n,e,d] = ecef_to_ned(x,y,z,lat0,lon0,h0)
%ECEF_TO_NED Convert ECEF coordinates to local NED frame
if nargin < 6, h0 = 0; end
[x0,y0,z0] = lla_to_ecef(lat0, lon0, h0);
R = rot_ecef_to_ned(lat0, lon0);
diff = [x - x0; y - y0; z - z0];
ned = R * diff;
n = ned(1,:); e = ned(2,:); d = ned(3,:);
end

function [x,y,z] = lla_to_ecef(lat, lon, h)
A = 6378137.0; E2 = 6.69437999014e-3;
lat = deg2rad(lat); lon = deg2rad(lon);
N = A ./ sqrt(1 - E2*sin(lat).^2);
x = (N + h).*cos(lat).*cos(lon);
y = (N + h).*cos(lat).*sin(lon);
z = (N*(1 - E2) + h).*sin(lat);
end

function R = rot_ecef_to_ned(lat, lon)
lat = deg2rad(lat); lon = deg2rad(lon);
R = [ -sin(lat).*cos(lon), -sin(lon), -cos(lat).*cos(lon);
      -sin(lat).*sin(lon),  cos(lon), -cos(lat).*sin(lon);
       cos(lat),            0,        -sin(lat)];
end
end
