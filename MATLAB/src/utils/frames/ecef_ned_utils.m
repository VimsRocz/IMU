function R = ecef_to_ned(lat, lon)
%ECEF_TO_NED Rotation from ECEF to NED at latitude ``lat`` and longitude ``lon``.
%   R = ECEF_TO_NED(lat, lon) returns the 3x3 direction cosine matrix that
%   transforms vectors from the ECEF frame to the local NED frame.
sl = sin(lat); cl = cos(lat);
st = sin(lon); ct = cos(lon);
R = [ -sl*ct, -sl*st,  cl;
      -st,     ct,     0;
      -cl*ct, -cl*st, -sl ];
end

function R = ned_to_ecef(lat, lon)
%NED_TO_ECEF Rotation from NED to ECEF at latitude ``lat`` and longitude ``lon``.
R = ecef_to_ned(lat, lon).';
end
