function [R_en, R_ne] = ecef_ned_rot(lat_rad, lon_rad)
%ECEF_NED_ROT  Rotation matrices between ECEF and NED frames.
%   [R_en, R_ne] = ECEF_NED_ROT(lat_rad, lon_rad) returns the rotation
%   matrix from ECEF to NED (R_en) and its transpose (NED to ECEF, R_ne).
%
%   Usage:
%       [R_en, R_ne] = ecef_ned_rot(lat_rad, lon_rad)
%
%   Inputs are latitude and longitude in radians.

    sL = sin(lat_rad); cL = cos(lat_rad);
    sLmbd = sin(lon_rad); cLmbd = cos(lon_rad);

    R_en = [ -sL*cLmbd, -sL*sLmbd,  cL;
             -sLmbd,      cLmbd,    0;
             -cL*cLmbd, -cL*sLmbd, -sL ];
    R_ne = R_en.';
end

