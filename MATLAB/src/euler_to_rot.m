function R = euler_to_rot(eul)
%EULER_TO_ROT Convert XYZ Euler angles to rotation matrix.
%   R = EULER_TO_ROT(EUL) returns the rotation matrix corresponding to
%   the Euler angles ``[roll; pitch; yaw]`` in radians using the XYZ
%   convention.  This helper mirrors the Python implementation for
%   cross-language parity.
%
%   Usage:
%       R = euler_to_rot([roll; pitch; yaw]);
%
%   See also quat_to_rot.
%
%   "IMU" research codebase utility function.

cr = cos(eul(1)); sr = sin(eul(1));
cp = cos(eul(2)); sp = sin(eul(2));
cy = cos(eul(3)); sy = sin(eul(3));
R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
     sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
     -sp,   cp*sr,            cp*cr];
end
