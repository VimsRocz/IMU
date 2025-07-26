function [R_bn, q_bn] = triad_algorithm(acc_body, mag_body, vel_ned)
%TRIAD_ALGORITHM Compute initial attitude using TRIAD.
%   [R_BN, Q_BN] = TRIAD_ALGORITHM(ACC_BODY, MAG_BODY, VEL_NED) returns the
%   body-to-NED rotation matrix and equivalent quaternion.
%   ACC_BODY and MAG_BODY are 3x1 vectors measured in the body frame.
%   VEL_NED is the velocity vector in the navigation frame.

    b1 = acc_body / norm(acc_body);
    b2 = cross(b1, mag_body);
    b2 = b2 / norm(b2);
    b3 = cross(b1, b2);
    B = [b1, b2, b3];

    n1 = vel_ned / norm(vel_ned);
    n_ref = [1; 0; 0];
    n2 = cross(n1, n_ref);
    if norm(n2) < eps
        n_ref = [0; 1; 0];
        n2 = cross(n1, n_ref);
    end
    n2 = n2 / norm(n2);
    n3 = cross(n1, n2);
    N = [n1, n2, n3];

    R_bn = B * N';
    q_bn = rotm2quat(R_bn);
    if q_bn(1) < 0
        q_bn = -q_bn;
    end
end
