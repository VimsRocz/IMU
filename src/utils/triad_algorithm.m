function [R_bn, q_bn] = triad_algorithm(v1_b, v2_b, v1_n, v2_n)
%TRIAD_ALGORITHM  Compute body-to-NED rotation using TRIAD.
%   [R, Q] = TRIAD_ALGORITHM(V1_B, V2_B, V1_N, V2_N) returns the rotation
%   matrix ``R`` and quaternion ``Q`` (``[w x y z]``) that align the body
%   frame vectors ``V1_B``/``V2_B`` with the reference vectors
%   ``V1_N``/``V2_N`` in the NED frame.

    M_b = triad_basis(v1_b, v2_b);
    M_n = triad_basis(v1_n, v2_n);
    R_bn = M_n * M_b';
    q_bn = rotm2quat_custom(R_bn);
end

function M = triad_basis(v1, v2)
    t1 = v1 / norm(v1);
    t2 = cross(t1, v2);
    t2 = t2 / norm(t2);
    t3 = cross(t1, t2);
    M = [t1(:), t2(:), t3(:)];
end

function q = rotm2quat_custom(R)
    tr = trace(R);
    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (R(3,2) - R(2,3)) / S;
        qy = (R(1,3) - R(3,1)) / S;
        qz = (R(2,1) - R(1,2)) / S;
    else
        [~, idx] = max([R(1,1), R(2,2), R(3,3)]);
        switch idx
            case 1
                S = sqrt(1 + R(1,1) - R(2,2) - R(3,3)) * 2;
                qw = (R(3,2) - R(2,3)) / S;
                qx = 0.25 * S;
                qy = (R(1,2) + R(2,1)) / S;
                qz = (R(1,3) + R(3,1)) / S;
            case 2
                S = sqrt(1 + R(2,2) - R(1,1) - R(3,3)) * 2;
                qw = (R(1,3) - R(3,1)) / S;
                qx = (R(1,2) + R(2,1)) / S;
                qy = 0.25 * S;
                qz = (R(2,3) + R(3,2)) / S;
            case 3
                S = sqrt(1 + R(3,3) - R(1,1) - R(2,2)) * 2;
                qw = (R(2,1) - R(1,2)) / S;
                qx = (R(1,3) + R(3,1)) / S;
                qy = (R(2,3) + R(3,2)) / S;
                qz = 0.25 * S;
        end
    end
    q = [qw, qx, qy, qz];
end
