function [R_bn, q_bn] = triad_algorithm(v1_b, v2_b, v1_n, v2_n)
%TRIAD_ALGORITHM  Compute attitude from two vector pairs.
%   [R_BN, Q_BN] = TRIAD_ALGORITHM(V1_B, V2_B, V1_N, V2_N) returns the
%   body-to-NED rotation matrix and quaternion using the classic TRIAD
%   method. Vectors are normalised internally.

    v1_b = v1_b(:) / norm(v1_b);
    v2_b = v2_b(:) / norm(v2_b);
    t2b = cross(v1_b, v2_b);
    if norm(t2b) < 1e-10
        if abs(v1_b(1)) < abs(v1_b(2)), tmp = [1;0;0]; else, tmp = [0;1;0]; end
        t2b = cross(v1_b, tmp);
    end
    t2b = t2b / norm(t2b);
    t3b = cross(v1_b, t2b);
    M_b = [v1_b, t2b, t3b];

    v1_n = v1_n(:) / norm(v1_n);
    v2_n = v2_n(:) / norm(v2_n);
    t2n = cross(v1_n, v2_n);
    if norm(t2n) < 1e-10
        if abs(v1_n(1)) < abs(v1_n(2)), tmp = [1;0;0]; else, tmp = [0;1;0]; end
        t2n = cross(v1_n, tmp);
    end
    t2n = t2n / norm(t2n);
    t3n = cross(v1_n, t2n);
    M_n = [v1_n, t2n, t3n];

    R_bn = M_n * M_b';
    q_bn = rotm2quat(R_bn);  % [qw qx qy qz]
    if q_bn(1) < 0
        q_bn = -q_bn;
    end
end
