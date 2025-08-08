function q = dcm_to_quat(C)
% DCM_TO_QUAT  Convert rotation matrix to quaternion [qw qx qy qz].
%   q = DCM_TO_QUAT(C) returns the quaternion corresponding to rotation
%   matrix C.  The function assumes a passive rotation matrix (body->NED
%   or similar) but is convention agnostic.
%
%   Usage:
%       q = dcm_to_quat(C)

    assert(all(size(C)==[3 3]), 'dcm_to_quat expects 3x3');
    tr = trace(C);
    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (C(3,2) - C(2,3)) / S;
        qy = (C(1,3) - C(3,1)) / S;
        qz = (C(2,1) - C(1,2)) / S;
    else
        [~, i] = max([C(1,1), C(2,2), C(3,3)]);
        switch i
            case 1
                S = sqrt(1.0 + C(1,1) - C(2,2) - C(3,3)) * 2;
                qw = (C(3,2) - C(2,3)) / S;
                qx = 0.25 * S;
                qy = (C(1,2) + C(2,1)) / S;
                qz = (C(1,3) + C(3,1)) / S;
            case 2
                S = sqrt(1.0 + C(2,2) - C(1,1) - C(3,3)) * 2;
                qw = (C(1,3) - C(3,1)) / S;
                qx = (C(1,2) + C(2,1)) / S;
                qy = 0.25 * S;
                qz = (C(2,3) + C(3,2)) / S;
            otherwise
                S = sqrt(1.0 + C(3,3) - C(1,1) - C(2,2)) * 2;
                qw = (C(2,1) - C(1,2)) / S;
                qx = (C(1,3) + C(3,1)) / S;
                qy = (C(2,3) + C(3,2)) / S;
                qz = 0.25 * S;
        end
    end
    q = [qw qx qy qz];
end

