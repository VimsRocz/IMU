function [grav_err_deg, earth_err_deg] = compute_wahba_errors(C_bn, g_b, omega_b, g_ref, omega_ref)
%COMPUTE_WAHBA_ERRORS Angular errors for gravity and Earth rate.
%   [EG, EO] = COMPUTE_WAHBA_ERRORS(C_BN, G_B, OMEGA_B, G_REF, OMEGA_REF)
%   returns the angle between measured and reference gravity vectors and
%   between Earth rotation vectors, matching the Python helper of the same
%   name.
%
%   Inputs:
%     C_bn      - body-to-NED rotation matrix
%     g_b       - gravity vector measured in the body frame
%     omega_b   - Earth rotation vector measured in the body frame
%     g_ref     - reference gravity in the NED frame
%     omega_ref - reference Earth rotation in the NED frame
%
%   Outputs:
%     grav_err_deg  - gravity vector angular error in degrees
%     earth_err_deg - Earth-rate vector angular error in degrees
%
%   Frames:
%     g_b, omega_b are expressed in the body frame.
%     g_ref, omega_ref are expressed in the NED frame.
%
%   See also: angle_between
%
%   Usage:
%       [eg, eo] = compute_wahba_errors(C_bn, g_b, omega_b, g_ref, omega_ref)
%
%   This mirrors ``gnss_imu_fusion.init_vectors.compute_wahba_errors`` in
%   the Python implementation to keep cross-language parity.

    g_pred = C_bn * g_b;
    omega_pred = C_bn * omega_b;

    grav_err_deg = angle_between(g_pred, g_ref);
    earth_err_deg = angle_between(omega_pred, omega_ref);
end

function deg = angle_between(v1, v2)
%ANGLE_BETWEEN Angle between two 3-D vectors in degrees.
%   DEG = ANGLE_BETWEEN(V1, V2)
%   returns the angle between V1 and V2 in degrees.
    cos_theta = max(min(dot(v1, v2) / (norm(v1) * norm(v2)), 1.0), -1.0);
    deg = acosd(cos_theta);
end
