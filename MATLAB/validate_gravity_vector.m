function is_valid = validate_gravity_vector(acc_body, start_idx, end_idx)
%VALIDATE_GRAVITY_VECTOR Validate gravity magnitude from accelerometer data.
%   IS_VALID = VALIDATE_GRAVITY_VECTOR(ACC_BODY, STATIC_START, STATIC_END)
%   computes the mean magnitude of the accelerometer measurements between
%   STATIC_START and STATIC_END and compares it against the expected
%   gravity value 9.79424753 m/s^2 used by the Python implementation.
%   The function returns true when the difference is less than 0.01 m/s^2.
%
%   This mirrors the helper used in ``src/gnss_imu_fusion``.
%
%   Parameters
%   ----------
%   ACC_BODY : Nx3 matrix
%       Body-frame accelerometer data in m/s^2.
%   STATIC_START, STATIC_END : int
%       Index range of the static interval (1-based, inclusive).
%
%   Returns
%   -------
%   IS_VALID : logical
%       True when the measured magnitude matches expected gravity.

    if nargin < 3
        error('validate_gravity_vector requires ACC_BODY, START_IDX and END_IDX');
    end
    if start_idx < 1 || end_idx > size(acc_body, 1)
        error('Invalid static interval indices: start_idx=%d, end_idx=%d', start_idx, end_idx);
    end
    static_acc = acc_body(start_idx:end_idx, :);
    gravity_magnitude = mean(vecnorm(static_acc, 2, 2));
    expected_gravity = 9.79424753;
    is_valid = abs(gravity_magnitude - expected_gravity) < 0.01;
    fprintf('Estimated gravity magnitude: %.4f m/s^2 (expected %.4f)\n', gravity_magnitude, expected_gravity);
end
