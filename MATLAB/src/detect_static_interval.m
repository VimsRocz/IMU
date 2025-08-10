function [static_start, static_end] = detect_static_interval(acc_body, gyro_body, window_size, accel_var_thresh, gyro_var_thresh, min_length)
%DETECT_STATIC_INTERVAL  Find a static IMU segment using variance thresholds.
%   [START, END] = DETECT_STATIC_INTERVAL(ACC_BODY, GYRO_BODY) searches for the
%   longest contiguous segment where both the accelerometer and gyroscope
%   variances remain below fixed thresholds.  This mirrors the helper used in
%   the Python pipeline.
%
%   Optional arguments match the Python defaults:
%       WINDOW_SIZE       - length of the rolling variance window (default 80)
%       ACCEL_VAR_THRESH  - accelerometer variance threshold (default 0.01)
%       GYRO_VAR_THRESH   - gyroscope variance threshold (default 1e-6)
%       MIN_LENGTH        - minimum acceptable segment length (default 80)
%
%   Returned indices are 1-based with ``END`` being exclusive, matching the
%   Python implementation.

    if nargin < 3 || isempty(window_size)
        window_size = 80; % Match Python defaults
    end
    if nargin < 4 || isempty(accel_var_thresh)
        accel_var_thresh = 0.01;
    end
    if nargin < 5 || isempty(gyro_var_thresh)
        gyro_var_thresh = 1e-6;
    end
    if nargin < 6 || isempty(min_length)
        min_length = 80;
    end

    if size(acc_body,1) < window_size
        error('detect_static_interval:WindowTooLarge', ...
              'window_size larger than data length');
    end

    num_win = size(acc_body,1) - window_size;
    accel_var = zeros(num_win, size(acc_body,2));
    gyro_var  = zeros(num_win, size(gyro_body,2));
    for i = 1:num_win
        accel_var(i,:) = var(acc_body(i:i+window_size-1,:), 0, 1);
        gyro_var(i,:)  = var(gyro_body(i:i+window_size-1,:), 0, 1);
    end

    max_accel_var = max(accel_var, [], 2);
    max_gyro_var  = max(gyro_var,  [], 2);

    static_mask = (max_accel_var < accel_var_thresh) & (max_gyro_var < gyro_var_thresh);

    d = diff([false; static_mask(:); false]);
    start_idx = find(d==1);
    end_idx   = find(d==-1) - 1;

    longest_len = 0;
    static_start = 1;
    static_end = window_size + 1;
    for k = 1:numel(start_idx)
        len = end_idx(k) - start_idx(k) + 1;
        if len >= min_length && len > longest_len
            longest_len = len;
            static_start = start_idx(k);
            static_end = end_idx(k) + window_size;
        end
    end

end
