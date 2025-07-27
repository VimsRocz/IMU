function [static_start, static_end] = detect_static_interval(acc_body, gyro_body, window_size, accel_var_thresh, gyro_var_thresh)
%DETECT_STATIC_INTERVAL Detect a static interval in IMU data.
%   [STATIC_START, STATIC_END] = DETECT_STATIC_INTERVAL(ACC_BODY, GYRO_BODY)
%   returns the indices of the first and last samples of a segment where both
%   accelerometer and gyroscope variance stay below fixed thresholds.  This
%   mirrors the Python implementation used by ``run_all_methods.py``.
%
%   Optional arguments match the Python defaults:
%       window_size       - variance window length (default 80 samples)
%       accel_var_thresh  - accelerometer variance threshold (default 0.01)
%       gyro_var_thresh   - gyroscope variance threshold (default 1e-6)
%
%   The returned indices are 1-based and inclusive.

    if nargin < 3 || isempty(window_size)
        window_size = 80;
    end
    if nargin < 4 || isempty(accel_var_thresh)
        accel_var_thresh = 0.01;
    end
    if nargin < 5 || isempty(gyro_var_thresh)
        gyro_var_thresh = 1e-6;
    end

    if size(acc_body,1) < window_size
        error('detect_static_interval:WindowTooLarge', ...
              'window_size larger than data length');
    end

    if exist('movvar','file') == 2
        accel_var = movvar(acc_body, window_size, 0, 'Endpoints','discard');
        gyro_var  = movvar(gyro_body,  window_size, 0, 'Endpoints','discard');
    else
        num_win = size(acc_body,1) - window_size + 1;
        accel_var = zeros(num_win, size(acc_body,2));
        gyro_var  = zeros(num_win, size(gyro_body,2));
        for i = 1:num_win
            accel_var(i,:) = var(acc_body(i:i+window_size-1,:), 0, 1);
            gyro_var(i,:)  = var(gyro_body(i:i+window_size-1,:), 0, 1);
        end
    end

    is_static = all(accel_var < accel_var_thresh, 2) & ...
                all(gyro_var < gyro_var_thresh, 2);
    static_indices = find(is_static);
    if isempty(static_indices)
        error('No static intervals detected');
    end
    static_start = static_indices(1);
    static_end   = static_indices(end);
end
