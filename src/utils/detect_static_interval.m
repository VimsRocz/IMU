function [start_idx, end_idx] = detect_static_interval(accel, gyro, window_size, accel_var_thresh, gyro_var_thresh, min_length)
%DETECT_STATIC_INTERVAL  Find longest initial static segment of IMU data.
%   [START_IDX, END_IDX] = DETECT_STATIC_INTERVAL(ACCEL, GYRO) returns the
%   start and end indices of the longest segment where the variance of the
%   accelerometer and gyroscope signals stays below predefined thresholds.

    if nargin < 3 || isempty(window_size);      window_size = 200;   end
    if nargin < 4 || isempty(accel_var_thresh); accel_var_thresh = 0.01; end
    if nargin < 5 || isempty(gyro_var_thresh);  gyro_var_thresh = 1e-6; end
    if nargin < 6 || isempty(min_length);       min_length = 100;   end

    N = size(accel,1);
    if N < window_size
        error('window_size larger than data length');
    end

    if exist('movvar','file') == 2
        accel_var = movvar(accel, window_size, 0, 'Endpoints','discard');
        gyro_var  = movvar(gyro,  window_size, 0, 'Endpoints','discard');
    else
        num_win = N - window_size + 1;
        accel_var = zeros(num_win, size(accel,2));
        gyro_var  = zeros(num_win, size(gyro,2));
        for i = 1:num_win
            accel_var(i,:) = var(accel(i:i+window_size-1,:),0,1);
            gyro_var(i,:)  = var(gyro(i:i+window_size-1,:),0,1);
        end
    end
    max_accel_var = max(accel_var, [], 2);
    max_gyro_var  = max(gyro_var, [], 2);
    static_mask = (max_accel_var < accel_var_thresh) & (max_gyro_var < gyro_var_thresh);
    diff_mask = diff([0; static_mask; 0]);
    starts = find(diff_mask == 1);
    ends   = find(diff_mask == -1) - 1;
    longest_len = 0; start_idx = 1; end_idx = window_size;
    for k = 1:length(starts)
        seg_len = ends(k) - starts(k) + 1;
        if seg_len >= min_length && seg_len > longest_len
            longest_len = seg_len;
            start_idx = starts(k);
            end_idx = ends(k) + window_size - 1;
        end
    end
end
