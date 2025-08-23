function [accel_clean, gyro_clean, compensation_info] = vibration_compensation(accel, gyro, fs, varargin)
%VIBRATION_COMPENSATION Remove vibration signals from IMU data.
%   [ACCEL_CLEAN, GYRO_CLEAN, INFO] = VIBRATION_COMPENSATION(ACCEL, GYRO, FS)
%   applies vibration compensation to remove vibration artifacts from
%   accelerometer and gyroscope measurements.
%
%   [ACCEL_CLEAN, GYRO_CLEAN, INFO] = VIBRATION_COMPENSATION(ACCEL, GYRO, FS, 'PARAM', VALUE, ...)
%   allows specification of compensation parameters.
%
%   Inputs:
%       ACCEL - Accelerometer data (Nx3) in m/s^2
%       GYRO  - Gyroscope data (Nx3) in rad/s
%       FS    - Sampling frequency in Hz
%
%   Parameters:
%       'Method'           - Compensation method: 'lowpass', 'notch', 'adaptive', 'spectral' (default: 'adaptive')
%       'VibrationFreq'    - Known vibration frequency in Hz (for notch filter)
%       'FreqRange'        - Vibration frequency range [low, high] in Hz (default: [10, 200])
%       'FilterOrder'      - Filter order (default: 4)
%       'AdaptationRate'   - Adaptation rate for adaptive filter (default: 0.01)
%       'NotchWidth'       - Notch filter bandwidth in Hz (default: 5)
%       'AutoDetect'       - Auto-detect vibration parameters (default: true)
%
%   Outputs:
%       ACCEL_CLEAN       - Compensated accelerometer data (Nx3)
%       GYRO_CLEAN        - Compensated gyroscope data (Nx3)
%       COMPENSATION_INFO - Structure with compensation details:
%           .method              - Method used
%           .vibration_detected  - Boolean indicating if vibration was detected
%           .removed_frequencies - Frequencies that were filtered out
%           .filter_response     - Filter frequency response (if applicable)
%           .effectiveness       - Compensation effectiveness ratio (0-1)
%
%   Examples:
%       % Basic adaptive compensation
%       [accel_clean, gyro_clean, info] = vibration_compensation(accel, gyro, 400);
%
%       % Notch filter for known vibration frequency
%       [accel_clean, gyro_clean, info] = vibration_compensation(accel, gyro, 400, ...
%                       'Method', 'notch', 'VibrationFreq', 50);
%
%       % Low-pass filter compensation
%       [accel_clean, gyro_clean, info] = vibration_compensation(accel, gyro, 400, ...
%                       'Method', 'lowpass', 'FreqRange', [0, 20]);
%
%   See also VIBRATION_MODEL, VIBRATION_DETECTION.

    % Parse input arguments
    p = inputParser;
    addRequired(p, 'accel');
    addRequired(p, 'gyro');
    addRequired(p, 'fs');
    addParameter(p, 'Method', 'adaptive', @(x) any(validatestring(x, {'lowpass', 'notch', 'adaptive', 'spectral'})));
    addParameter(p, 'VibrationFreq', [], @isnumeric);
    addParameter(p, 'FreqRange', [10, 200], @(x) isnumeric(x) && length(x) == 2);
    addParameter(p, 'FilterOrder', 4, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'AdaptationRate', 0.01, @(x) isnumeric(x) && x > 0 && x < 1);
    addParameter(p, 'NotchWidth', 5, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'AutoDetect', true, @islogical);
    
    parse(p, accel, gyro, fs, varargin{:});
    
    method = p.Results.Method;
    vibration_freq = p.Results.VibrationFreq;
    freq_range = p.Results.FreqRange;
    filter_order = p.Results.FilterOrder;
    adaptation_rate = p.Results.AdaptationRate;
    notch_width = p.Results.NotchWidth;
    auto_detect = p.Results.AutoDetect;
    
    % Ensure data is in correct format
    if size(accel, 1) < size(accel, 2)
        accel = accel';
    end
    if size(gyro, 1) < size(gyro, 2)
        gyro = gyro';
    end
    
    % Initialize compensation info
    compensation_info = struct();
    compensation_info.method = method;
    compensation_info.vibration_detected = false;
    compensation_info.removed_frequencies = [];
    compensation_info.filter_response = [];
    compensation_info.effectiveness = 0;
    
    % Auto-detect vibration if requested
    if auto_detect
        detection_result = vibration_detection(accel, gyro, fs, 'FreqRange', freq_range);
        compensation_info.vibration_detected = detection_result.vibration_detected;
        
        if detection_result.vibration_detected && isempty(vibration_freq)
            vibration_freq = detection_result.dominant_freq;
        end
    end
    
    % Apply compensation based on method
    switch lower(method)
        case 'lowpass'
            [accel_clean, gyro_clean, info] = apply_lowpass_compensation(accel, gyro, fs, freq_range, filter_order);
            
        case 'notch'
            [accel_clean, gyro_clean, info] = apply_notch_compensation(accel, gyro, fs, vibration_freq, notch_width, filter_order);
            
        case 'adaptive'
            [accel_clean, gyro_clean, info] = apply_adaptive_compensation(accel, gyro, fs, freq_range, adaptation_rate);
            
        case 'spectral'
            [accel_clean, gyro_clean, info] = apply_spectral_compensation(accel, gyro, fs, freq_range);
            
        otherwise
            error('vibration_compensation:InvalidMethod', 'Unknown compensation method: %s', method);
    end
    
    % Merge compensation info
    compensation_info = merge_structs(compensation_info, info);
    
    % Calculate effectiveness
    if compensation_info.vibration_detected
        compensation_info.effectiveness = calculate_effectiveness(accel, gyro, accel_clean, gyro_clean, fs, freq_range);
    end
end

function [accel_clean, gyro_clean, info] = apply_lowpass_compensation(accel, gyro, fs, freq_range, filter_order)
%APPLY_LOWPASS_COMPENSATION Low-pass filter based compensation.
    
    cutoff_freq = freq_range(1); % Use lower bound as cutoff
    
    info = struct();
    info.removed_frequencies = [cutoff_freq, fs/2];
    
    % Apply low-pass filter
    if exist('butter', 'file') == 2 && exist('filtfilt', 'file') == 2
        nyq = fs / 2;
        [b, a] = butter(filter_order, cutoff_freq / nyq, 'low');
        
        accel_clean = zeros(size(accel));
        gyro_clean = zeros(size(gyro));
        
        for axis = 1:3
            accel_clean(:, axis) = filtfilt(b, a, accel(:, axis));
            gyro_clean(:, axis) = filtfilt(b, a, gyro(:, axis));
        end
        
        % Compute filter response
        [h, w] = freqz(b, a, 256);
        info.filter_response = struct('frequencies', w*fs/(2*pi), 'magnitude', abs(h));
    else
        % Fallback using basic filter
        accel_clean = low_pass_filter(accel, cutoff_freq, fs);
        gyro_clean = low_pass_filter(gyro, cutoff_freq, fs);
        info.filter_response = [];
    end
end

function [accel_clean, gyro_clean, info] = apply_notch_compensation(accel, gyro, fs, vibration_freq, notch_width, filter_order)
%APPLY_NOTCH_COMPENSATION Notch filter based compensation.
    
    info = struct();
    
    if isempty(vibration_freq) || isnan(vibration_freq)
        % No vibration frequency detected, return original data
        accel_clean = accel;
        gyro_clean = gyro;
        info.removed_frequencies = [];
        info.filter_response = [];
        return;
    end
    
    info.removed_frequencies = [vibration_freq - notch_width/2, vibration_freq + notch_width/2];
    
    % Design notch filter
    if exist('butter', 'file') == 2 && exist('filtfilt', 'file') == 2
        nyq = fs / 2;
        low_freq = max(1, vibration_freq - notch_width/2);
        high_freq = min(nyq - 1, vibration_freq + notch_width/2);
        
        [b, a] = butter(filter_order, [low_freq, high_freq] / nyq, 'stop');
        
        accel_clean = zeros(size(accel));
        gyro_clean = zeros(size(gyro));
        
        for axis = 1:3
            accel_clean(:, axis) = filtfilt(b, a, accel(:, axis));
            gyro_clean(:, axis) = filtfilt(b, a, gyro(:, axis));
        end
        
        % Compute filter response
        [h, w] = freqz(b, a, 256);
        info.filter_response = struct('frequencies', w*fs/(2*pi), 'magnitude', abs(h));
    else
        % Fallback: simple moving average (not ideal but functional)
        window_size = round(fs / vibration_freq / 2);
        accel_clean = apply_moving_average(accel, window_size);
        gyro_clean = apply_moving_average(gyro, window_size);
        info.filter_response = [];
    end
end

function [accel_clean, gyro_clean, info] = apply_adaptive_compensation(accel, gyro, fs, freq_range, adaptation_rate)
%APPLY_ADAPTIVE_COMPENSATION Adaptive filter based compensation.
    
    info = struct();
    info.removed_frequencies = freq_range;
    
    % Initialize
    N = size(accel, 1);
    accel_clean = zeros(size(accel));
    gyro_clean = zeros(size(gyro));
    
    % Adaptive filter parameters
    filter_length = min(32, round(fs/freq_range(1)/2)); % Adaptive filter length
    
    for axis = 1:3
        % Apply LMS adaptive filter to each axis
        [accel_clean(:, axis), weights_accel] = lms_adaptive_filter(accel(:, axis), filter_length, adaptation_rate);
        [gyro_clean(:, axis), weights_gyro] = lms_adaptive_filter(gyro(:, axis), filter_length, adaptation_rate);
    end
    
    info.filter_response = struct('accel_weights', weights_accel, 'gyro_weights', weights_gyro);
end

function [accel_clean, gyro_clean, info] = apply_spectral_compensation(accel, gyro, fs, freq_range)
%APPLY_SPECTRAL_COMPENSATION Spectral subtraction based compensation.
    
    info = struct();
    info.removed_frequencies = freq_range;
    
    % Apply spectral subtraction to each axis
    accel_clean = zeros(size(accel));
    gyro_clean = zeros(size(gyro));
    
    for axis = 1:3
        accel_clean(:, axis) = spectral_subtraction(accel(:, axis), fs, freq_range);
        gyro_clean(:, axis) = spectral_subtraction(gyro(:, axis), fs, freq_range);
    end
    
    info.filter_response = [];
end

function [output, weights] = lms_adaptive_filter(input, filter_length, mu)
%LMS_ADAPTIVE_FILTER Least Mean Squares adaptive filter.
    
    N = length(input);
    weights = zeros(filter_length, 1);
    output = zeros(N, 1);
    
    for n = filter_length:N
        % Get input vector
        x = input(n:-1:n-filter_length+1);
        
        % Filter output
        y = weights' * x;
        output(n) = y;
        
        % Error calculation (assuming we want to remove the correlated component)
        error = input(n) - y;
        
        % Weight update
        weights = weights + mu * error * x;
    end
    
    % Copy initial samples
    output(1:filter_length-1) = input(1:filter_length-1);
    
    % Subtract filtered component to get cleaned signal
    output = input - output;
end

function clean_signal = spectral_subtraction(signal, fs, freq_range)
%SPECTRAL_SUBTRACTION Spectral subtraction for vibration removal.
    
    % Compute FFT
    N = length(signal);
    Y = fft(signal);
    freqs = (0:N-1) * fs / N;
    
    % Create frequency mask
    mask = ones(size(freqs));
    vibration_indices = (freqs >= freq_range(1) & freqs <= freq_range(2)) | ...
                       (freqs >= fs - freq_range(2) & freqs <= fs - freq_range(1));
    
    % Attenuate vibration frequencies
    attenuation = 0.1; % Keep 10% of original amplitude
    mask(vibration_indices) = attenuation;
    
    % Apply mask and convert back to time domain
    Y_clean = Y .* mask';
    clean_signal = real(ifft(Y_clean));
end

function smoothed_data = apply_moving_average(data, window_size)
%APPLY_MOVING_AVERAGE Apply moving average to multi-channel data.
    
    if window_size <= 1
        smoothed_data = data;
        return;
    end
    
    [N, num_channels] = size(data);
    smoothed_data = zeros(size(data));
    
    if exist('movmean', 'file') == 2
        for ch = 1:num_channels
            smoothed_data(:, ch) = movmean(data(:, ch), window_size);
        end
    else
        % Manual implementation
        half_win = floor(window_size / 2);
        for ch = 1:num_channels
            for i = 1:N
                start_idx = max(1, i - half_win);
                end_idx = min(N, i + half_win);
                smoothed_data(i, ch) = mean(data(start_idx:end_idx, ch));
            end
        end
    end
end

function effectiveness = calculate_effectiveness(accel_orig, gyro_orig, accel_clean, gyro_clean, fs, freq_range)
%CALCULATE_EFFECTIVENESS Calculate vibration compensation effectiveness.
    
    % Compute power in vibration band before and after compensation
    power_before = calculate_vibration_power([accel_orig, gyro_orig], fs, freq_range);
    power_after = calculate_vibration_power([accel_clean, gyro_clean], fs, freq_range);
    
    % Effectiveness as power reduction ratio
    if power_before > 0
        effectiveness = max(0, 1 - power_after / power_before);
    else
        effectiveness = 0;
    end
end

function power = calculate_vibration_power(data, fs, freq_range)
%CALCULATE_VIBRATION_POWER Calculate power in vibration frequency band.
    
    N = size(data, 1);
    
    % Simple periodogram for power calculation
    data_combined = sum(data, 2); % Combine all channels
    Y = fft(data_combined);
    P = abs(Y).^2 / (N * fs);
    
    freqs = (0:N-1) * fs / N;
    vib_indices = freqs >= freq_range(1) & freqs <= freq_range(2);
    
    power = sum(P(vib_indices));
end

function merged = merge_structs(struct1, struct2)
%MERGE_STRUCTS Merge two structures.
    
    merged = struct1;
    fields = fieldnames(struct2);
    
    for i = 1:length(fields)
        merged.(fields{i}) = struct2.(fields{i});
    end
end