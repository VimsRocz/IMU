function result = vibration_detection(accel, gyro, fs, varargin)
%VIBRATION_DETECTION Detect presence of vibration in IMU data.
%   RESULT = VIBRATION_DETECTION(ACCEL, GYRO, FS) analyzes accelerometer
%   and gyroscope data to detect the presence of vibration signals.
%
%   RESULT = VIBRATION_DETECTION(ACCEL, GYRO, FS, 'PARAM', VALUE, ...)
%   allows specification of detection parameters.
%
%   Inputs:
%       ACCEL - Accelerometer data (Nx3) in m/s^2
%       GYRO  - Gyroscope data (Nx3) in rad/s
%       FS    - Sampling frequency in Hz
%
%   Parameters:
%       'Method'           - Detection method: 'frequency', 'variance', 'energy', 'combined' (default: 'combined')
%       'FreqRange'        - Frequency range for vibration [low, high] in Hz (default: [10, 200])
%       'WindowSize'       - Window size for analysis in samples (default: fs/2)
%       'OverlapFactor'    - Overlap factor for windowing (default: 0.5)
%       'ThresholdFactor'  - Factor for automatic threshold (default: 2.0)
%       'MinDuration'      - Minimum vibration duration in seconds (default: 0.1)
%
%   Output:
%       RESULT - Structure with fields:
%           .vibration_detected - Boolean indicating if vibration is present
%           .vibration_flag     - Time series flag (Nx1) indicating vibration periods
%           .dominant_freq      - Dominant vibration frequency in Hz
%           .power_ratio        - Ratio of vibration band power to total power
%           .detection_method   - Method used for detection
%           .confidence         - Detection confidence (0-1)
%           .statistics         - Additional detection statistics
%
%   Examples:
%       % Basic detection
%       result = vibration_detection(accel, gyro, 400);
%
%       % Detection with custom frequency range
%       result = vibration_detection(accel, gyro, 400, ...
%                   'FreqRange', [20, 150], 'Method', 'frequency');
%
%   See also VIBRATION_MODEL, VIBRATION_COMPENSATION.

    % Parse input arguments
    p = inputParser;
    addRequired(p, 'accel');
    addRequired(p, 'gyro');
    addRequired(p, 'fs');
    addParameter(p, 'Method', 'combined', @(x) any(validatestring(x, {'frequency', 'variance', 'energy', 'combined'})));
    addParameter(p, 'FreqRange', [10, 200], @(x) isnumeric(x) && length(x) == 2 && x(2) > x(1));
    addParameter(p, 'WindowSize', [], @isnumeric);
    addParameter(p, 'OverlapFactor', 0.5, @(x) isnumeric(x) && x >= 0 && x < 1);
    addParameter(p, 'ThresholdFactor', 2.0, @(x) isnumeric(x) && x > 0);
    addParameter(p, 'MinDuration', 0.1, @(x) isnumeric(x) && x > 0);
    
    parse(p, accel, gyro, fs, varargin{:});
    
    method = p.Results.Method;
    freq_range = p.Results.FreqRange;
    window_size = p.Results.WindowSize;
    overlap_factor = p.Results.OverlapFactor;
    threshold_factor = p.Results.ThresholdFactor;
    min_duration = p.Results.MinDuration;
    
    if isempty(window_size)
        window_size = round(fs / 2); % 0.5 second window
    end
    
    % Ensure data is in correct format
    if size(accel, 1) < size(accel, 2)
        accel = accel';
    end
    if size(gyro, 1) < size(gyro, 2)
        gyro = gyro';
    end
    
    N = size(accel, 1);
    
    % Initialize result structure
    result = struct();
    result.detection_method = method;
    result.vibration_flag = false(N, 1);
    result.dominant_freq = NaN;
    result.power_ratio = 0;
    result.confidence = 0;
    result.statistics = struct();
    
    switch lower(method)
        case 'frequency'
            result = detect_frequency_domain(accel, gyro, fs, freq_range, window_size, overlap_factor, threshold_factor, result);
            
        case 'variance'
            result = detect_variance_based(accel, gyro, fs, window_size, threshold_factor, result);
            
        case 'energy'
            result = detect_energy_based(accel, gyro, fs, freq_range, window_size, threshold_factor, result);
            
        case 'combined'
            % Use multiple methods and combine results
            result_freq = detect_frequency_domain(accel, gyro, fs, freq_range, window_size, overlap_factor, threshold_factor, result);
            result_var = detect_variance_based(accel, gyro, fs, window_size, threshold_factor, result);
            result_energy = detect_energy_based(accel, gyro, fs, freq_range, window_size, threshold_factor, result);
            
            result = combine_detection_results(result_freq, result_var, result_energy);
    end
    
    % Apply minimum duration filter
    result.vibration_flag = apply_min_duration_filter(result.vibration_flag, fs, min_duration);
    result.vibration_detected = any(result.vibration_flag);
    
    % Calculate overall confidence
    if strcmp(method, 'combined')
        % Already computed in combine_detection_results
    else
        result.confidence = min(1.0, result.power_ratio * 2); % Simple confidence metric
    end
end

function result = detect_frequency_domain(accel, gyro, fs, freq_range, window_size, overlap_factor, threshold_factor, result)
%DETECT_FREQUENCY_DOMAIN Frequency domain vibration detection.
    
    N = size(accel, 1);
    hop_size = round(window_size * (1 - overlap_factor));
    num_windows = floor((N - window_size) / hop_size) + 1;
    
    vibration_flag = false(N, 1);
    dominant_freqs = [];
    power_ratios = [];
    
    for i = 1:num_windows
        start_idx = (i-1) * hop_size + 1;
        end_idx = min(start_idx + window_size - 1, N);
        
        % Extract window data
        accel_win = accel(start_idx:end_idx, :);
        gyro_win = gyro(start_idx:end_idx, :);
        
        % Compute power spectral density
        [psd_accel, freqs] = compute_psd([accel_win, gyro_win], fs);
        
        % Find vibration band indices
        vib_idx = freqs >= freq_range(1) & freqs <= freq_range(2);
        
        % Calculate power in vibration band vs total power
        power_vib = sum(psd_accel(vib_idx));
        power_total = sum(psd_accel);
        power_ratio = power_vib / power_total;
        
        % Find dominant frequency
        [~, max_idx] = max(psd_accel(vib_idx));
        vib_freqs = freqs(vib_idx);
        if ~isempty(max_idx) && max_idx <= length(vib_freqs)
            dominant_freq = vib_freqs(max_idx);
        else
            dominant_freq = NaN;
        end
        
        % Threshold-based detection
        baseline_power = median(psd_accel(~vib_idx));
        vibration_detected = power_ratio > 0.1 && power_vib > threshold_factor * baseline_power;
        
        if vibration_detected
            vibration_flag(start_idx:end_idx) = true;
            dominant_freqs(end+1) = dominant_freq;
            power_ratios(end+1) = power_ratio;
        end
    end
    
    result.vibration_flag = vibration_flag;
    if ~isempty(dominant_freqs)
        result.dominant_freq = median(dominant_freqs);
        result.power_ratio = median(power_ratios);
    else
        result.dominant_freq = NaN;
        result.power_ratio = 0;
    end
    
    result.statistics.freq_method = struct();
    result.statistics.freq_method.num_windows = num_windows;
    result.statistics.freq_method.detection_rate = sum(vibration_flag) / N;
end

function result = detect_variance_based(accel, gyro, fs, window_size, threshold_factor, result)
%DETECT_VARIANCE_BASED Variance-based vibration detection.
    
    N = size(accel, 1);
    
    % Calculate combined sensor magnitude
    accel_mag = sqrt(sum(accel.^2, 2));
    gyro_mag = sqrt(sum(gyro.^2, 2));
    
    % Compute sliding variance
    accel_var = sliding_variance_1d(accel_mag, window_size);
    gyro_var = sliding_variance_1d(gyro_mag, window_size);
    
    % Normalize variances
    accel_var = accel_var / median(accel_var);
    gyro_var = gyro_var / median(gyro_var);
    
    % Detection based on variance thresholds
    vibration_flag = (accel_var > threshold_factor) | (gyro_var > threshold_factor);
    
    % Pad to original length
    pad_size = window_size - 1;
    vibration_flag = [false(pad_size, 1); vibration_flag];
    if length(vibration_flag) > N
        vibration_flag = vibration_flag(1:N);
    end
    
    result.vibration_flag = vibration_flag;
    result.power_ratio = sum(vibration_flag) / N;
    result.statistics.var_method = struct();
    result.statistics.var_method.accel_var_mean = mean(accel_var);
    result.statistics.var_method.gyro_var_mean = mean(gyro_var);
end

function result = detect_energy_based(accel, gyro, fs, freq_range, window_size, threshold_factor, result)
%DETECT_ENERGY_BASED Energy-based vibration detection.
    
    % Apply band-pass filter to isolate vibration frequencies
    if exist('butter', 'file') == 2 && exist('filtfilt', 'file') == 2
        nyq = fs / 2;
        [b, a] = butter(4, freq_range / nyq, 'bandpass');
        accel_filt = filtfilt(b, a, accel);
        gyro_filt = filtfilt(b, a, gyro);
    else
        % Fallback: use original data
        accel_filt = accel;
        gyro_filt = gyro;
    end
    
    % Calculate instantaneous energy
    energy_accel = sum(accel_filt.^2, 2);
    energy_gyro = sum(gyro_filt.^2, 2);
    total_energy = energy_accel + energy_gyro;
    
    % Smooth energy signal
    if window_size > 1
        total_energy = smooth_signal(total_energy, window_size);
    end
    
    % Threshold-based detection
    energy_threshold = threshold_factor * median(total_energy);
    vibration_flag = total_energy > energy_threshold;
    
    result.vibration_flag = vibration_flag;
    result.power_ratio = sum(vibration_flag) / length(vibration_flag);
    result.statistics.energy_method = struct();
    result.statistics.energy_method.mean_energy = mean(total_energy);
    result.statistics.energy_method.energy_threshold = energy_threshold;
end

function result = combine_detection_results(result_freq, result_var, result_energy)
%COMBINE_DETECTION_RESULTS Combine results from multiple detection methods.
    
    result = result_freq;  % Use frequency domain result as base
    result.detection_method = 'combined';
    
    % Combine vibration flags using voting
    flags = [result_freq.vibration_flag, result_var.vibration_flag, result_energy.vibration_flag];
    vote_sum = sum(flags, 2);
    result.vibration_flag = vote_sum >= 2;  % Majority vote
    
    % Combine power ratios
    power_ratios = [result_freq.power_ratio, result_var.power_ratio, result_energy.power_ratio];
    result.power_ratio = mean(power_ratios);
    
    % Calculate combined confidence
    confidences = [min(1.0, result_freq.power_ratio * 2), ...
                   min(1.0, result_var.power_ratio * 2), ...
                   min(1.0, result_energy.power_ratio * 2)];
    result.confidence = mean(confidences);
    
    % Keep statistics from all methods
    result.statistics.combined = struct();
    result.statistics.combined.method_agreement = sum(vote_sum == 3) / length(vote_sum);
    result.statistics.combined.individual_confidences = confidences;
end

function [psd, freqs] = compute_psd(data, fs)
%COMPUTE_PSD Compute power spectral density.
    
    [N, num_channels] = size(data);
    
    % Use Welch's method if available, otherwise simple periodogram
    if exist('pwelch', 'file') == 2
        [psd_temp, freqs] = pwelch(data(:,1), [], [], [], fs);
        psd = zeros(length(psd_temp), num_channels);
        for ch = 1:num_channels
            [psd(:,ch), ~] = pwelch(data(:,ch), [], [], [], fs);
        end
        % Sum across all channels
        psd = sum(psd, 2);
    else
        % Simple periodogram fallback
        data_combined = sum(data, 2);  % Combine channels
        Y = fft(data_combined);
        P = abs(Y).^2 / (N * fs);
        psd = P(1:floor(N/2)+1);
        psd(2:end-1) = 2 * psd(2:end-1);  % Single-sided
        freqs = (0:floor(N/2)) * fs / N;
        freqs = freqs';
    end
end

function var_out = sliding_variance_1d(data, window_size)
%SLIDING_VARIANCE_1D Compute sliding variance of 1D signal.
    
    N = length(data);
    var_out = zeros(N - window_size + 1, 1);
    
    for i = 1:(N - window_size + 1)
        window_data = data(i:i+window_size-1);
        var_out(i) = var(window_data);
    end
end

function smoothed = smooth_signal(signal, window_size)
%SMOOTH_SIGNAL Simple moving average smoothing.
    
    if window_size <= 1
        smoothed = signal;
        return;
    end
    
    if exist('movmean', 'file') == 2
        smoothed = movmean(signal, window_size);
    else
        % Manual implementation
        N = length(signal);
        smoothed = zeros(size(signal));
        half_win = floor(window_size / 2);
        
        for i = 1:N
            start_idx = max(1, i - half_win);
            end_idx = min(N, i + half_win);
            smoothed(i) = mean(signal(start_idx:end_idx));
        end
    end
end

function filtered_flag = apply_min_duration_filter(vibration_flag, fs, min_duration)
%APPLY_MIN_DURATION_FILTER Remove short vibration detections.
    
    min_samples = round(min_duration * fs);
    
    % Find vibration segments
    d = diff([false; vibration_flag; false]);
    start_indices = find(d == 1);
    end_indices = find(d == -1) - 1;
    
    filtered_flag = false(size(vibration_flag));
    
    for i = 1:length(start_indices)
        segment_length = end_indices(i) - start_indices(i) + 1;
        if segment_length >= min_samples
            filtered_flag(start_indices(i):end_indices(i)) = true;
        end
    end
end