function result = detect_vibration(imu_data, fs, varargin)
%DETECT_VIBRATION Detect vibration in IMU data using statistical or frequency methods
%   RESULT = DETECT_VIBRATION(IMU_DATA, FS) detects vibration in IMU_DATA
%   using statistical analysis at 400 Hz sampling rate.
%
%   RESULT = DETECT_VIBRATION(IMU_DATA, FS, 'param', value, ...) allows
%   customization with parameter-value pairs:
%
%   Parameters:
%     'Method'         - Detection method: 'statistical', 'frequency' (default: 'statistical')
%     'WindowSize'     - Window size for analysis in samples (default: 200)
%     'ThresholdFactor'- Threshold multiplier for statistical method (default: 2.0)
%     'FreqRange'      - Frequency range for vibration [low, high] in Hz (default: [20, 200])
%     'OverlapRatio'   - Window overlap ratio for frequency method (default: 0.5)
%
%   Input:
%     imu_data - Nx3 matrix of IMU measurements (accelerometer or gyroscope)
%     fs       - Sampling frequency in Hz
%
%   Returns:
%     result - Struct with fields:
%       .vibration_detected  - Nx1 logical array indicating vibration presence
%       .vibration_intensity - Nx1 array of vibration intensity measure
%       .method              - Detection method used
%       .parameters          - Detection parameters
%       .summary             - Summary statistics
%
%   Examples:
%     % Statistical vibration detection
%     result = detect_vibration(accel_data, 400, 'Method', 'statistical');
%     vibration_ratio = sum(result.vibration_detected) / length(result.vibration_detected);
%     
%     % Frequency-based detection
%     result = detect_vibration(accel_data, 400, 'Method', 'frequency', ...
%                              'FreqRange', [50, 150]);

% Parse input arguments
p = inputParser;
addRequired(p, 'imu_data', @(x) isnumeric(x) && size(x,2) == 3);
addRequired(p, 'fs', @(x) isscalar(x) && x > 0);
addParameter(p, 'Method', 'statistical', @(x) ischar(x) || isstring(x));
addParameter(p, 'WindowSize', 200, @(x) isscalar(x) && x > 0);
addParameter(p, 'ThresholdFactor', 2.0, @(x) isscalar(x) && x > 0);
addParameter(p, 'FreqRange', [20, 200], @(x) isnumeric(x) && length(x) == 2 && x(1) < x(2));
addParameter(p, 'OverlapRatio', 0.5, @(x) isscalar(x) && x >= 0 && x < 1);

parse(p, imu_data, fs, varargin{:});

% Extract parameters
method = lower(string(p.Results.Method));
window_size = p.Results.WindowSize;
threshold_factor = p.Results.ThresholdFactor;
freq_range = p.Results.FreqRange;
overlap_ratio = p.Results.OverlapRatio;

[n_samples, n_axes] = size(imu_data);

% Apply detection based on method
switch method
    case 'statistical'
        [vibration_detected, vibration_intensity, params, summary] = ...
            detect_statistical(imu_data, window_size, threshold_factor);
        
    case 'frequency'
        [vibration_detected, vibration_intensity, params, summary] = ...
            detect_frequency(imu_data, fs, freq_range, window_size, overlap_ratio, threshold_factor);
        
    otherwise
        error('Unknown detection method: %s', method);
end

% Pack results
result = struct();
result.vibration_detected = vibration_detected;
result.vibration_intensity = vibration_intensity;
result.method = char(method);
result.parameters = params;
result.summary = summary;

end

function [vibration_detected, vibration_intensity, params, summary] = ...
    detect_statistical(imu_data, window_size, threshold_factor)
%DETECT_STATISTICAL Statistical vibration detection using variance analysis
    
    [n_samples, n_axes] = size(imu_data);
    vibration_detected = false(n_samples, 1);
    vibration_intensity = zeros(n_samples, 1);
    
    % Compute baseline statistics from first portion of data
    baseline_samples = min(window_size * 2, floor(n_samples / 4));
    baseline_var = var(imu_data(1:baseline_samples, :), 0, 1);
    threshold = threshold_factor * mean(baseline_var);
    
    % Sliding window analysis
    for i = window_size+1:n_samples
        window_data = imu_data(i-window_size:i-1, :);
        window_var = var(window_data, 0, 1);
        avg_var = mean(window_var);
        
        vibration_intensity(i) = avg_var / mean(baseline_var);
        vibration_detected(i) = avg_var > threshold;
    end
    
    % Calculate summary statistics
    vibration_ratio = sum(vibration_detected) / n_samples;
    avg_intensity = mean(vibration_intensity(vibration_detected));
    if isnan(avg_intensity)
        avg_intensity = 0;
    end
    
    params = struct('window_size', window_size, 'threshold_factor', threshold_factor, ...
                   'threshold_value', threshold, 'baseline_variance', baseline_var);
    
    summary = struct('vibration_ratio', vibration_ratio, ...
                    'average_intensity', avg_intensity, ...
                    'peak_intensity', max(vibration_intensity));
end

function [vibration_detected, vibration_intensity, params, summary] = ...
    detect_frequency(imu_data, fs, freq_range, window_size, overlap_ratio, threshold_factor)
%DETECT_FREQUENCY Frequency-domain vibration detection using FFT analysis
    
    [n_samples, n_axes] = size(imu_data);
    hop_size = floor(window_size * (1 - overlap_ratio));
    n_windows = floor((n_samples - window_size) / hop_size) + 1;
    
    vibration_detected = false(n_samples, 1);
    vibration_intensity = zeros(n_samples, 1);
    dominant_frequency = zeros(n_samples, 1);
    
    % Define frequency bins
    if window_size > n_samples
        window_size = n_samples;
    end
    
    dt = 1.0 / fs;
    freqs = (0:floor(window_size/2)) * fs / window_size;
    freq_mask = (freqs >= freq_range(1)) & (freqs <= freq_range(2));
    
    % Check if we have frequency bins in the specified range
    if ~any(freq_mask)
        warning('No frequency bins in specified range [%.1f, %.1f] Hz', freq_range(1), freq_range(2));
        params = struct('freq_range', freq_range, 'window_size', window_size, ...
                       'overlap_ratio', overlap_ratio);
        summary = struct('vibration_ratio', 0, 'average_intensity', 0, 'peak_intensity', 0);
        return;
    end
    
    % Analyze each window
    for win_idx = 1:n_windows
        start_idx = (win_idx - 1) * hop_size + 1;
        end_idx = start_idx + window_size - 1;
        
        if end_idx > n_samples
            break;
        end
        
        window_data = imu_data(start_idx:end_idx, :);
        
        % Compute power spectral density
        total_vibration_power = 0;
        peak_frequencies = [];
        baseline_power = 0;
        
        for axis = 1:n_axes
            % Apply windowing to reduce spectral leakage
            if exist('hann', 'file') == 2
                window_func = hann(window_size);
                windowed_data = window_data(:, axis) .* window_func;
            else
                % Simple Hann window implementation
                n = 0:window_size-1;
                window_func = 0.5 * (1 - cos(2*pi*n'/(window_size-1)));
                windowed_data = window_data(:, axis) .* window_func;
            end
            
            % Compute FFT
            fft_data = fft(windowed_data);
            psd = abs(fft_data(1:floor(window_size/2)+1)).^2;
            
            % Extract vibration frequency range
            vibration_psd = psd(freq_mask);
            total_vibration_power = total_vibration_power + sum(vibration_psd);
            
            % Find dominant frequency in vibration range
            if ~isempty(vibration_psd)
                [~, peak_idx] = max(vibration_psd);
                vibration_freqs = freqs(freq_mask);
                if ~isempty(vibration_freqs)
                    peak_freq = vibration_freqs(peak_idx);
                    peak_frequencies(end+1) = peak_freq; %#ok<AGROW>
                end
            end
            
            % Compute baseline power (DC and very low frequencies)
            baseline_mask = freqs < freq_range(1);
            if any(baseline_mask)
                baseline_power = baseline_power + sum(psd(baseline_mask));
            end
        end
        
        % Detection logic
        vibration_ratio = total_vibration_power / (baseline_power + 1e-10);
        is_vibration = vibration_ratio > threshold_factor;
        
        % Fill results for this window
        center_idx = start_idx + floor(window_size / 2);
        if center_idx <= n_samples
            vibration_detected(start_idx:min(end_idx, n_samples)) = is_vibration;
            vibration_intensity(center_idx) = total_vibration_power;
            if ~isempty(peak_frequencies)
                dominant_frequency(center_idx) = mean(peak_frequencies);
            end
        end
    end
    
    % Calculate summary statistics
    vibration_ratio = sum(vibration_detected) / n_samples;
    avg_intensity = mean(vibration_intensity(vibration_detected));
    if isnan(avg_intensity)
        avg_intensity = 0;
    end
    
    % Find most common dominant frequency
    dom_freqs = dominant_frequency(dominant_frequency > 0);
    if ~isempty(dom_freqs)
        dominant_freq = median(dom_freqs);
    else
        dominant_freq = 0;
    end
    
    params = struct('freq_range', freq_range, 'window_size', window_size, ...
                   'overlap_ratio', overlap_ratio, 'threshold_factor', threshold_factor);
    
    summary = struct('vibration_ratio', vibration_ratio, ...
                    'average_intensity', avg_intensity, ...
                    'peak_intensity', max(vibration_intensity), ...
                    'dominant_frequency', dominant_freq);
end