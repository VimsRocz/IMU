function result = compensate_vibration(imu_data, fs, varargin)
%COMPENSATE_VIBRATION Compensate vibration in IMU data using various methods
%   RESULT = COMPENSATE_VIBRATION(IMU_DATA, FS) compensates vibration in IMU_DATA
%   using default Butterworth filtering at 400 Hz sampling rate.
%
%   RESULT = COMPENSATE_VIBRATION(IMU_DATA, FS, 'param', value, ...) allows
%   customization with parameter-value pairs:
%
%   Parameters:
%     'Method'         - Compensation method: 'butterworth', 'notch', 'kalman' (default: 'butterworth')
%     'CutoffFreq'     - Cutoff frequency for Butterworth filter in Hz (default: 20.0)
%     'NotchFreq'      - Notch frequency in Hz (default: 50.0)
%     'QualityFactor'  - Quality factor for notch filter (default: 30.0)
%     'FilterOrder'    - Filter order for Butterworth filter (default: 4)
%     'ProcessNoise'   - Process noise variance for Kalman filter (default: 1e-4)
%     'MeasNoise'      - Measurement noise variance for Kalman filter (default: 1e-2)
%
%   Input:
%     imu_data - Nx3 matrix of IMU measurements (accelerometer or gyroscope)
%     fs       - Sampling frequency in Hz
%
%   Returns:
%     result - Struct with fields:
%       .filtered_data - Nx3 compensated IMU data
%       .method        - Compensation method used
%       .parameters    - Parameters used for compensation
%
%   Examples:
%     % Butterworth low-pass filtering
%     result = compensate_vibration(accel_data, 400, 'Method', 'butterworth', 'CutoffFreq', 15);
%     
%     % Notch filtering at 60 Hz
%     result = compensate_vibration(accel_data, 400, 'Method', 'notch', 'NotchFreq', 60);
%     
%     % Kalman filtering
%     result = compensate_vibration(accel_data, 400, 'Method', 'kalman');

% Parse input arguments
p = inputParser;
addRequired(p, 'imu_data', @(x) isnumeric(x) && size(x,2) == 3);
addRequired(p, 'fs', @(x) isscalar(x) && x > 0);
addParameter(p, 'Method', 'butterworth', @(x) ischar(x) || isstring(x));
addParameter(p, 'CutoffFreq', 20.0, @(x) isscalar(x) && x > 0);
addParameter(p, 'NotchFreq', 50.0, @(x) isscalar(x) && x > 0);
addParameter(p, 'QualityFactor', 30.0, @(x) isscalar(x) && x > 0);
addParameter(p, 'FilterOrder', 4, @(x) isscalar(x) && x > 0);
addParameter(p, 'ProcessNoise', 1e-4, @(x) isscalar(x) && x > 0);
addParameter(p, 'MeasNoise', 1e-2, @(x) isscalar(x) && x > 0);

parse(p, imu_data, fs, varargin{:});

% Extract parameters
method = lower(string(p.Results.Method));
cutoff_freq = p.Results.CutoffFreq;
notch_freq = p.Results.NotchFreq;
quality_factor = p.Results.QualityFactor;
filter_order = p.Results.FilterOrder;
process_noise = p.Results.ProcessNoise;
meas_noise = p.Results.MeasNoise;

[n_samples, n_axes] = size(imu_data);

% Apply compensation based on method
switch method
    case 'butterworth'
        [filtered_data, params] = apply_butterworth_filter(imu_data, fs, cutoff_freq, filter_order);
        
    case 'notch'
        [filtered_data, params] = apply_notch_filter(imu_data, fs, notch_freq, quality_factor);
        
    case 'kalman'
        [filtered_data, params] = apply_kalman_filter(imu_data, fs, process_noise, meas_noise);
        
    otherwise
        error('Unknown compensation method: %s', method);
end

% Pack results
result = struct();
result.filtered_data = filtered_data;
result.method = char(method);
result.parameters = params;

end

function [filtered_data, params] = apply_butterworth_filter(imu_data, fs, cutoff_freq, filter_order)
%APPLY_BUTTERWORTH_FILTER Apply Butterworth low-pass filter
    [n_samples, n_axes] = size(imu_data);
    filtered_data = zeros(n_samples, n_axes);
    
    nyquist = 0.5 * fs;
    normal_cutoff = cutoff_freq / nyquist;
    
    % Check if Signal Processing Toolbox is available
    if exist('butter', 'file') == 2 && exist('filtfilt', 'file') == 2
        [b, a] = butter(filter_order, normal_cutoff, 'low');
        
        for axis = 1:n_axes
            filtered_data(:, axis) = filtfilt(b, a, imu_data(:, axis));
        end
    else
        % Fallback: moving average filter
        window_size = max(1, round(fs / cutoff_freq));
        fprintf('Warning: Signal Processing Toolbox not available. Using moving average filter.\n');
        
        for axis = 1:n_axes
            filtered_data(:, axis) = movmean(imu_data(:, axis), window_size);
        end
    end
    
    params = struct('cutoff_frequency', cutoff_freq, 'filter_order', filter_order, ...
                   'filter_type', 'butterworth');
end

function [filtered_data, params] = apply_notch_filter(imu_data, fs, notch_freq, quality_factor)
%APPLY_NOTCH_FILTER Apply notch filter to remove specific frequency
    [n_samples, n_axes] = size(imu_data);
    filtered_data = zeros(n_samples, n_axes);
    
    % Check if Signal Processing Toolbox is available
    if exist('iirnotch', 'file') == 2 && exist('filtfilt', 'file') == 2
        % Design notch filter
        w0 = notch_freq / (fs/2);  % Normalized frequency
        bw = w0 / quality_factor;  % Bandwidth
        [b, a] = iirnotch(w0, bw);
        
        for axis = 1:n_axes
            filtered_data(:, axis) = filtfilt(b, a, imu_data(:, axis));
        end
    else
        % Fallback: Simple frequency domain filtering
        fprintf('Warning: Signal Processing Toolbox not available. Using frequency domain filtering.\n');
        
        for axis = 1:n_axes
            % FFT-based notch filtering
            Y = fft(imu_data(:, axis));
            freqs = (0:n_samples-1) * fs / n_samples;
            
            % Create notch filter in frequency domain
            notch_width = fs / quality_factor;
            notch_mask = abs(freqs - notch_freq) > notch_width/2 & ...
                        abs(freqs - (fs - notch_freq)) > notch_width/2;
            
            Y_filtered = Y .* notch_mask';
            filtered_data(:, axis) = real(ifft(Y_filtered));
        end
    end
    
    params = struct('notch_frequency', notch_freq, 'quality_factor', quality_factor, ...
                   'filter_type', 'notch');
end

function [filtered_data, params] = apply_kalman_filter(imu_data, fs, process_noise, meas_noise)
%APPLY_KALMAN_FILTER Apply Kalman filter for vibration compensation
    [n_samples, n_axes] = size(imu_data);
    filtered_data = zeros(n_samples, n_axes);
    
    dt = 1.0 / fs;
    
    for axis = 1:n_axes
        % Initialize Kalman filter states
        x = zeros(2, 1);  % State: [position, velocity]
        P = eye(2);       % Covariance matrix
        
        % State transition matrix (constant velocity model)
        F = [1, dt; 0, 1];
        
        % Measurement matrix (observe position only)
        H = [1, 0];
        
        % Process noise covariance
        Q = process_noise * [dt^4/4, dt^3/2; dt^3/2, dt^2];
        
        % Measurement noise covariance
        R = meas_noise;
        
        % Kalman filtering
        for k = 1:n_samples
            % Prediction step
            x = F * x;
            P = F * P * F' + Q;
            
            % Update step
            z = imu_data(k, axis);          % Measurement
            y = z - H * x;                  % Innovation
            S = H * P * H' + R;             % Innovation covariance
            K = P * H' / S;                 % Kalman gain
            
            x = x + K * y;
            P = P - K * H * P;
            
            filtered_data(k, axis) = x(1);
        end
    end
    
    params = struct('process_noise', process_noise, 'measurement_noise', meas_noise, ...
                   'filter_type', 'kalman');
end