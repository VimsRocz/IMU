function vibration_signals = generate_vibration_model(duration, fs, varargin)
%GENERATE_VIBRATION_MODEL Generate vibration signals for IMU simulation
%   VIBRATION_SIGNALS = GENERATE_VIBRATION_MODEL(DURATION, FS) generates
%   vibration signals for accelerometer and gyroscope with default parameters.
%
%   VIBRATION_SIGNALS = GENERATE_VIBRATION_MODEL(DURATION, FS, 'param', value, ...)
%   allows customization with parameter-value pairs:
%
%   Parameters:
%     'Type'           - Vibration type: 'sinusoidal', 'random', 'motor', 'rotor' (default: 'sinusoidal')
%     'Frequency'      - Base frequency in Hz (default: 50.0)
%     'AmplitudeAcc'   - Accelerometer amplitude in m/s² (default: 1.0)
%     'AmplitudeGyro'  - Gyroscope amplitude in rad/s (default: 0.1)
%     'NumBlades'      - Number of rotor blades for 'rotor' type (default: 4)
%     'Harmonics'      - Harmonic amplitudes for 'motor' type (default: [1.0, 0.5, 0.3, 0.2])
%     'CutoffFreq'     - Cutoff frequency for 'random' type in Hz (default: 100.0)
%     'Seed'           - Random seed for reproducibility (default: [])
%
%   Returns:
%     vibration_signals - Struct with fields:
%       .accel_vib - Nx3 accelerometer vibration signals
%       .gyro_vib  - Nx3 gyroscope vibration signals
%       .time      - Nx1 time vector
%       .fs        - Sampling frequency
%
%   Example:
%     % Generate 10 seconds of motor vibration at 400 Hz
%     vib = generate_vibration_model(10, 400, 'Type', 'motor', ...
%                                   'Frequency', 60, 'AmplitudeAcc', 2.0);
%     
%     % Plot results
%     figure; 
%     subplot(2,1,1); plot(vib.time, vib.accel_vib);
%     title('Accelerometer Vibration'); ylabel('Acceleration (m/s²)');
%     subplot(2,1,2); plot(vib.time, vib.gyro_vib);
%     title('Gyroscope Vibration'); ylabel('Angular Rate (rad/s)');

% Parse input arguments
p = inputParser;
addRequired(p, 'duration', @(x) isscalar(x) && x > 0);
addRequired(p, 'fs', @(x) isscalar(x) && x > 0);
addParameter(p, 'Type', 'sinusoidal', @(x) ischar(x) || isstring(x));
addParameter(p, 'Frequency', 50.0, @(x) isscalar(x) && x > 0);
addParameter(p, 'AmplitudeAcc', 1.0, @(x) isscalar(x) && x >= 0);
addParameter(p, 'AmplitudeGyro', 0.1, @(x) isscalar(x) && x >= 0);
addParameter(p, 'NumBlades', 4, @(x) isscalar(x) && x > 0);
addParameter(p, 'Harmonics', [1.0, 0.5, 0.3, 0.2], @(x) isnumeric(x) && all(x >= 0));
addParameter(p, 'CutoffFreq', 100.0, @(x) isscalar(x) && x > 0);
addParameter(p, 'Seed', [], @(x) isempty(x) || (isscalar(x) && x >= 0));

parse(p, duration, fs, varargin{:});

% Extract parameters
vibration_type = lower(string(p.Results.Type));
frequency = p.Results.Frequency;
amplitude_acc = p.Results.AmplitudeAcc;
amplitude_gyro = p.Results.AmplitudeGyro;
num_blades = p.Results.NumBlades;
harmonics = p.Results.Harmonics;
cutoff_freq = p.Results.CutoffFreq;
seed = p.Results.Seed;

% Set random seed if provided
if ~isempty(seed)
    rng(seed);
end

% Generate time vector
dt = 1.0 / fs;
t = (0:dt:(duration-dt))';
n_samples = length(t);

% Initialize output signals
accel_vib = zeros(n_samples, 3);
gyro_vib = zeros(n_samples, 3);

% Generate vibration based on type
switch vibration_type
    case 'sinusoidal'
        [accel_vib, gyro_vib] = generate_sinusoidal_vibration(t, frequency, amplitude_acc, amplitude_gyro);
        
    case 'random'
        [accel_vib, gyro_vib] = generate_random_vibration(n_samples, fs, cutoff_freq, amplitude_acc, amplitude_gyro);
        
    case 'motor'
        [accel_vib, gyro_vib] = generate_motor_vibration(t, frequency, harmonics, amplitude_acc, amplitude_gyro);
        
    case 'rotor'
        [accel_vib, gyro_vib] = generate_rotor_vibration(t, frequency, num_blades, amplitude_acc, amplitude_gyro);
        
    otherwise
        error('Unknown vibration type: %s', vibration_type);
end

% Pack results
vibration_signals = struct();
vibration_signals.accel_vib = accel_vib;
vibration_signals.gyro_vib = gyro_vib;
vibration_signals.time = t;
vibration_signals.fs = fs;

end

function [accel_vib, gyro_vib] = generate_sinusoidal_vibration(t, frequency, amplitude_acc, amplitude_gyro)
%GENERATE_SINUSOIDAL_VIBRATION Generate sinusoidal vibration signals
    n_samples = length(t);
    accel_vib = zeros(n_samples, 3);
    gyro_vib = zeros(n_samples, 3);
    
    % Phase offsets for each axis (120° apart)
    phase_acc = [0, pi/3, 2*pi/3];
    phase_gyro = [pi/4, pi/2, 3*pi/4];
    
    for axis = 1:3
        accel_vib(:, axis) = amplitude_acc * sin(2 * pi * frequency * t + phase_acc(axis));
        gyro_vib(:, axis) = amplitude_gyro * sin(2 * pi * frequency * t + phase_gyro(axis));
    end
end

function [accel_vib, gyro_vib] = generate_random_vibration(n_samples, fs, cutoff_freq, amplitude_acc, amplitude_gyro)
%GENERATE_RANDOM_VIBRATION Generate random vibration with spectral shaping
    accel_vib = zeros(n_samples, 3);
    gyro_vib = zeros(n_samples, 3);
    
    % Generate white noise
    accel_noise = randn(n_samples, 3);
    gyro_noise = randn(n_samples, 3);
    
    % Design low-pass filter
    nyquist = 0.5 * fs;
    normal_cutoff = cutoff_freq / nyquist;
    
    % Check if Signal Processing Toolbox is available
    if exist('butter', 'file') == 2 && exist('filtfilt', 'file') == 2
        [b, a] = butter(4, normal_cutoff, 'low');
        
        for axis = 1:3
            % Filter noise
            accel_filtered = filtfilt(b, a, accel_noise(:, axis));
            gyro_filtered = filtfilt(b, a, gyro_noise(:, axis));
            
            % Scale to desired amplitude
            accel_vib(:, axis) = amplitude_acc * accel_filtered / std(accel_filtered);
            gyro_vib(:, axis) = amplitude_gyro * gyro_filtered / std(gyro_filtered);
        end
    else
        % Fallback: simple moving average filter
        window_size = max(1, round(fs / cutoff_freq));
        
        for axis = 1:3
            accel_filtered = movmean(accel_noise(:, axis), window_size);
            gyro_filtered = movmean(gyro_noise(:, axis), window_size);
            
            accel_vib(:, axis) = amplitude_acc * accel_filtered / std(accel_filtered);
            gyro_vib(:, axis) = amplitude_gyro * gyro_filtered / std(gyro_filtered);
        end
    end
end

function [accel_vib, gyro_vib] = generate_motor_vibration(t, base_frequency, harmonics, amplitude_acc, amplitude_gyro)
%GENERATE_MOTOR_VIBRATION Generate motor-like vibration with harmonics
    n_samples = length(t);
    accel_vib = zeros(n_samples, 3);
    gyro_vib = zeros(n_samples, 3);
    
    % Different axis orientations for motor vibration
    axis_factors = [1.0, 0.7, 0.3;   % X-axis dominant
                    0.3, 1.0, 0.7;   % Y-axis dominant
                    0.7, 0.3, 1.0];  % Z-axis dominant
    
    for i = 1:length(harmonics)
        harmonic_amp = harmonics(i);
        freq = base_frequency * i;
        
        for axis = 1:3
            phase = 2 * pi * rand();  % Random phase
            factor = axis_factors(axis, axis);
            
            accel_vib(:, axis) = accel_vib(:, axis) + ...
                amplitude_acc * harmonic_amp * factor * sin(2 * pi * freq * t + phase);
            
            gyro_vib(:, axis) = gyro_vib(:, axis) + ...
                amplitude_gyro * harmonic_amp * factor * sin(2 * pi * freq * t + phase + pi/4);
        end
    end
end

function [accel_vib, gyro_vib] = generate_rotor_vibration(t, rotor_frequency, num_blades, amplitude_acc, amplitude_gyro)
%GENERATE_ROTOR_VIBRATION Generate quadrotor-like vibration patterns
    n_samples = length(t);
    accel_vib = zeros(n_samples, 3);
    gyro_vib = zeros(n_samples, 3);
    
    % Blade passing frequency
    blade_freq = rotor_frequency * num_blades;
    
    % Z-axis: dominant blade passing frequency
    accel_vib(:, 3) = amplitude_acc * sin(2 * pi * blade_freq * t);
    
    % X,Y axes: rotor frequency with some harmonics
    for axis = 1:2
        phase = (axis - 1) * pi/2;  % 90° phase difference between X and Y
        accel_vib(:, axis) = amplitude_acc * 0.6 * sin(2 * pi * rotor_frequency * t + phase);
    end
    
    % Gyro vibration couples with acceleration through rotor dynamics
    for axis = 1:3
        coupling_factor = 0.3;
        if axis == 3
            coupling_factor = 1.0;  % Z-axis more affected
        end
        
        gyro_vib(:, axis) = amplitude_gyro * coupling_factor * ...
            sin(2 * pi * rotor_frequency * t + (axis-1) * pi/3 + pi/6);
    end
end