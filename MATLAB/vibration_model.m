function vibration_data = vibration_model(time, vib_type, vib_params)
%VIBRATION_MODEL Generate vibration signals for IMU simulation.
%   VIBRATION_DATA = VIBRATION_MODEL(TIME, VIB_TYPE, VIB_PARAMS) generates
%   vibration signals that can be added to clean IMU data to simulate the
%   effect of vibration on accelerometer and gyroscope measurements.
%
%   Inputs:
%       TIME       - Time vector (Nx1) in seconds
%       VIB_TYPE   - String specifying vibration type:
%                    'sinusoidal' - Pure sinusoidal vibrations
%                    'motor'      - Motor-induced vibrations (multiple harmonics)
%                    'random'     - Random vibration with specified bandwidth
%                    'rotor'      - Quadcopter rotor vibrations
%       VIB_PARAMS - Structure with vibration parameters (see examples below)
%
%   Output:
%       VIBRATION_DATA - Structure with fields:
%           .accel - Vibration acceleration (Nx3) in m/s^2
%           .gyro  - Vibration angular velocity (Nx3) in rad/s
%           .freq  - Dominant frequency in Hz
%           .type  - Vibration type used
%
%   Examples:
%       % Sinusoidal vibration at 50 Hz
%       t = (0:0.0025:10)';  % 400 Hz sampling
%       params.frequency = 50;
%       params.amplitude_accel = [0.5, 0.3, 0.2];  % m/s^2
%       params.amplitude_gyro = [0.1, 0.05, 0.02]; % rad/s
%       vib = vibration_model(t, 'sinusoidal', params);
%
%       % Motor vibration with harmonics
%       params.base_freq = 30;
%       params.harmonics = [1, 2, 3];
%       params.harmonic_weights = [1.0, 0.5, 0.2];
%       params.amplitude_accel = [1.0, 0.8, 0.6];
%       params.amplitude_gyro = [0.2, 0.15, 0.1];
%       vib = vibration_model(t, 'motor', params);
%
%   See also VIBRATION_DETECTION, VIBRATION_COMPENSATION.

    if nargin < 3
        error('vibration_model:MissingParams', 'All three arguments required');
    end
    
    % Initialize output structure
    N = length(time);
    vibration_data = struct();
    vibration_data.accel = zeros(N, 3);
    vibration_data.gyro = zeros(N, 3);
    vibration_data.type = vib_type;
    
    switch lower(vib_type)
        case 'sinusoidal'
            [accel_vib, gyro_vib, freq] = generate_sinusoidal(time, vib_params);
            
        case 'motor'
            [accel_vib, gyro_vib, freq] = generate_motor_vibration(time, vib_params);
            
        case 'random'
            [accel_vib, gyro_vib, freq] = generate_random_vibration(time, vib_params);
            
        case 'rotor'
            [accel_vib, gyro_vib, freq] = generate_rotor_vibration(time, vib_params);
            
        otherwise
            error('vibration_model:InvalidType', ...
                'Unknown vibration type: %s. Valid types: sinusoidal, motor, random, rotor', vib_type);
    end
    
    vibration_data.accel = accel_vib;
    vibration_data.gyro = gyro_vib;
    vibration_data.freq = freq;
end

function [accel_vib, gyro_vib, freq] = generate_sinusoidal(time, params)
%GENERATE_SINUSOIDAL Pure sinusoidal vibration.
    
    % Set default parameters
    if ~isfield(params, 'frequency')
        params.frequency = 50; % Hz
    end
    if ~isfield(params, 'amplitude_accel')
        params.amplitude_accel = [0.5, 0.3, 0.2]; % m/s^2
    end
    if ~isfield(params, 'amplitude_gyro')
        params.amplitude_gyro = [0.1, 0.05, 0.02]; % rad/s
    end
    if ~isfield(params, 'phase')
        params.phase = [0, pi/4, pi/2]; % rad
    end
    
    freq = params.frequency;
    omega = 2 * pi * freq;
    
    % Generate sinusoidal signals for each axis
    N = length(time);
    accel_vib = zeros(N, 3);
    gyro_vib = zeros(N, 3);
    
    for axis = 1:3
        accel_vib(:, axis) = params.amplitude_accel(axis) * ...
                             sin(omega * time + params.phase(axis));
        gyro_vib(:, axis) = params.amplitude_gyro(axis) * ...
                           sin(omega * time + params.phase(axis) + pi/3);
    end
end

function [accel_vib, gyro_vib, freq] = generate_motor_vibration(time, params)
%GENERATE_MOTOR_VIBRATION Motor-induced vibrations with harmonics.
    
    % Set default parameters
    if ~isfield(params, 'base_freq')
        params.base_freq = 30; % Hz
    end
    if ~isfield(params, 'harmonics')
        params.harmonics = [1, 2, 3, 4]; % Harmonic numbers
    end
    if ~isfield(params, 'harmonic_weights')
        params.harmonic_weights = [1.0, 0.6, 0.3, 0.15];
    end
    if ~isfield(params, 'amplitude_accel')
        params.amplitude_accel = [1.0, 0.8, 0.6];
    end
    if ~isfield(params, 'amplitude_gyro')
        params.amplitude_gyro = [0.2, 0.15, 0.1];
    end
    
    freq = params.base_freq;
    N = length(time);
    accel_vib = zeros(N, 3);
    gyro_vib = zeros(N, 3);
    
    % Generate harmonics
    for h = 1:length(params.harmonics)
        harmonic_freq = params.base_freq * params.harmonics(h);
        omega = 2 * pi * harmonic_freq;
        weight = params.harmonic_weights(h);
        
        for axis = 1:3
            % Random phase for each harmonic and axis
            phase = 2 * pi * rand();
            accel_vib(:, axis) = accel_vib(:, axis) + ...
                weight * params.amplitude_accel(axis) * sin(omega * time + phase);
            gyro_vib(:, axis) = gyro_vib(:, axis) + ...
                weight * params.amplitude_gyro(axis) * sin(omega * time + phase + pi/6);
        end
    end
end

function [accel_vib, gyro_vib, freq] = generate_random_vibration(time, params)
%GENERATE_RANDOM_VIBRATION Band-limited random vibration.
    
    % Set default parameters
    if ~isfield(params, 'freq_band')
        params.freq_band = [20, 100]; % Hz
    end
    if ~isfield(params, 'amplitude_accel')
        params.amplitude_accel = [0.3, 0.25, 0.2];
    end
    if ~isfield(params, 'amplitude_gyro')
        params.amplitude_gyro = [0.05, 0.04, 0.03];
    end
    
    dt = mean(diff(time));
    fs = 1/dt;
    freq = mean(params.freq_band);
    
    N = length(time);
    accel_vib = zeros(N, 3);
    gyro_vib = zeros(N, 3);
    
    % Generate band-limited random signals
    for axis = 1:3
        % Generate white noise
        noise_accel = randn(N, 1);
        noise_gyro = randn(N, 1);
        
        % Apply band-pass filtering
        if exist('butter', 'file') == 2 && exist('filtfilt', 'file') == 2
            [b, a] = butter(4, params.freq_band / (fs/2), 'bandpass');
            noise_accel = filtfilt(b, a, noise_accel);
            noise_gyro = filtfilt(b, a, noise_gyro);
        end
        
        % Scale to desired amplitude
        noise_accel = noise_accel / std(noise_accel) * params.amplitude_accel(axis);
        noise_gyro = noise_gyro / std(noise_gyro) * params.amplitude_gyro(axis);
        
        accel_vib(:, axis) = noise_accel;
        gyro_vib(:, axis) = noise_gyro;
    end
end

function [accel_vib, gyro_vib, freq] = generate_rotor_vibration(time, params)
%GENERATE_ROTOR_VIBRATION Quadcopter rotor-induced vibrations.
    
    % Set default parameters (typical for quadcopter)
    if ~isfield(params, 'rotor_freq')
        params.rotor_freq = 35; % Hz (rotor RPM / 60)
    end
    if ~isfield(params, 'blade_pass_freq')
        params.blade_pass_freq = params.rotor_freq * 2; % 2-blade rotor
    end
    if ~isfield(params, 'amplitude_accel')
        params.amplitude_accel = [0.8, 0.6, 1.2]; % Stronger in Z
    end
    if ~isfield(params, 'amplitude_gyro')
        params.amplitude_gyro = [0.15, 0.12, 0.08];
    end
    if ~isfield(params, 'modulation')
        params.modulation = 0.2; % Amplitude modulation factor
    end
    
    freq = params.blade_pass_freq;
    omega_rotor = 2 * pi * params.rotor_freq;
    omega_blade = 2 * pi * params.blade_pass_freq;
    
    N = length(time);
    accel_vib = zeros(N, 3);
    gyro_vib = zeros(N, 3);
    
    % Generate rotor vibrations with modulation
    for axis = 1:3
        % Main blade passage frequency
        base_signal = sin(omega_blade * time + 2*pi*rand());
        
        % Add rotor frequency component
        rotor_component = 0.3 * sin(omega_rotor * time + 2*pi*rand());
        
        % Add amplitude modulation
        modulation = 1 + params.modulation * sin(0.1 * omega_rotor * time);
        
        accel_vib(:, axis) = params.amplitude_accel(axis) * ...
                             modulation .* (base_signal + rotor_component);
        gyro_vib(:, axis) = params.amplitude_gyro(axis) * ...
                           modulation .* (0.7 * base_signal + 0.5 * rotor_component);
    end
end