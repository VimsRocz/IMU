%VIBRATION_DEMO Comprehensive demo of vibration detection and rejection from IMU data.
%   This script demonstrates the complete vibration processing pipeline:
%   1. Load or simulate clean IMU data
%   2. Add realistic vibration signals using vibration_model
%   3. Detect vibration using vibration_detection
%   4. Remove vibration using vibration_compensation
%   5. Analyze and visualize results
%
%   The demo covers various vibration types and compensation methods,
%   showing the effectiveness of each approach.

clear; clc; close all;

fprintf('=== Vibration Detection and Rejection Demo ===\n');

%% Configuration
config = struct();
config.fs = 400;                    % Sampling frequency (Hz)
config.duration = 10;               % Signal duration (seconds)
config.use_real_data = false;       % Set to true to load real IMU data
config.save_plots = true;           % Save plots to files
config.show_plots = true;           % Display plots

%% 1. Generate or Load IMU Data
fprintf('\n1. Preparing IMU data...\n');

if config.use_real_data
    % Load real IMU data (modify path as needed)
    try
        imu_data = load_real_imu_data('DATA/IMU/IMU_X001.dat');
        time = imu_data.time(1:min(config.fs * config.duration, end));
        accel_clean = imu_data.accel(1:length(time), :);
        gyro_clean = imu_data.gyro(1:length(time), :);
        fprintf('   Loaded real IMU data: %d samples\n', length(time));
    catch
        fprintf('   Could not load real data, using simulated data instead\n');
        config.use_real_data = false;
    end
end

if ~config.use_real_data
    % Generate synthetic clean IMU data
    time = (0:1/config.fs:config.duration-1/config.fs)';
    N = length(time);
    
    % Simulate clean IMU measurements (stationary with some motion)
    accel_clean = simulate_clean_accel(time);
    gyro_clean = simulate_clean_gyro(time);
    fprintf('   Generated synthetic clean IMU data: %d samples\n', N);
end

%% 2. Add Various Types of Vibration
fprintf('\n2. Adding vibration to clean data...\n');

vibration_types = {'sinusoidal', 'motor', 'rotor', 'random'};
vibrated_data = struct();

for i = 1:length(vibration_types)
    vib_type = vibration_types{i};
    fprintf('   Adding %s vibration...\n', vib_type);
    
    % Configure vibration parameters
    vib_params = get_vibration_params(vib_type);
    
    % Generate vibration
    vibration = vibration_model(time, vib_type, vib_params);
    
    % Add to clean data
    vibrated_data.(vib_type).accel = accel_clean + vibration.accel;
    vibrated_data.(vib_type).gyro = gyro_clean + vibration.gyro;
    vibrated_data.(vib_type).vibration = vibration;
end

%% 3. Demonstrate Vibration Detection
fprintf('\n3. Testing vibration detection methods...\n');

detection_methods = {'frequency', 'variance', 'energy', 'combined'};
detection_results = struct();

for i = 1:length(vibration_types)
    vib_type = vibration_types{i};
    detection_results.(vib_type) = struct();
    
    for j = 1:length(detection_methods)
        method = detection_methods{j};
        
        result = vibration_detection(vibrated_data.(vib_type).accel, ...
                                   vibrated_data.(vib_type).gyro, ...
                                   config.fs, 'Method', method);
        
        detection_results.(vib_type).(method) = result;
        
        fprintf('   %s vibration, %s method: %s (confidence: %.2f)\n', ...
                vib_type, method, ...
                char("Detected" * result.vibration_detected + "Not detected" * ~result.vibration_detected), ...
                result.confidence);
    end
end

%% 4. Demonstrate Vibration Compensation
fprintf('\n4. Testing vibration compensation methods...\n');

compensation_methods = {'lowpass', 'notch', 'adaptive', 'spectral'};
compensation_results = struct();

for i = 1:length(vibration_types)
    vib_type = vibration_types{i};
    compensation_results.(vib_type) = struct();
    
    fprintf('   Processing %s vibration:\n', vib_type);
    
    for j = 1:length(compensation_methods)
        method = compensation_methods{j};
        
        [accel_comp, gyro_comp, comp_info] = vibration_compensation(...
            vibrated_data.(vib_type).accel, ...
            vibrated_data.(vib_type).gyro, ...
            config.fs, 'Method', method);
        
        compensation_results.(vib_type).(method) = struct();
        compensation_results.(vib_type).(method).accel = accel_comp;
        compensation_results.(vib_type).(method).gyro = gyro_comp;
        compensation_results.(vib_type).(method).info = comp_info;
        
        fprintf('     %s method: effectiveness = %.1f%%\n', ...
                method, comp_info.effectiveness * 100);
    end
end

%% 5. Performance Analysis
fprintf('\n5. Analyzing compensation performance...\n');

performance_analysis(accel_clean, gyro_clean, vibrated_data, ...
                    compensation_results, config.fs);

%% 6. Generate Visualization Plots
fprintf('\n6. Generating visualization plots...\n');

if config.show_plots
    % Create comprehensive plots
    create_vibration_plots(time, accel_clean, gyro_clean, vibrated_data, ...
                          detection_results, compensation_results, config);
end

fprintf('\n=== Demo completed successfully! ===\n');

%% Helper Functions

function accel = simulate_clean_accel(time)
%SIMULATE_CLEAN_ACCEL Generate realistic clean accelerometer data.
    
    % Gravity + small movements
    gravity = [0; 0; -9.81];
    
    % Small sinusoidal movements (simulating gentle motion)
    motion_freq = [0.1, 0.15, 0.05]; % Hz
    motion_amp = [0.05, 0.03, 0.02];  % m/s^2
    
    accel = repmat(gravity', length(time), 1);
    
    for axis = 1:3
        accel(:, axis) = accel(:, axis) + ...
            motion_amp(axis) * sin(2*pi*motion_freq(axis)*time) + ...
            0.01 * randn(length(time), 1); % Small noise
    end
end

function gyro = simulate_clean_gyro(time)
%SIMULATE_CLEAN_GYRO Generate realistic clean gyroscope data.
    
    % Earth rotation (approximately 7.29e-5 rad/s)
    earth_rate = 7.29e-5;
    
    % Small angular movements
    motion_freq = [0.08, 0.12, 0.03]; % Hz
    motion_amp = [0.002, 0.001, 0.0015]; % rad/s
    
    gyro = zeros(length(time), 3);
    
    % Add Earth rotation (simplified, assumes certain orientation)
    gyro(:, 3) = earth_rate;
    
    for axis = 1:3
        gyro(:, axis) = gyro(:, axis) + ...
            motion_amp(axis) * sin(2*pi*motion_freq(axis)*time) + ...
            1e-5 * randn(length(time), 1); % Small noise
    end
end

function params = get_vibration_params(vib_type)
%GET_VIBRATION_PARAMS Get parameters for different vibration types.
    
    switch lower(vib_type)
        case 'sinusoidal'
            params.frequency = 50;
            params.amplitude_accel = [0.5, 0.4, 0.3];
            params.amplitude_gyro = [0.08, 0.06, 0.04];
            
        case 'motor'
            params.base_freq = 25;
            params.harmonics = [1, 2, 3, 4];
            params.harmonic_weights = [1.0, 0.6, 0.3, 0.15];
            params.amplitude_accel = [0.8, 0.7, 0.6];
            params.amplitude_gyro = [0.15, 0.12, 0.1];
            
        case 'rotor'
            params.rotor_freq = 35;
            params.blade_pass_freq = 70;
            params.amplitude_accel = [0.6, 0.5, 1.0];
            params.amplitude_gyro = [0.1, 0.08, 0.06];
            params.modulation = 0.3;
            
        case 'random'
            params.freq_band = [20, 120];
            params.amplitude_accel = [0.4, 0.35, 0.3];
            params.amplitude_gyro = [0.06, 0.05, 0.04];
    end
end

function imu_data = load_real_imu_data(filename)
%LOAD_REAL_IMU_DATA Load real IMU data from file.
    
    % This is a placeholder - adapt to your specific data format
    data = readmatrix(filename);
    
    % Assuming format: time, gyro(3), accel(3), ...
    imu_data.time = data(:, 1);
    imu_data.gyro = data(:, 2:4);
    imu_data.accel = data(:, 5:7);
end

function performance_analysis(accel_clean, gyro_clean, vibrated_data, compensation_results, fs)
%PERFORMANCE_ANALYSIS Analyze compensation performance.
    
    fprintf('   Performance Summary:\n');
    fprintf('   %-12s %-10s %-10s %-10s %-10s\n', 'Vibration', 'LowPass', 'Notch', 'Adaptive', 'Spectral');
    fprintf('   %s\n', repmat('-', 1, 60));
    
    vibration_types = fieldnames(vibrated_data);
    compensation_methods = {'lowpass', 'notch', 'adaptive', 'spectral'};
    
    for i = 1:length(vibration_types)
        vib_type = vibration_types{i};
        fprintf('   %-12s ', vib_type);
        
        for j = 1:length(compensation_methods)
            method = compensation_methods{j};
            effectiveness = compensation_results.(vib_type).(method).info.effectiveness;
            fprintf('%-10.1f%% ', effectiveness * 100);
        end
        fprintf('\n');
    end
end

function create_vibration_plots(time, accel_clean, gyro_clean, vibrated_data, ...
                               detection_results, compensation_results, config)
%CREATE_VIBRATION_PLOTS Create comprehensive visualization plots.
    
    % Plot 1: Original vs Vibrated Signals
    figure('Name', 'Clean vs Vibrated IMU Data', 'Position', [100, 100, 1200, 800]);
    
    vibration_types = fieldnames(vibrated_data);
    
    for i = 1:length(vibration_types)
        vib_type = vibration_types{i};
        
        % Accelerometer comparison
        subplot(2, length(vibration_types), i);
        plot(time, vecnorm(accel_clean, 2, 2), 'b-', 'LineWidth', 1.5); hold on;
        plot(time, vecnorm(vibrated_data.(vib_type).accel, 2, 2), 'r--', 'LineWidth', 1);
        xlabel('Time (s)'); ylabel('|Acceleration| (m/s²)');
        title(sprintf('%s - Accelerometer', upper(vib_type)));
        legend('Clean', 'Vibrated', 'Location', 'best');
        grid on;
        
        % Gyroscope comparison
        subplot(2, length(vibration_types), i + length(vibration_types));
        plot(time, vecnorm(gyro_clean, 2, 2), 'b-', 'LineWidth', 1.5); hold on;
        plot(time, vecnorm(vibrated_data.(vib_type).gyro, 2, 2), 'r--', 'LineWidth', 1);
        xlabel('Time (s)'); ylabel('|Angular Rate| (rad/s)');
        title(sprintf('%s - Gyroscope', upper(vib_type)));
        legend('Clean', 'Vibrated', 'Location', 'best');
        grid on;
    end
    
    if config.save_plots
        saveas(gcf, 'vibration_demo_clean_vs_vibrated.png');
    end
    
    % Plot 2: Detection Results
    figure('Name', 'Vibration Detection Results', 'Position', [150, 150, 1200, 600]);
    
    for i = 1:length(vibration_types)
        vib_type = vibration_types{i};
        subplot(2, 2, i);
        
        % Show combined detection result
        result = detection_results.(vib_type).combined;
        
        % Plot signal magnitude and detection flag
        yyaxis left;
        signal_mag = vecnorm(vibrated_data.(vib_type).accel, 2, 2);
        plot(time, signal_mag, 'b-'); 
        ylabel('|Acceleration| (m/s²)');
        
        yyaxis right;
        plot(time, double(result.vibration_flag), 'r-', 'LineWidth', 2);
        ylabel('Detection Flag');
        ylim([-0.1, 1.1]);
        
        xlabel('Time (s)');
        title(sprintf('%s Vibration Detection\n(Confidence: %.2f)', upper(vib_type), result.confidence));
        grid on;
    end
    
    if config.save_plots
        saveas(gcf, 'vibration_demo_detection_results.png');
    end
    
    % Plot 3: Compensation Effectiveness
    figure('Name', 'Vibration Compensation Effectiveness', 'Position', [200, 200, 1200, 800]);
    
    compensation_methods = {'lowpass', 'notch', 'adaptive', 'spectral'};
    
    for i = 1:length(vibration_types)
        vib_type = vibration_types{i};
        
        subplot(2, 2, i);
        
        % Show original, vibrated, and compensated signals
        plot(time, vecnorm(accel_clean, 2, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clean'); hold on;
        plot(time, vecnorm(vibrated_data.(vib_type).accel, 2, 2), 'r:', 'LineWidth', 1, 'DisplayName', 'Vibrated');
        
        colors = {'b', 'm', 'c', 'k'};
        for j = 1:length(compensation_methods)
            method = compensation_methods{j};
            accel_comp = compensation_results.(vib_type).(method).accel;
            effectiveness = compensation_results.(vib_type).(method).info.effectiveness;
            
            plot(time, vecnorm(accel_comp, 2, 2), colors{j}, 'LineWidth', 1, ...
                 'DisplayName', sprintf('%s (%.0f%%)', method, effectiveness*100));
        end
        
        xlabel('Time (s)'); ylabel('|Acceleration| (m/s²)');
        title(sprintf('%s Vibration Compensation', upper(vib_type)));
        legend('Location', 'best');
        grid on;
    end
    
    if config.save_plots
        saveas(gcf, 'vibration_demo_compensation_results.png');
    end
    
    % Plot 4: Frequency Domain Analysis
    create_frequency_analysis_plot(time, accel_clean, vibrated_data, compensation_results, config);
end

function create_frequency_analysis_plot(time, accel_clean, vibrated_data, compensation_results, config)
%CREATE_FREQUENCY_ANALYSIS_PLOT Create frequency domain analysis plots.
    
    figure('Name', 'Frequency Domain Analysis', 'Position', [250, 250, 1200, 800]);
    
    fs = config.fs;
    vibration_types = fieldnames(vibrated_data);
    
    for i = 1:length(vibration_types)
        vib_type = vibration_types{i};
        
        subplot(2, 2, i);
        
        % Compute power spectral densities
        accel_mag_clean = vecnorm(accel_clean, 2, 2);
        accel_mag_vibrated = vecnorm(vibrated_data.(vib_type).accel, 2, 2);
        accel_mag_compensated = vecnorm(compensation_results.(vib_type).adaptive.accel, 2, 2);
        
        [psd_clean, f] = compute_psd_simple(accel_mag_clean, fs);
        [psd_vibrated, ~] = compute_psd_simple(accel_mag_vibrated, fs);
        [psd_compensated, ~] = compute_psd_simple(accel_mag_compensated, fs);
        
        % Plot PSDs
        semilogy(f, psd_clean, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Clean'); hold on;
        semilogy(f, psd_vibrated, 'r--', 'LineWidth', 1, 'DisplayName', 'Vibrated');
        semilogy(f, psd_compensated, 'b-', 'LineWidth', 1, 'DisplayName', 'Compensated');
        
        xlabel('Frequency (Hz)'); ylabel('PSD (m²/s⁴/Hz)');
        title(sprintf('%s - Frequency Analysis', upper(vib_type)));
        legend('Location', 'best');
        grid on;
        xlim([0, min(100, fs/2)]);
    end
    
    if config.save_plots
        saveas(gcf, 'vibration_demo_frequency_analysis.png');
    end
end

function [psd, f] = compute_psd_simple(signal, fs)
%COMPUTE_PSD_SIMPLE Simple PSD computation.
    
    N = length(signal);
    Y = fft(signal);
    P = abs(Y).^2 / (N * fs);
    psd = P(1:floor(N/2)+1);
    psd(2:end-1) = 2 * psd(2:end-1);  % Single-sided
    f = (0:floor(N/2)) * fs / N;
end