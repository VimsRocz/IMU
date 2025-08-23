%VIBRATION_EXAMPLE_SIMPLE Simple example of vibration detection and rejection.
%   This script shows basic usage of the vibration processing functions:
%   - Generate clean IMU data
%   - Add vibration using vibration_model
%   - Detect vibration using vibration_detection  
%   - Remove vibration using vibration_compensation

clear; clc; close all;

fprintf('Simple Vibration Processing Example\n');
fprintf('===================================\n');

%% 1. Setup parameters
fs = 400;           % Sampling frequency (Hz)
duration = 5;       % Signal duration (seconds)
time = (0:1/fs:duration-1/fs)';
N = length(time);

fprintf('Generated %d samples at %d Hz for %.1f seconds\n', N, fs, duration);

%% 2. Create clean IMU data (stationary IMU)
% Accelerometer: gravity + small noise
accel_clean = zeros(N, 3);
accel_clean(:, 3) = -9.81;  % Gravity in Z-axis
accel_clean = accel_clean + 0.01 * randn(N, 3);  % Add noise

% Gyroscope: Earth rotation + small noise  
gyro_clean = zeros(N, 3);
gyro_clean(:, 3) = 7.29e-5;  % Earth rotation
gyro_clean = gyro_clean + 1e-6 * randn(N, 3);  % Add noise

fprintf('Created clean IMU data (stationary with gravity and Earth rotation)\n');

%% 3. Add vibration to the data
fprintf('\nAdding 50 Hz sinusoidal vibration...\n');

% Configure vibration parameters
vib_params.frequency = 50;                    % Hz
vib_params.amplitude_accel = [0.5, 0.3, 0.2]; % m/s^2
vib_params.amplitude_gyro = [0.1, 0.05, 0.02]; % rad/s

% Generate vibration
vibration = vibration_model(time, 'sinusoidal', vib_params);

% Add vibration to clean data
accel_vibrated = accel_clean + vibration.accel;
gyro_vibrated = gyro_clean + vibration.gyro;

fprintf('Added vibration: %.1f Hz, amplitude [%.1f, %.1f, %.1f] m/s² (accel)\n', ...
        vibration.freq, vib_params.amplitude_accel);

%% 4. Detect vibration
fprintf('\nDetecting vibration...\n');

detection_result = vibration_detection(accel_vibrated, gyro_vibrated, fs, ...
                                     'Method', 'combined', ...
                                     'FreqRange', [20, 100]);

fprintf('Vibration detected: %s\n', char("YES" * detection_result.vibration_detected + "NO" * ~detection_result.vibration_detected));
fprintf('Dominant frequency: %.1f Hz\n', detection_result.dominant_freq);
fprintf('Detection confidence: %.2f\n', detection_result.confidence);
fprintf('Power ratio in vibration band: %.3f\n', detection_result.power_ratio);

%% 5. Remove vibration using different methods
fprintf('\nTesting vibration compensation methods...\n');

methods = {'lowpass', 'notch', 'adaptive', 'spectral'};
compensated_data = struct();

for i = 1:length(methods)
    method = methods{i};
    
    [accel_comp, gyro_comp, comp_info] = vibration_compensation(...
        accel_vibrated, gyro_vibrated, fs, ...
        'Method', method, 'AutoDetect', true);
    
    compensated_data.(method).accel = accel_comp;
    compensated_data.(method).gyro = gyro_comp;
    compensated_data.(method).info = comp_info;
    
    fprintf('  %s method: %.1f%% effective\n', method, comp_info.effectiveness * 100);
end

%% 6. Calculate error metrics
fprintf('\nCalculating error metrics (RMS error vs clean data)...\n');

% RMS error for vibrated data
rms_error_vibrated_accel = sqrt(mean(sum((accel_vibrated - accel_clean).^2, 2)));
rms_error_vibrated_gyro = sqrt(mean(sum((gyro_vibrated - gyro_clean).^2, 2)));

fprintf('Vibrated data RMS error:\n');
fprintf('  Accelerometer: %.4f m/s²\n', rms_error_vibrated_accel);
fprintf('  Gyroscope:     %.6f rad/s\n', rms_error_vibrated_gyro);

fprintf('\nCompensated data RMS error:\n');
for i = 1:length(methods)
    method = methods{i};
    
    rms_error_comp_accel = sqrt(mean(sum((compensated_data.(method).accel - accel_clean).^2, 2)));
    rms_error_comp_gyro = sqrt(mean(sum((compensated_data.(method).gyro - gyro_clean).^2, 2)));
    
    improvement_accel = (1 - rms_error_comp_accel / rms_error_vibrated_accel) * 100;
    improvement_gyro = (1 - rms_error_comp_gyro / rms_error_vibrated_gyro) * 100;
    
    fprintf('  %s method:\n', method);
    fprintf('    Accelerometer: %.4f m/s² (%.1f%% improvement)\n', ...
            rms_error_comp_accel, improvement_accel);
    fprintf('    Gyroscope:     %.6f rad/s (%.1f%% improvement)\n', ...
            rms_error_comp_gyro, improvement_gyro);
end

%% 7. Create visualization plots
fprintf('\nGenerating plots...\n');

% Plot 1: Time domain comparison
figure('Name', 'Time Domain Comparison', 'Position', [100, 100, 1200, 800]);

% Accelerometer data
subplot(2, 1, 1);
plot(time, vecnorm(accel_clean, 2, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Clean'); hold on;
plot(time, vecnorm(accel_vibrated, 2, 2), 'r--', 'LineWidth', 1, 'DisplayName', 'Vibrated');
plot(time, vecnorm(compensated_data.adaptive.accel, 2, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Compensated (Adaptive)');
xlabel('Time (s)'); ylabel('|Acceleration| (m/s²)');
title('Accelerometer Data - Before and After Vibration Compensation');
legend('Location', 'best'); grid on;

% Gyroscope data  
subplot(2, 1, 2);
plot(time, vecnorm(gyro_clean, 2, 2), 'g-', 'LineWidth', 2, 'DisplayName', 'Clean'); hold on;
plot(time, vecnorm(gyro_vibrated, 2, 2), 'r--', 'LineWidth', 1, 'DisplayName', 'Vibrated');
plot(time, vecnorm(compensated_data.adaptive.gyro, 2, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Compensated (Adaptive)');
xlabel('Time (s)'); ylabel('|Angular Rate| (rad/s)');
title('Gyroscope Data - Before and After Vibration Compensation');
legend('Location', 'best'); grid on;

% Plot 2: Frequency domain analysis
figure('Name', 'Frequency Domain Analysis', 'Position', [150, 150, 1200, 600]);

% Compute frequency spectra
accel_magnitude_clean = vecnorm(accel_clean, 2, 2);
accel_magnitude_vibrated = vecnorm(accel_vibrated, 2, 2);
accel_magnitude_compensated = vecnorm(compensated_data.adaptive.accel, 2, 2);

[psd_clean, f] = compute_power_spectrum(accel_magnitude_clean, fs);
[psd_vibrated, ~] = compute_power_spectrum(accel_magnitude_vibrated, fs);
[psd_compensated, ~] = compute_power_spectrum(accel_magnitude_compensated, fs);

% Plot power spectral densities
semilogy(f, psd_clean, 'g-', 'LineWidth', 2, 'DisplayName', 'Clean'); hold on;
semilogy(f, psd_vibrated, 'r--', 'LineWidth', 1, 'DisplayName', 'Vibrated');
semilogy(f, psd_compensated, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Compensated (Adaptive)');

% Highlight vibration frequency
line([50, 50], ylim, 'Color', 'm', 'LineStyle', ':', 'LineWidth', 2, 'DisplayName', '50 Hz Vibration');

xlabel('Frequency (Hz)'); ylabel('PSD (m²/s⁴/Hz)');
title('Power Spectral Density - Accelerometer Magnitude');
legend('Location', 'best'); grid on;
xlim([0, 100]); % Focus on relevant frequency range

% Plot 3: Detection results
figure('Name', 'Vibration Detection', 'Position', [200, 200, 1000, 400]);

yyaxis left;
plot(time, vecnorm(accel_vibrated, 2, 2), 'b-', 'LineWidth', 1);
ylabel('|Acceleration| (m/s²)'); 

yyaxis right;
plot(time, double(detection_result.vibration_flag), 'r-', 'LineWidth', 3);
ylabel('Vibration Detection Flag');
ylim([-0.1, 1.1]);

xlabel('Time (s)');
title(sprintf('Vibration Detection Result (Confidence: %.2f)', detection_result.confidence));
grid on;

fprintf('Plots generated successfully!\n');

%% 8. Summary
fprintf('\n=== SUMMARY ===\n');
fprintf('Vibration successfully detected and compensated:\n');
fprintf('- Original vibration: %.1f Hz, RMS error %.4f m/s² (accel)\n', ...
        vibration.freq, rms_error_vibrated_accel);
fprintf('- Best compensation method: %s\n', get_best_method(compensated_data, accel_clean, gyro_clean));
fprintf('- Detection confidence: %.2f\n', detection_result.confidence);
fprintf('Example completed successfully!\n');

%% Helper functions

function [psd, f] = compute_power_spectrum(signal, fs)
%COMPUTE_POWER_SPECTRUM Compute single-sided power spectral density
    N = length(signal);
    Y = fft(signal);
    P = abs(Y).^2 / (N * fs);
    psd = P(1:floor(N/2)+1);
    psd(2:end-1) = 2 * psd(2:end-1);  % Single-sided spectrum
    f = (0:floor(N/2)) * fs / N;
    f = f';
    psd = psd';
end

function best_method = get_best_method(compensated_data, accel_clean, gyro_clean)
%GET_BEST_METHOD Find the compensation method with lowest RMS error
    methods = fieldnames(compensated_data);
    min_error = inf;
    best_method = '';
    
    for i = 1:length(methods)
        method = methods{i};
        accel_comp = compensated_data.(method).accel;
        gyro_comp = compensated_data.(method).gyro;
        
        rms_error_accel = sqrt(mean(sum((accel_comp - accel_clean).^2, 2)));
        rms_error_gyro = sqrt(mean(sum((gyro_comp - gyro_clean).^2, 2)));
        total_error = rms_error_accel + rms_error_gyro;
        
        if total_error < min_error
            min_error = total_error;
            best_method = method;
        end
    end
end