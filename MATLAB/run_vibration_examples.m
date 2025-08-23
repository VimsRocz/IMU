function run_vibration_examples()
%RUN_VIBRATION_EXAMPLES Demonstrate IMU vibration simulation and compensation
%   This function runs examples showing how to use the MATLAB vibration
%   simulation and compensation functions for IMU data processing.
%
%   The examples demonstrate:
%   1. Basic vibration simulation with different vibration types
%   2. Vibration detection using statistical and frequency methods
%   3. Vibration compensation using various filtering techniques
%   4. Integration with existing IMU processing pipeline
%
%   Examples generate plots saved to the current directory.

fprintf('IMU Vibration Simulation and Compensation Examples (MATLAB)\n');
fprintf('==========================================================\n\n');

try
    % Run examples
    example_1_basic_vibration_simulation();
    example_2_vibration_detection();
    example_3_vibration_compensation(); 
    example_4_integrated_analysis();
    
    fprintf('All examples completed successfully!\n');
    fprintf('Generated plots saved in current directory.\n');
    
catch ME
    fprintf('Error running examples: %s\n', ME.message);
    rethrow(ME);
end

end

function example_1_basic_vibration_simulation()
%EXAMPLE_1_BASIC_VIBRATION_SIMULATION Demonstrate different vibration types

fprintf('=== Example 1: Basic Vibration Simulation ===\n');

% Simulation parameters
fs = 400;      % Sampling frequency (Hz)
duration = 5;  % Duration (seconds)

% Generate different types of vibration
vibration_types = {
    'sinusoidal', struct('Frequency', 50, 'AmplitudeAcc', 1.0, 'AmplitudeGyro', 0.1);
    'random',     struct('CutoffFreq', 80, 'AmplitudeAcc', 0.8, 'AmplitudeGyro', 0.08);
    'motor',      struct('Frequency', 60, 'AmplitudeAcc', 2.0, 'AmplitudeGyro', 0.2);
    'rotor',      struct('Frequency', 120, 'NumBlades', 4, 'AmplitudeAcc', 2.5, 'AmplitudeGyro', 0.25)
};

figure('Name', 'Vibration Types Comparison', 'Position', [100, 100, 1200, 800]);

for i = 1:length(vibration_types)
    vib_type = vibration_types{i, 1};
    params = vibration_types{i, 2};
    
    fprintf('Generating %s vibration...\n', vib_type);
    
    % Generate vibration signals
    if strcmp(vib_type, 'sinusoidal')
        vib = generate_vibration_model(duration, fs, 'Type', vib_type, ...
                                      'Frequency', params.Frequency, ...
                                      'AmplitudeAcc', params.AmplitudeAcc, ...
                                      'AmplitudeGyro', params.AmplitudeGyro);
    elseif strcmp(vib_type, 'random')
        vib = generate_vibration_model(duration, fs, 'Type', vib_type, ...
                                      'CutoffFreq', params.CutoffFreq, ...
                                      'AmplitudeAcc', params.AmplitudeAcc, ...
                                      'AmplitudeGyro', params.AmplitudeGyro);
    elseif strcmp(vib_type, 'motor')
        vib = generate_vibration_model(duration, fs, 'Type', vib_type, ...
                                      'Frequency', params.Frequency, ...
                                      'AmplitudeAcc', params.AmplitudeAcc, ...
                                      'AmplitudeGyro', params.AmplitudeGyro);
    elseif strcmp(vib_type, 'rotor')
        vib = generate_vibration_model(duration, fs, 'Type', vib_type, ...
                                      'Frequency', params.Frequency, ...
                                      'NumBlades', params.NumBlades, ...
                                      'AmplitudeAcc', params.AmplitudeAcc, ...
                                      'AmplitudeGyro', params.AmplitudeGyro);
    end
    
    % Plot accelerometer vibration
    subplot(2, 2, i);
    plot(vib.time, vib.accel_vib(:, 1), 'r-', 'LineWidth', 0.8); hold on;
    plot(vib.time, vib.accel_vib(:, 2), 'g-', 'LineWidth', 0.8);
    plot(vib.time, vib.accel_vib(:, 3), 'b-', 'LineWidth', 0.8);
    title([upper(vib_type(1)), vib_type(2:end), ' Vibration - Accelerometer']);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    legend('X-axis', 'Y-axis', 'Z-axis', 'Location', 'best');
    grid on;
    
    % Print statistics
    rms_acc = sqrt(mean(vib.accel_vib.^2, 1));
    rms_gyro = sqrt(mean(vib.gyro_vib.^2, 1));
    fprintf('  RMS Accelerometer: [%.3f, %.3f, %.3f] m/s²\n', rms_acc);
    fprintf('  RMS Gyroscope: [%.3f, %.3f, %.3f] rad/s\n', rms_gyro);
end

% Save plot
saveas(gcf, 'vibration_types_comparison_matlab.png');
fprintf('Saved: vibration_types_comparison_matlab.png\n\n');

end

function example_2_vibration_detection()
%EXAMPLE_2_VIBRATION_DETECTION Demonstrate vibration detection methods

fprintf('=== Example 2: Vibration Detection ===\n');

% Create test data with and without vibration
fs = 400;
duration = 10;
t = (0:1/fs:(duration-1/fs))';

% Clean IMU signal (low frequency motion)
clean_freq = 0.3;
clean_accel = [0.5 * sin(2 * pi * clean_freq * t), ...
               0.3 * cos(2 * pi * clean_freq * t), ...
               9.81 + 0.2 * sin(2 * pi * clean_freq * t)];

% Add vibration to second half
n_half = floor(length(t) / 2);
vibration = generate_vibration_model(duration/2, fs, 'Type', 'motor', ...
                                    'Frequency', 75, 'AmplitudeAcc', 1.5);

% Combine clean and vibrated data
test_accel = clean_accel;
test_accel(n_half+1:end, :) = test_accel(n_half+1:end, :) + vibration.accel_vib;

% Test different detection methods
detection_methods = {'statistical', 'frequency'};

figure('Name', 'Vibration Detection Comparison', 'Position', [150, 150, 1200, 800]);

% Plot original signal
subplot(2, 2, 1);
plot(t, test_accel(:, 1), 'r-', 'LineWidth', 0.8); hold on;
plot(t, test_accel(:, 2), 'g-', 'LineWidth', 0.8);
plot(t, test_accel(:, 3), 'b-', 'LineWidth', 0.8);
xline(duration/2, 'k--', 'Alpha', 0.7, 'LineWidth', 1.5, 'DisplayName', 'Vibration Start');
title('Test Signal (Clean + Vibration)');
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
legend('X', 'Y', 'Z', 'Vibration Start', 'Location', 'best');
grid on;

% Test each detection method
for i = 1:length(detection_methods)
    method = detection_methods{i};
    fprintf('Testing %s detection method...\n', method);
    
    if strcmp(method, 'statistical')
        result = detect_vibration(test_accel, fs, 'Method', method);
    else  % frequency
        result = detect_vibration(test_accel, fs, 'Method', method, ...
                                'FreqRange', [50, 150]);
    end
    
    % Plot detection results
    subplot(2, 2, i + 1);
    detection_signal = double(result.vibration_detected);
    plot(t, detection_signal, 'r-', 'LineWidth', 2); hold on;
    xline(duration/2, 'k--', 'Alpha', 0.7, 'LineWidth', 1.5);
    title([upper(method(1)), method(2:end), ' Detection']);
    xlabel('Time (s)');
    ylabel('Detection Flag');
    ylim([-0.1, 1.1]);
    legend('Vibration Detected', 'True Start', 'Location', 'best');
    grid on;
    
    % Calculate detection statistics  
    true_positive = sum(detection_signal(n_half+1:end));
    false_positive = sum(detection_signal(1:n_half));
    sensitivity = true_positive / (length(t) - n_half) * 100;
    specificity = (n_half - false_positive) / n_half * 100;
    
    fprintf('  %s Detection - Sensitivity: %.1f%%, Specificity: %.1f%%\n', ...
            method, sensitivity, specificity);
    fprintf('  Vibration ratio: %.1f%%, Summary: %s\n', ...
            result.summary.vibration_ratio * 100, ...
            jsonencode(result.summary));
end

% Add intensity plot
subplot(2, 2, 4);
result_stat = detect_vibration(test_accel, fs, 'Method', 'statistical');
plot(t, result_stat.vibration_intensity, 'b-', 'LineWidth', 1.5);
xline(duration/2, 'k--', 'Alpha', 0.7, 'LineWidth', 1.5);
title('Vibration Intensity (Statistical)');
xlabel('Time (s)');
ylabel('Intensity Ratio');
legend('Intensity', 'Vibration Start', 'Location', 'best');
grid on;

% Save plot
saveas(gcf, 'vibration_detection_comparison_matlab.png');
fprintf('Saved: vibration_detection_comparison_matlab.png\n\n');

end

function example_3_vibration_compensation()
%EXAMPLE_3_VIBRATION_COMPENSATION Demonstrate vibration compensation methods

fprintf('=== Example 3: Vibration Compensation ===\n');

% Generate vibrated IMU data
fs = 400;
duration = 8;
t = (0:1/fs:(duration-1/fs))';

% Clean trajectory motion
clean_accel = [0.8 * sin(2 * pi * 0.2 * t), ...
               0.6 * cos(2 * pi * 0.3 * t), ...
               9.81 + 0.4 * sin(2 * pi * 0.1 * t)];

% Add motor vibration
vibration = generate_vibration_model(duration, fs, 'Type', 'motor', ...
                                    'Frequency', 85, 'AmplitudeAcc', 2.0);

vibrated_accel = clean_accel + vibration.accel_vib;

% Test compensation methods
compensation_methods = {
    'butterworth', struct('CutoffFreq', 15);
    'notch',       struct('NotchFreq', 85, 'QualityFactor', 20);
    'kalman',      struct('ProcessNoise', 1e-4, 'MeasNoise', 1e-2)
};

figure('Name', 'Vibration Compensation Comparison', 'Position', [200, 200, 1200, 800]);

% Plot original signals
subplot(2, 2, 1);
plot(t, clean_accel(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', 'Clean Signal'); hold on;
plot(t, vibrated_accel(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', 'With Vibration', ...
     'Color', [1, 0, 0, 0.7]);
title('Z-axis Accelerometer: Original vs Vibrated');
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
legend('Location', 'best');
grid on;

% Test each compensation method
for i = 1:length(compensation_methods)
    method = compensation_methods{i, 1};
    params = compensation_methods{i, 2};
    
    fprintf('Testing %s compensation...\n', method);
    
    if strcmp(method, 'butterworth')
        result = compensate_vibration(vibrated_accel, fs, 'Method', method, ...
                                    'CutoffFreq', params.CutoffFreq);
    elseif strcmp(method, 'notch')
        result = compensate_vibration(vibrated_accel, fs, 'Method', method, ...
                                    'NotchFreq', params.NotchFreq, ...
                                    'QualityFactor', params.QualityFactor);
    elseif strcmp(method, 'kalman')
        result = compensate_vibration(vibrated_accel, fs, 'Method', method, ...
                                    'ProcessNoise', params.ProcessNoise, ...
                                    'MeasNoise', params.MeasNoise);
    end
    
    compensated_accel = result.filtered_data;
    
    % Plot compensation results (Z-axis)
    subplot(2, 2, i + 1);
    plot(t, clean_accel(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', 'True Signal'); hold on;
    plot(t, compensated_accel(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Compensated');
    title([upper(method(1)), method(2:end), ' Compensation']);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    legend('Location', 'best');
    grid on;
    
    % Calculate compensation metrics
    error_original = sqrt(mean((vibrated_accel(:, 3) - clean_accel(:, 3)).^2));
    error_compensated = sqrt(mean((compensated_accel(:, 3) - clean_accel(:, 3)).^2));
    improvement = (error_original - error_compensated) / error_original * 100;
    
    fprintf('  %s - RMSE reduction: %.1f%%\n', method, improvement);
end

% Save plot
saveas(gcf, 'vibration_compensation_comparison_matlab.png');
fprintf('Saved: vibration_compensation_comparison_matlab.png\n\n');

end

function example_4_integrated_analysis()
%EXAMPLE_4_INTEGRATED_ANALYSIS Complete vibration analysis workflow

fprintf('=== Example 4: Integrated Vibration Analysis ===\n');

% Generate realistic IMU data with vibration
fs = 400;
duration = 12;

% Simulate a quadrotor flight with vibration
fprintf('Generating quadrotor flight simulation with vibration...\n');

% Clean trajectory (simplified)
t = (0:1/fs:(duration-1/fs))';
trajectory_freq = 0.1;
clean_accel = [1.5 * sin(2 * pi * trajectory_freq * t), ...
               1.0 * cos(2 * pi * trajectory_freq * t), ...
               9.81 + 2.0 * sin(2 * pi * trajectory_freq * t)];

clean_gyro = [0.1 * cos(2 * pi * trajectory_freq * t), ...
              0.1 * sin(2 * pi * trajectory_freq * t), ...
              0.05 * sin(4 * pi * trajectory_freq * t)];

% Add rotor vibration
rotor_vibration = generate_vibration_model(duration, fs, 'Type', 'rotor', ...
                                          'Frequency', 110, 'NumBlades', 4, ...
                                          'AmplitudeAcc', 2.5, 'AmplitudeGyro', 0.25);

% Create vibrated IMU data
vibrated_accel = clean_accel + rotor_vibration.accel_vib;
vibrated_gyro = clean_gyro + rotor_vibration.gyro_vib;

% Perform vibration detection
fprintf('Performing vibration detection...\n');
accel_detection = detect_vibration(vibrated_accel, fs, 'Method', 'frequency', ...
                                  'FreqRange', [80, 200]);
gyro_detection = detect_vibration(vibrated_gyro, fs, 'Method', 'frequency', ...
                                 'FreqRange', [80, 200]);

% Perform vibration compensation
fprintf('Applying vibration compensation...\n');
accel_compensated = compensate_vibration(vibrated_accel, fs, 'Method', 'butterworth', ...
                                        'CutoffFreq', 25);
gyro_compensated = compensate_vibration(vibrated_gyro, fs, 'Method', 'butterworth', ...
                                       'CutoffFreq', 25);

% Generate comprehensive analysis plot
figure('Name', 'Integrated Vibration Analysis', 'Position', [250, 250, 1400, 1000]);

% Original vs compensated accelerometer
subplot(2, 3, 1);
plot(t, vibrated_accel(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', 'Original'); hold on;
plot(t, accel_compensated.filtered_data(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Compensated');
title('Accelerometer Z-axis: Before/After');
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
legend('Location', 'best');
grid on;

% Original vs compensated gyroscope  
subplot(2, 3, 2);
plot(t, vibrated_gyro(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', 'Original'); hold on;
plot(t, gyro_compensated.filtered_data(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Compensated');
title('Gyroscope Z-axis: Before/After');
xlabel('Time (s)');
ylabel('Angular Rate (rad/s)');
legend('Location', 'best');
grid on;

% Vibration detection results
subplot(2, 3, 3);
accel_detect_signal = double(accel_detection.vibration_detected);
gyro_detect_signal = double(gyro_detection.vibration_detected);
plot(t, accel_detect_signal, 'r-', 'LineWidth', 2, 'DisplayName', 'Accelerometer'); hold on;
plot(t, gyro_detect_signal + 0.1, 'b-', 'LineWidth', 2, 'DisplayName', 'Gyroscope');
title('Vibration Detection Results');
xlabel('Time (s)');
ylabel('Detection Flag');
ylim([-0.1, 1.2]);
legend('Location', 'best');
grid on;

% Power spectral density comparison (accelerometer)
subplot(2, 3, 4);
[psd_orig, f_orig] = pwelch(vibrated_accel(:, 3), [], [], [], fs);
[psd_comp, f_comp] = pwelch(accel_compensated.filtered_data(:, 3), [], [], [], fs);
semilogy(f_orig, psd_orig, 'r-', 'DisplayName', 'Original'); hold on;
semilogy(f_comp, psd_comp, 'b-', 'DisplayName', 'Compensated');
title('Power Spectral Density (Accel Z)');
xlabel('Frequency (Hz)');
ylabel('PSD (m²/s⁴/Hz)');
xlim([0, 200]);
legend('Location', 'best');
grid on;

% Vibration intensity over time
subplot(2, 3, 5);
plot(t, accel_detection.vibration_intensity, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Accel Intensity'); hold on;
plot(t, gyro_detection.vibration_intensity / max(gyro_detection.vibration_intensity) * ...
     max(accel_detection.vibration_intensity), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Gyro Intensity (scaled)');
title('Vibration Intensity Over Time');
xlabel('Time (s)');
ylabel('Intensity');
legend('Location', 'best');
grid on;

% Error reduction analysis
subplot(2, 3, 6);
accel_error_orig = sqrt(mean((vibrated_accel - clean_accel).^2, 1));
accel_error_comp = sqrt(mean((accel_compensated.filtered_data - clean_accel).^2, 1));
gyro_error_orig = sqrt(mean((vibrated_gyro - clean_gyro).^2, 1));
gyro_error_comp = sqrt(mean((gyro_compensated.filtered_data - clean_gyro).^2, 1));

improvement_accel = (accel_error_orig - accel_error_comp) ./ accel_error_orig * 100;
improvement_gyro = (gyro_error_orig - gyro_error_comp) ./ gyro_error_orig * 100;

bar_data = [improvement_accel; improvement_gyro];
bar(bar_data);
title('RMSE Improvement by Axis');
xlabel('Axis');
ylabel('Improvement (%)');
legend('X', 'Y', 'Z', 'Location', 'best');
set(gca, 'XTickLabel', {'Accelerometer', 'Gyroscope'});
grid on;

% Save plot
saveas(gcf, 'integrated_vibration_analysis_matlab.png');
fprintf('Saved: integrated_vibration_analysis_matlab.png\n');

% Print analysis summary
fprintf('\n=== Analysis Summary ===\n');
fprintf('Accelerometer Analysis:\n');
fprintf('  Vibration ratio: %.1f%%\n', accel_detection.summary.vibration_ratio * 100);
fprintf('  Dominant frequency: %.1f Hz\n', accel_detection.summary.dominant_frequency);
fprintf('  RMSE improvement: X=%.1f%%, Y=%.1f%%, Z=%.1f%%\n', improvement_accel);

fprintf('Gyroscope Analysis:\n');
fprintf('  Vibration ratio: %.1f%%\n', gyro_detection.summary.vibration_ratio * 100);
fprintf('  Dominant frequency: %.1f Hz\n', gyro_detection.summary.dominant_frequency);
fprintf('  RMSE improvement: X=%.1f%%, Y=%.1f%%, Z=%.1f%%\n', improvement_gyro);

fprintf('\n');

end