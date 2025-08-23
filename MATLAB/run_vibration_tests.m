function run_vibration_tests()
%RUN_VIBRATION_TESTS Test suite for vibration detection and rejection functions.
%   This function runs comprehensive tests for:
%   - vibration_model: vibration signal generation
%   - vibration_detection: vibration detection algorithms
%   - vibration_compensation: vibration removal methods
%
%   All tests include validation of outputs and error checking.

fprintf('Running Vibration Processing Test Suite\n');
fprintf('=======================================\n');

% Test configuration
test_config.fs = 400;
test_config.duration = 2; % Short duration for fast tests
test_config.tolerance = 0.05; % 5% tolerance for numerical tests

% Track test results
total_tests = 0;
passed_tests = 0;

%% Test 1: Vibration Model Tests
fprintf('\n1. Testing vibration_model...\n');

try
    [model_passed, model_total] = test_vibration_model(test_config);
    passed_tests = passed_tests + model_passed;
    total_tests = total_tests + model_total;
catch ME
    fprintf('   ERROR in vibration_model tests: %s\n', ME.message);
    total_tests = total_tests + 4; % Expected number of model tests
end

%% Test 2: Vibration Detection Tests  
fprintf('\n2. Testing vibration_detection...\n');

try
    [detect_passed, detect_total] = test_vibration_detection(test_config);
    passed_tests = passed_tests + detect_passed;
    total_tests = total_tests + detect_total;
catch ME
    fprintf('   ERROR in vibration_detection tests: %s\n', ME.message);
    total_tests = total_tests + 5; % Expected number of detection tests
end

%% Test 3: Vibration Compensation Tests
fprintf('\n3. Testing vibration_compensation...\n');

try
    [comp_passed, comp_total] = test_vibration_compensation(test_config);
    passed_tests = passed_tests + comp_passed;
    total_tests = total_tests + comp_total;
catch ME
    fprintf('   ERROR in vibration_compensation tests: %s\n', ME.message);
    total_tests = total_tests + 4; % Expected number of compensation tests
end

%% Test 4: Integration Tests
fprintf('\n4. Running integration tests...\n');

try
    [integ_passed, integ_total] = test_integration(test_config);
    passed_tests = passed_tests + integ_passed;
    total_tests = total_tests + integ_total;
catch ME
    fprintf('   ERROR in integration tests: %s\n', ME.message);
    total_tests = total_tests + 2; % Expected number of integration tests
end

%% Summary
fprintf('\n========== TEST SUMMARY ==========\n');
fprintf('Total tests: %d\n', total_tests);
fprintf('Passed: %d\n', passed_tests);
fprintf('Failed: %d\n', total_tests - passed_tests);
fprintf('Success rate: %.1f%%\n', (passed_tests/total_tests)*100);

if passed_tests == total_tests
    fprintf('*** ALL TESTS PASSED! ***\n');
else
    fprintf('*** SOME TESTS FAILED ***\n');
end

end

function [passed, total] = test_vibration_model(config)
%TEST_VIBRATION_MODEL Test vibration model function.

    passed = 0;
    total = 4;
    
    time = (0:1/config.fs:config.duration-1/config.fs)';
    
    % Test 1: Sinusoidal vibration
    fprintf('   Testing sinusoidal vibration...');
    try
        params.frequency = 50;
        params.amplitude_accel = [0.5, 0.3, 0.2];
        params.amplitude_gyro = [0.1, 0.05, 0.02];
        
        vib = vibration_model(time, 'sinusoidal', params);
        
        % Validate output structure
        assert(isstruct(vib), 'Output should be a structure');
        assert(isfield(vib, 'accel'), 'Should have accel field');
        assert(isfield(vib, 'gyro'), 'Should have gyro field');
        assert(isfield(vib, 'freq'), 'Should have freq field');
        assert(size(vib.accel, 1) == length(time), 'Accel should match time length');
        assert(size(vib.accel, 2) == 3, 'Accel should have 3 axes');
        assert(abs(vib.freq - 50) < 1e-6, 'Frequency should be 50 Hz');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 2: Motor vibration
    fprintf('   Testing motor vibration...');
    try
        params.base_freq = 30;
        params.harmonics = [1, 2];
        params.harmonic_weights = [1.0, 0.5];
        params.amplitude_accel = [0.8, 0.6, 0.4];
        params.amplitude_gyro = [0.15, 0.1, 0.05];
        
        vib = vibration_model(time, 'motor', params);
        
        assert(isstruct(vib), 'Output should be a structure');
        assert(abs(vib.freq - 30) < 1e-6, 'Base frequency should be 30 Hz');
        assert(max(abs(vib.accel(:))) > 0.1, 'Should generate significant vibration');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 3: Random vibration
    fprintf('   Testing random vibration...');
    try
        params.freq_band = [20, 80];
        params.amplitude_accel = [0.3, 0.25, 0.2];
        params.amplitude_gyro = [0.05, 0.04, 0.03];
        
        vib = vibration_model(time, 'random', params);
        
        assert(isstruct(vib), 'Output should be a structure');
        assert(vib.freq >= 20 && vib.freq <= 80, 'Frequency should be in specified band');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 4: Error handling
    fprintf('   Testing error handling...');
    try
        % Should throw error with invalid vibration type
        error_caught = false;
        try
            vib = vibration_model(time, 'invalid_type', struct());
        catch
            error_caught = true;
        end
        assert(error_caught, 'Should throw error for invalid vibration type');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
end

function [passed, total] = test_vibration_detection(config)
%TEST_VIBRATION_DETECTION Test vibration detection function.

    passed = 0;
    total = 5;
    
    time = (0:1/config.fs:config.duration-1/config.fs)';
    
    % Create test data with known vibration
    accel_clean = [zeros(length(time), 2), -9.81 * ones(length(time), 1)];
    gyro_clean = zeros(length(time), 3);
    
    % Add 50 Hz vibration
    vib_freq = 50;
    vib_signal = 0.5 * sin(2*pi*vib_freq*time);
    accel_vibrated = accel_clean + [vib_signal, 0.3*vib_signal, 0.2*vib_signal];
    gyro_vibrated = gyro_clean + [0.1*vib_signal, 0.05*vib_signal, 0.02*vib_signal];
    
    % Test 1: Frequency domain detection
    fprintf('   Testing frequency domain detection...');
    try
        result = vibration_detection(accel_vibrated, gyro_vibrated, config.fs, ...
                                   'Method', 'frequency');
        
        assert(isstruct(result), 'Output should be a structure');
        assert(isfield(result, 'vibration_detected'), 'Should have vibration_detected field');
        assert(result.vibration_detected, 'Should detect vibration');
        assert(abs(result.dominant_freq - vib_freq) < 5, 'Should identify correct frequency');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 2: Variance-based detection
    fprintf('   Testing variance-based detection...');
    try
        result = vibration_detection(accel_vibrated, gyro_vibrated, config.fs, ...
                                   'Method', 'variance');
        
        assert(result.vibration_detected, 'Should detect vibration');
        assert(result.power_ratio > 0, 'Power ratio should be positive');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 3: Energy-based detection
    fprintf('   Testing energy-based detection...');
    try
        result = vibration_detection(accel_vibrated, gyro_vibrated, config.fs, ...
                                   'Method', 'energy');
        
        assert(result.vibration_detected, 'Should detect vibration');
        assert(result.confidence > 0, 'Confidence should be positive');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 4: Combined detection
    fprintf('   Testing combined detection...');
    try
        result = vibration_detection(accel_vibrated, gyro_vibrated, config.fs, ...
                                   'Method', 'combined');
        
        assert(result.vibration_detected, 'Should detect vibration');
        assert(strcmp(result.detection_method, 'combined'), 'Method should be combined');
        assert(result.confidence >= 0 && result.confidence <= 1, 'Confidence should be 0-1');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 5: Clean data (should not detect vibration)
    fprintf('   Testing clean data detection...');
    try
        result = vibration_detection(accel_clean, gyro_clean, config.fs);
        
        assert(~result.vibration_detected || result.confidence < 0.3, ...
               'Should not detect significant vibration in clean data');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
end

function [passed, total] = test_vibration_compensation(config)
%TEST_VIBRATION_COMPENSATION Test vibration compensation function.

    passed = 0;
    total = 4;
    
    time = (0:1/config.fs:config.duration-1/config.fs)';
    
    % Create test data
    accel_clean = [zeros(length(time), 2), -9.81 * ones(length(time), 1)];
    gyro_clean = zeros(length(time), 3);
    
    % Add 50 Hz vibration
    vib_signal = 0.5 * sin(2*pi*50*time);
    accel_vibrated = accel_clean + [vib_signal, 0.3*vib_signal, 0.2*vib_signal];
    gyro_vibrated = gyro_clean + [0.1*vib_signal, 0.05*vib_signal, 0.02*vib_signal];
    
    % Test 1: Low-pass compensation
    fprintf('   Testing low-pass compensation...');
    try
        [accel_comp, gyro_comp, info] = vibration_compensation(accel_vibrated, gyro_vibrated, ...
                                                             config.fs, 'Method', 'lowpass');
        
        assert(size(accel_comp, 1) == length(time), 'Output should match input size');
        assert(size(accel_comp, 2) == 3, 'Should have 3 axes');
        assert(isstruct(info), 'Info should be a structure');
        assert(info.effectiveness >= 0 && info.effectiveness <= 1, 'Effectiveness should be 0-1');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 2: Notch compensation  
    fprintf('   Testing notch compensation...');
    try
        [accel_comp, gyro_comp, info] = vibration_compensation(accel_vibrated, gyro_vibrated, ...
                                                             config.fs, 'Method', 'notch', ...
                                                             'VibrationFreq', 50);
        
        assert(size(accel_comp, 1) == length(time), 'Output should match input size');
        assert(info.effectiveness >= 0, 'Effectiveness should be non-negative');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 3: Adaptive compensation
    fprintf('   Testing adaptive compensation...');
    try
        [accel_comp, gyro_comp, info] = vibration_compensation(accel_vibrated, gyro_vibrated, ...
                                                             config.fs, 'Method', 'adaptive');
        
        assert(size(accel_comp, 1) == length(time), 'Output should match input size');
        assert(strcmp(info.method, 'adaptive'), 'Method should be adaptive');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 4: Spectral compensation
    fprintf('   Testing spectral compensation...');
    try
        [accel_comp, gyro_comp, info] = vibration_compensation(accel_vibrated, gyro_vibrated, ...
                                                             config.fs, 'Method', 'spectral');
        
        assert(size(accel_comp, 1) == length(time), 'Output should match input size');
        assert(strcmp(info.method, 'spectral'), 'Method should be spectral');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
end

function [passed, total] = test_integration(config)
%TEST_INTEGRATION Test integration of all vibration functions.

    passed = 0;
    total = 2;
    
    time = (0:1/config.fs:config.duration-1/config.fs)';
    
    % Test 1: Full pipeline test
    fprintf('   Testing full pipeline...');
    try
        % Generate clean data
        accel_clean = [zeros(length(time), 2), -9.81 * ones(length(time), 1)];
        gyro_clean = zeros(length(time), 3);
        
        % Add vibration
        vib_params.frequency = 40;
        vib_params.amplitude_accel = [0.4, 0.3, 0.2];
        vib_params.amplitude_gyro = [0.08, 0.06, 0.04];
        
        vibration = vibration_model(time, 'sinusoidal', vib_params);
        accel_vibrated = accel_clean + vibration.accel;
        gyro_vibrated = gyro_clean + vibration.gyro;
        
        % Detect vibration
        detection = vibration_detection(accel_vibrated, gyro_vibrated, config.fs);
        
        % Compensate vibration
        [accel_comp, gyro_comp, comp_info] = vibration_compensation(accel_vibrated, gyro_vibrated, config.fs);
        
        % Validate pipeline
        assert(detection.vibration_detected, 'Should detect added vibration');
        assert(comp_info.effectiveness > 0.1, 'Should show some compensation effectiveness');
        
        % Calculate improvement
        error_before = sqrt(mean(sum((accel_vibrated - accel_clean).^2, 2)));
        error_after = sqrt(mean(sum((accel_comp - accel_clean).^2, 2)));
        improvement = (error_before - error_after) / error_before;
        
        assert(improvement > 0, 'Compensation should reduce error');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
    
    % Test 2: Performance consistency test
    fprintf('   Testing performance consistency...');
    try
        % Run the same test multiple times and check consistency
        improvements = zeros(3, 1);
        
        for i = 1:3
            % Add slight randomization
            accel_clean = [0.01*randn(length(time), 2), -9.81 * ones(length(time), 1)];
            gyro_clean = 1e-6 * randn(length(time), 3);
            
            % Add consistent vibration
            vib_params.frequency = 60;
            vib_params.amplitude_accel = [0.3, 0.2, 0.1];
            vib_params.amplitude_gyro = [0.05, 0.03, 0.02];
            
            vibration = vibration_model(time, 'sinusoidal', vib_params);
            accel_vibrated = accel_clean + vibration.accel;
            gyro_vibrated = gyro_clean + vibration.gyro;
            
            % Compensate
            [accel_comp, gyro_comp, ~] = vibration_compensation(accel_vibrated, gyro_vibrated, ...
                                                             config.fs, 'Method', 'adaptive');
            
            % Calculate improvement
            error_before = sqrt(mean(sum((accel_vibrated - accel_clean).^2, 2)));
            error_after = sqrt(mean(sum((accel_comp - accel_clean).^2, 2)));
            improvements(i) = (error_before - error_after) / error_before;
        end
        
        % Check consistency (standard deviation should be reasonable)
        std_improvement = std(improvements);
        mean_improvement = mean(improvements);
        
        assert(mean_improvement > 0.1, 'Mean improvement should be significant');
        assert(std_improvement < 0.3, 'Improvement should be reasonably consistent');
        
        fprintf(' PASSED\n');
        passed = passed + 1;
    catch ME
        fprintf(' FAILED: %s\n', ME.message);
    end
end