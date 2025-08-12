% Test for detect_static_interval helper.
% Generates synthetic IMU data and verifies the detected static window.

rng(0, 'twister');
static_acc = 0.001 * randn(500,3);
static_gyro = 1e-5 * randn(500,3);
moving_acc = 0.1 * randn(200,3);
moving_gyro = 0.01 * randn(200,3);

accel = [static_acc; moving_acc];
gyro = [static_gyro; moving_gyro];

[start_idx, end_idx] = detect_static_interval(accel, gyro, 50, 0.01, 1e-6, 100);

assert(start_idx <= 5, 'Static interval should start near sample 1');
assert(end_idx >= 480 && end_idx <= 520, 'Static interval should span ~500 samples');

fprintf('test_static_interval_matlab passed: [%d, %d]\n', start_idx, end_idx);

