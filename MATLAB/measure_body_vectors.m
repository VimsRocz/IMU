function [dt, g_body, omega_ie_body, mag_body] = measure_body_vectors(imu_file, static_start, static_end, mag_file)
%MEASURE_BODY_VECTORS  Estimate gravity and Earth rotation in body frame.
%   This function mirrors the behaviour of the Python implementation used in
%   ``src/run_triad_only.py``.  It loads IMU_FILE, converts incremental
%   measurements to rates, applies a low-pass filter and then detects a static
%   interval for averaging.  If STATIC_START and STATIC_END are not provided the
%   interval is obtained automatically using DETECT_STATIC_INTERVAL.  When a
%   magnetometer file is supplied the mean of its first three columns is
%   returned as MAG_BODY.

if nargin < 4 || isempty(mag_file)
    mag_file = '';
end
if nargin < 3
    static_end = [];
end
if nargin < 2
    static_start = [];
end

imu_data = readmatrix(imu_file);
time_s   = imu_data(:,2);
gyro_inc = imu_data(:,3:5);
acc_inc  = imu_data(:,6:8);

if numel(time_s) > 1
    dt = mean(diff(time_s(1:min(100,end))));
else
    dt = 1/400; % default 400 Hz
end

gyro = gyro_inc / dt; % rad/s
acc  = acc_inc  / dt; % m/s^2

fs = 1/dt;
gyro = low_pass_filter(gyro, 5, fs);
acc  = low_pass_filter(acc, 5, fs);

if isempty(static_start)
    [static_start, static_end] = detect_static_interval(acc, gyro, 80, 0.01, 1e-6, 80);
else
    static_start = max(1, static_start);
    if isempty(static_end)
        static_end = size(acc,1);
    else
        static_end = min(static_end, size(acc,1));
    end
end

static_acc  = mean(acc(static_start:static_end,:),1);   % 1x3
static_gyro = mean(gyro(static_start:static_end,:),1);  % 1x3

n_static  = static_end - static_start + 1;
static_dur = n_static * dt;
total_dur  = size(acc,1) * dt;
ratio = static_dur / total_dur * 100;
fprintf('Static interval duration: %.2f s of %.2f s total (%.1f%%)\n', ...
        static_dur, total_dur, ratio);
if ratio > 90
    warning('measure_body_vectors:LargeStatic', ...
        'Static interval covers %.1f%% of the dataset. Verify motion data or adjust detection thresholds.', ratio);
end

scale = constants.GRAVITY / norm(static_acc);
static_acc = static_acc * scale;

g_body = -static_acc';           % column vector
omega_ie_body = static_gyro';    % column vector

mag_body = [];
if ~isempty(mag_file) && isfile(mag_file)
    try
        md = readmatrix(mag_file);
        if ismatrix(md) && size(md,2) >= 3
            mag_body = mean(md(:,1:3),1)';
        end
    catch ME %#ok<NASGU>
        mag_body = [];
    end
end

end
