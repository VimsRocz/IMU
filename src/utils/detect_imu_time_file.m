function t = detect_imu_time_file(imu_path, dt_hint)
%DETECT_IMU_TIME_FILE Detect IMU time vector from a file path.
%   T = DETECT_IMU_TIME_FILE(IMU_PATH, DT_HINT) reads IMU data using
%   READ_IMU_NUMERIC and invokes DETECT_IMU_TIME on the numeric matrix,
%   mirroring the Python ``_detect_imu_time`` helper.

if nargin < 2 || isempty(dt_hint)
    dt_hint = 0.0025;
end

tbl = read_imu_numeric(imu_path);
t = detect_imu_time(tbl{:,:}, dt_hint);
end

