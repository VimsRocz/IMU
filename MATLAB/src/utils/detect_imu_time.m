function t = detect_imu_time(imu_path, dt_hint, notes)
% DETECT_IMU_TIME Stub for Python _detect_imu_time.
%   T = DETECT_IMU_TIME(IMU_PATH, DT_HINT, NOTES) returns a monotonic time
%   vector for IMU samples. Currently this stub falls back to a uniform
%   grid with sampling interval DT_HINT. NOTES, if provided, receives a
%   message indicating the fallback.

if nargin < 2 || isempty(dt_hint), dt_hint = 0.0025; end

imu = read_imu_numeric(imu_path); %#ok<NASGU>
n = size(imu, 1);
t = (0:n-1)' * dt_hint;

if nargin >= 3
    notes{end+1} = 'IMU: detect_imu_time.m stub used uniform grid.'; %#ok<AGROW>
end
end

