function t_imu = detect_imu_time(imu_mat, dt_fallback)
%DETECT_IMU_TIME Construct IMU time vector handling sub-second rollover.
%   t_imu = DETECT_IMU_TIME(imu_mat, dt_fallback) returns a time vector for
%   IMU samples given the raw matrix imu_mat from READMATRIX. If a
%   sub-second column exists (values in [0,1)), it unwraps per-second
%   resets; otherwise, it attempts to find a time-like column. If all
%   else fails, a constant-rate timeline is generated using DT_FALLBACK
%   (default 0.0025 s).
%
%   imu_mat: numeric matrix from readmatrix(imu_path)
%   dt_fallback: optional sample interval fallback in seconds
%
%   Returns:
%       t_imu: column vector of monotonically increasing time stamps
%   matching the number of rows in imu_mat.
%
%   This helper mirrors the MATLAB logic for Python/Matlab parity.

if nargin < 2 || isempty(dt_fallback)
    dt_fallback = 0.0025; % ~400 Hz
end

 t_imu = [];
% Try a sub-second column (last 1--3 columns)
for c = size(imu_mat,2):-1:max(1,size(imu_mat,2)-3)
    v = imu_mat(:,c);
    if all(isfinite(v)) && all(v>=0 & v<1)
        t_imu = unwrap_subsec(v);
        return;
    end
end

% Try a time-like column among first few
for c = 1:min(6,size(imu_mat,2))
    v = imu_mat(:,c);
    if all(isfinite(v)) && numel(v)>10
        dv = diff(v);
        med = median(abs(dv));
        if med>1e-4 && med<1.0
            t_imu = v(:);
            return;
        end
    end
end

% Fallback: constant-rate timeline
n = size(imu_mat,1);
t_imu = (0:n-1)' * dt_fallback;
end

function tw = unwrap_subsec(v)
%UNWRAP_SUBSEC Unwraps 0..1 second rollover clock values
%   tw = UNWRAP_SUBSEC(v) takes a vector v of sub-second timestamps and
%   returns a monotonically increasing timeline accounting for 1 Hz
%   rollovers.
tw = zeros(size(v));
acc = 0;
tw(1) = v(1);
for i = 2:numel(v)
    dv = v(i) - v(i-1);
    if dv < -0.5
        acc = acc + 1;
    end
    tw(i) = v(i) + acc;
end
end

