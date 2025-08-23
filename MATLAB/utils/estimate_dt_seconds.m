function dt = estimate_dt_seconds(t_imu, wz_imu, t_gnss, vE, vN, lag_range_s)
%ESTIMATE_DT_SECONDS Estimate time shift dt maximizing correlation between IMU yaw-rate and GNSS heading-rate.
%   dt = ESTIMATE_DT_SECONDS(t_imu, wz_imu, t_gnss, vE, vN, lag_range_s)
%   Inputs:
%     t_imu  - IMU time (Nx1)
%     wz_imu - IMU yaw-rate (rad/s, Nx1)
%     t_gnss - GNSS time (Mx1)
%     vE,vN  - GNSS horizontal velocities (Mx1 each) in world (NED/ENU-consistent)
%     lag_range_s - scalar seconds for +/- lag search (default 2.0)
%   Output:
%     dt  - estimated time shift in seconds (positive means IMU lags GNSS)

    if nargin < 6 || isempty(lag_range_s), lag_range_s = 2.0; end
    t_imu = t_imu(:); wz_imu = wz_imu(:);
    t_gnss = t_gnss(:); vE = vE(:); vN = vN(:);

    % Build GNSS heading-rate
    psi = unwrap(atan2(vE, vN));
    dpsi = gradient(psi) ./ max(gradient(t_gnss), 1e-6);

    % Common grid for correlation
    fs = 20; % Hz
    t0 = max(t_imu(1), t_gnss(1));
    t1 = min(t_imu(end), t_gnss(end));
    if t1 <= t0
        dt = 0.0; return;
    end
    tt = (t0:1/fs:t1).';
    a = interp1(t_imu, wz_imu, tt, 'linear', 'extrap');
    b = interp1(t_gnss, dpsi,   tt, 'linear', 'extrap');

    % Detrend / zero-mean
    a = a - mean(a, 'omitnan'); b = b - mean(b, 'omitnan');
    a(~isfinite(a)) = 0; b(~isfinite(b)) = 0;

    % Correlate within lag window
    [xc, lags] = xcorr(a, b, 'normalized');
    maxlag = round(lag_range_s * fs);
    mid = ceil(numel(lags)/2);
    idx = max(1, mid-maxlag):min(numel(lags), mid+maxlag);
    [~,k] = max(abs(xc(idx)));
    dt = lags(idx(k)) / fs;
end

