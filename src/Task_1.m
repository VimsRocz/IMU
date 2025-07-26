function [R_bn, q_bn] = Task_1(imu_file, gnss_file, method)
%TASK_1  Perform TRIAD initial attitude estimation for one dataset pair.
%   [R_BN, Q_BN] = TASK_1(IMU_FILE, GNSS_FILE, METHOD) reads the IMU and GNSS
%   files, derives reference vectors in the NED frame, measures the body-frame
%   vectors and solves for the initial attitude using the TRIAD algorithm.
%   Results are saved under ``results/<IMU>_<GNSS>_<METHOD>/`` as
%   ``initial_attitude.mat`` and a PDF plot of the orientation.
%
%   METHOD defaults to ``'TRIAD'``.

    if nargin < 3 || isempty(method)
        method = 'TRIAD';
    end

    [~, imu_name, ~]  = fileparts(imu_file);
    [~, gnss_name, ~] = fileparts(gnss_file);
    tag = sprintf('%s_%s_%s', imu_name, gnss_name, method);
    out_dir = fullfile('results', tag);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    imu = read_imu(imu_file);
    gnss = read_gnss(gnss_file);

    idx = find((gnss.X_ECEF_m ~= 0) | (gnss.Y_ECEF_m ~= 0) | (gnss.Z_ECEF_m ~= 0), 1, 'first');
    if isempty(idx)
        error('Task_1:NoValidGNSS','No valid ECEF coordinates found in %s', gnss_file);
    end
    [lat_deg, lon_deg, ~] = ecef2geodetic(gnss.X_ECEF_m(idx), gnss.Y_ECEF_m(idx), gnss.Z_ECEF_m(idx));
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);

    g_ned = [0; 0; constants.GRAVITY];
    omega_ie_ned = constants.EARTH_RATE * [cos(lat); 0; -sin(lat)];

    C_e2n = ecef2ned_matrix(lat, lon);
    vel_ned = C_e2n * [gnss.VX_ECEF_mps(idx); gnss.VY_ECEF_mps(idx); gnss.VZ_ECEF_mps(idx)]; %#ok<NASGU>

    dt = mean(diff(imu.time_s(1:min(100,end))));
    gyro = imu.dtheta / dt;
    acc  = imu.dv / dt;
    [start_idx, end_idx] = detect_static_interval(acc, gyro);
    acc_mean = mean(acc(start_idx:end_idx, :), 1)';
    gyro_mean = mean(gyro(start_idx:end_idx, :), 1)';
    scale = constants.GRAVITY / norm(acc_mean);
    acc_mean = acc_mean * scale;
    g_body = -acc_mean;
    omega_body = gyro_mean;

    [R_bn, q_bn] = triad_algorithm(g_body, omega_body, g_ned, omega_ie_ned);

    save_attitude_output(out_dir, R_bn, q_bn);
    plot_initial_orientation(R_bn, fullfile(out_dir, 'initial_orientation.pdf'));
end
