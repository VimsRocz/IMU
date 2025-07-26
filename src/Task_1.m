function result = Task_1(imu_file, gnss_file, method)
%TASK_1  TRIAD-based initial attitude estimation.
%   RESULT = TASK_1(IMU_FILE, GNSS_FILE, METHOD) loads the IMU and GNSS logs,
%   derives the reference and body vectors and solves for the body-to-NED
%   rotation using the TRIAD algorithm.  The output rotation matrix ``R_bn``
%   and quaternion ``q_bn`` are saved under ``results/<IMU>_<GNSS>_METHOD/``.
%
%   This function mirrors the logic of ``src/run_triad_only.py`` but is
%   implemented in MATLAB for cross-language parity.
%
% See also: run_all_datasets_matlab, triad_algorithm

    if nargin < 3 || isempty(method)
        method = 'TRIAD';
    end

    [~, imu_name, ~]  = fileparts(imu_file);
    [~, gnss_name, ~] = fileparts(gnss_file);
    tag = sprintf('%s_%s_%s', imu_name, gnss_name, method);
    out_dir = fullfile('results', tag);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    % Load data
    imu  = read_imu(imu_file);
    gnss = read_gnss(gnss_file);

    % First valid GNSS sample for reference location
    idx = find((gnss.X_ECEF_m ~= 0) | (gnss.Y_ECEF_m ~= 0) | ...
               (gnss.Z_ECEF_m ~= 0), 1, 'first');
    if isempty(idx)
        error('No valid GNSS position in %s', gnss_file);
    end
    [lat_deg, lon_deg, ~] = ecef2geodetic(gnss.X_ECEF_m(idx), ...
                                         gnss.Y_ECEF_m(idx), ...
                                         gnss.Z_ECEF_m(idx));
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);
    C_e2n = ecef2ned_matrix(lat, lon);

    vel_ecef = [gnss.VX_ECEF_mps(idx), gnss.VY_ECEF_mps(idx), gnss.VZ_ECEF_mps(idx)]';
    vel_ned = C_e2n * vel_ecef;

    % Simple static interval: first 200 samples
    win = 1:min(200, numel(imu.time_s));
    g_body = -mean(imu.accel_mps2(win, :), 1)';
    omega_body = mean(imu.gyro_radps(win, :), 1)';

    g_ned = [0; 0; 9.80665];
    omega_ned = 7.292115e-5 * [cos(lat); 0; -sin(lat)];

    [R_bn, q_bn] = triad_algorithm(g_body, omega_body, g_ned, omega_ned);

    save_attitude_output(out_dir, R_bn, q_bn);
    plot_initial_orientation(R_bn, out_dir);

    result.R_bn = R_bn;
    result.q_bn = q_bn;
end
