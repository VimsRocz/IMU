function Task_1(imu_path, gnss_path, method)
%TASK_1 Initial attitude estimation using TRIAD
%   TASK_1(IMU_PATH, GNSS_PATH, METHOD) reads the given dataset pair,
%   converts GNSS velocity from ECEF to NED and computes the body->NED
%   rotation matrix using the TRIAD algorithm. The result is saved under
%   results/<IMU>_<GNSS>_TRIAD/initial_attitude.mat and a simple 3-D plot
%   of the axes is written to initial_orientation.pdf.
%
% Example:
%   Task_1('IMU_X001.dat','GNSS_X001.csv','TRIAD')

    if nargin < 3
        method = 'TRIAD';
    end

    data_imu  = utils.read_imu(imu_path);
    data_gnss = utils.read_gnss(gnss_path);

    % Use first valid GNSS sample
    idx = find(data_gnss.X_ECEF_m ~= 0 | data_gnss.Y_ECEF_m ~= 0 | data_gnss.Z_ECEF_m ~= 0, 1);
    lat = data_gnss.Latitude_deg(idx);
    lon = data_gnss.Longitude_deg(idx);
    vel_ecef = [data_gnss.VX_ECEF_mps(idx); data_gnss.VY_ECEF_mps(idx); data_gnss.VZ_ECEF_mps(idx)];

    C = utils.ecef2ned_matrix(deg2rad(lat), deg2rad(lon));
    vel_ned = C * vel_ecef;

    acc_body = data_imu.accel(1,:).';
    mag_body = data_imu.mag(1,:).';

    [R_bn, q_bn] = utils.triad_algorithm(acc_body, mag_body, vel_ned);

    [~, imu_stem, ~]  = fileparts(imu_path);
    [~, gnss_stem, ~] = fileparts(gnss_path);
    out_dir = fullfile('results', sprintf('%s_%s_%s_TRIAD', imu_stem, gnss_stem, method));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    utils.save_attitude_output(out_dir, R_bn, q_bn);
    utils.plot_initial_orientation(out_dir, R_bn);
end
