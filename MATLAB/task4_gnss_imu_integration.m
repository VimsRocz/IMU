function task4_gnss_imu_integration(gnss_data, imu_data, task1_results, task2_results, task3_results, results_dir, dt, cfg)
%TASK4_GNSS_IMU_INTEGRATION  Integrate IMU data and compare with GNSS.
%   TASK4_GNSS_IMU_INTEGRATION(GNSS_DATA, IMU_DATA, TASK1_RESULTS,
%   TASK2_RESULTS, TASK3_RESULTS, RESULTS_DIR, DT) performs the Task 4
%   operations of the GNSS/IMU fusion pipeline. GNSS_DATA is a table loaded
%   from the CSV file containing ECEF position and velocity columns.
%   IMU_DATA is a struct with fields ``accel`` and ``gyro`` in the body
%   frame. The auxiliary MAT files from Tasks 1--3 are used to apply bias
%   corrections, obtain the gravity vector and the initial attitude. Plots
%   and a MAT-file are written to RESULTS_DIR.  The integration is carried
%   out in the NED frame using the same approach as the Python
%   implementation.
%
%   This function mirrors ``task4_gnss_imu_integration`` in the Python
%   codebase.
%
%   Example:
%       gnss = readtable('GNSS_X002.csv');
%       imu  = load('IMU_X002.mat');
%       task4_gnss_imu_integration(gnss, imu, 'Task1_init_IMU_X002_GNSS_X002_TRIAD.mat', ...
%                                  'Task2_IMU_biases.mat', 'Task3_results_IMU_X002_GNSS_X002.mat', ...
%                                  get_results_dir(), 0.0025);

addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));
if nargin < 7 || isempty(dt)
    dt = 0.0025; % default sample period
end
if nargin < 8 || isempty(cfg)
    try
        cfg = evalin('caller','cfg');
    catch
        cfg = default_cfg();
    end
end

%% Load previous task outputs
S1 = load(task1_results, 'gravity_ned', 'latitude', 'longitude');
S2 = load(task2_results, 'acc_bias', 'gyro_bias');
S3 = load(task3_results, 'C_b2n');
if isfield(S1, 'gravity_ned'); g_ned = S1.gravity_ned(:); else; error('gravity_ned missing'); end
if isfield(S2, 'acc_bias'); acc_bias = S2.acc_bias(:)'; else; error('acc_bias missing'); end
if isfield(S2, 'gyro_bias'); gyro_bias = S2.gyro_bias(:)'; else; error('gyro_bias missing'); end
if isfield(S3, 'C_b2n'); C_b2n = S3.C_b2n; else; error('C_b2n missing'); end

%% GNSS processing
ecef_pos = [gnss_data.X_ECEF_m, gnss_data.Y_ECEF_m, gnss_data.Z_ECEF_m];
ecef_vel = [gnss_data.VX_ECEF_mps, gnss_data.VY_ECEF_mps, gnss_data.VZ_ECEF_mps];
lat_ref = deg2rad(gnss_data.Latitude_deg(1));
lon_ref = deg2rad(gnss_data.Longitude_deg(1));
r0 = ecef_pos(1,:).';
C_e2n = compute_C_ECEF_to_NED(lat_ref, lon_ref);
C_n2e = C_e2n';
M = size(ecef_pos,1);
pos_ned_gnss = zeros(M,3);
vel_ned_gnss = zeros(M,3);
for k=1:M
    pos_ned_gnss(k,:) = (C_e2n*(ecef_pos(k,:).'-r0)).';
    vel_ned_gnss(k,:) = (C_e2n*ecef_vel(k,:).').';
end
accel_ned_gnss = [zeros(1,3); diff(vel_ned_gnss)./diff(gnss_data.Posix_Time)];

%% IMU correction
scale_factor = 0.8368;
accel_corrected = (imu_data.accel - acc_bias).*scale_factor;
gyro_corrected  = imu_data.gyro - gyro_bias;
accel_ned = (C_b2n*accel_corrected.').';

%% IMU integration
N = size(accel_ned,1);
pos_imu_ned = zeros(N,3);
vel_imu_ned = zeros(N,3);
vel_imu_ned(1,:) = vel_ned_gnss(1,:);
for i=2:N
    vel_imu_ned(i,:) = vel_imu_ned(i-1,:) + (accel_ned(i,:) - g_ned.')*dt;
    pos_imu_ned(i,:) = pos_imu_ned(i-1,:) + vel_imu_ned(i,:)*dt;
end

%% Frame transformations
accel_ecef_gnss = [zeros(1,3); diff(ecef_vel)./diff(gnss_data.Posix_Time)];
C_n2b = C_b2n';

% GNSS derived quantities
pos_body_gnss = (C_n2b*pos_ned_gnss.').';
vel_body_gnss = (C_n2b*vel_ned_gnss.').';
accel_body_gnss = (C_n2b*accel_ned_gnss.').';

derived_gnss.ecef.acc = accel_ecef_gnss;
derived_gnss.body.pos = pos_body_gnss;
derived_gnss.body.vel = vel_body_gnss;
derived_gnss.body.acc = accel_body_gnss;
derived_gnss.ned.pos  = pos_ned_gnss;
derived_gnss.ned.vel  = vel_ned_gnss;
derived_gnss.ned.acc  = accel_ned_gnss;

% IMU derived quantities
pos_ecef_imu = (C_n2e*pos_imu_ned.').' + r0.';
vel_ecef_imu = (C_n2e*vel_imu_ned.').';
accel_ecef_imu = (C_n2e*accel_ned.').';
pos_imu_body = (C_n2b*pos_imu_ned.').';
vel_imu_body = (C_n2b*vel_imu_ned.').';

derived_imu.ecef.pos = pos_ecef_imu;
derived_imu.ecef.vel = vel_ecef_imu;
derived_imu.ecef.acc = accel_ecef_imu;
derived_imu.body.pos = pos_imu_body;
derived_imu.body.vel = vel_imu_body;
derived_imu.ned.pos  = pos_imu_ned;
derived_imu.ned.vel  = vel_imu_ned;
derived_imu.ned.acc  = accel_ned;

%% Validation information
rms_acc = sqrt(mean(sum(accel_ned_gnss.^2,2)));
fprintf('GNSS NED pos first=[%.2f %.2f %.2f], last=[%.2f %.2f %.2f]\n', ...
        pos_ned_gnss(1,:), pos_ned_gnss(end,:));
fprintf('GNSS accel RMS = %.4f m/s^2\n', rms_acc);
fprintf('Applied accelerometer scale factor = %.4f, bias = [%.4f %.4f %.4f]\n', ...
        scale_factor, acc_bias);

%% Plotting
fig = figure('Visible','off');
for i=1:3
    subplot(3,1,i); hold on;
    plot(pos_ned_gnss(:,i),'b');
    plot(pos_imu_ned(:,i),'r');
    grid on; xlabel('Sample'); ylabel('m');
    if i==1, title('North Position'); elseif i==2, title('East Position'); else, title('Down Position'); end
end
save_plot(fig, 'IMU', 'GNSS', 'TRIAD', 4, cfg.plots.save_pdf, cfg.plots.save_png);

%% Save results
out_file = fullfile(results_dir,'Task4_results_IMU_GNSS.mat');
save(out_file,'pos_ned_gnss','vel_ned_gnss','accel_ned_gnss', ...
                 'pos_imu_ned','vel_imu_ned','accel_ned','C_e2n','C_n2e','r0','lat_ref','lon_ref', ...
                 'derived_gnss','derived_imu');
end
