%% GNSS_IMU_Fusion_Single_script.m - MATLAB mirror of GNSS_IMU_Fusion_Single_script.py
% Mirror of the Python script GNSS_IMU_Fusion_Single_script.py
% This script performs IMU/GNSS alignment, dead-reckoning and fusion
% using the TRIAD method and two alternative Wahba solutions. Results and
% plots are stored under the 'MATLAB/results/' directory.
% If you run the Python helper scripts, install filterpy with:
%   pip install filterpy --no-binary :all:
% and install build tools if required:
%   sudo apt install build-essential python3-dev
%   pip install filterpy --no-binary :all:
%
% The dataset filenames can be changed below. Each logical block is
% annotated with "Subtask X.Y" comments for clarity.

% ------------ User configuration -----------------------------------------
% Determine repository root based on this script's location
script_dir = fileparts(mfilename('fullpath'));
root_dir   = fileparts(script_dir);

% Dataset filenames relative to repository root
% Multiple logs can be listed here and are loaded into workspace
imu_files  = { 'IMU_X001.dat', 'IMU_X002.dat', 'IMU_X003.dat' };
gnss_files = { 'GNSS_X001.csv', 'GNSS_X002.csv', 'GNSS_X002.csv' };

% Pre-load all logs for convenience.  Task 1--4 below use the first pair
% by default, but the other tables are kept in memory for analysis.
imu_data_all  = cell(size(imu_files));
gnss_data_all = cell(size(gnss_files));
for i = 1:numel(imu_files)
    fname = fullfile(root_dir, imu_files{i});
    fprintf('Loaded IMU file %s\n', fname);
    imu_data_all{i}  = readmatrix(fname);
end
for i = 1:numel(gnss_files)
    fname = fullfile(root_dir, gnss_files{i});
    fprintf('Loaded GNSS file %s\n', fname);
    gnss_data_all{i} = readtable(fname);
end

% Use the first dataset for the processing below
imu_file  = fullfile(root_dir, imu_files{1});
gnss_file = fullfile(root_dir, gnss_files{1});
[~, imu_name, ~] = fileparts(imu_file);

% Create results directory at repository root
results_dir = fullfile(root_dir, 'MATLAB/results');
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

% Ensure this script's directory is on the MATLAB path
addpath(script_dir);

%% ========================================================================
%% Task 1: Define reference vectors in NED
%% ========================================================================
fprintf('\nTASK 1: Define reference vectors in NED\n');

% Subtask 1.1: Load GNSS ECEF file and convert first valid row to lat/lon
fprintf('Subtask 1.1: Loading GNSS file %s ...\n', gnss_file);
try
    gnss_tbl = readtable(gnss_file);
catch ME
    error('Failed to read %s: %s', gnss_file, ME.message);
end
idx = find(gnss_tbl.X_ECEF_m ~= 0 | gnss_tbl.Y_ECEF_m ~= 0 | ...
            gnss_tbl.Z_ECEF_m ~= 0, 1, 'first');
if isempty(idx); error('No valid ECEF row found in %s', gnss_file); end
[x0,y0,z0] = deal(gnss_tbl.X_ECEF_m(idx), gnss_tbl.Y_ECEF_m(idx), ...
                   gnss_tbl.Z_ECEF_m(idx));
[lat0, lon0, ~] = ecef2geodetic(x0, y0, z0);
fprintf(' -> lat0=%.6f deg, lon0=%.6f deg\n', lat0, lon0);

% Subtask 1.2: Define gravity vector in NED
fprintf('Subtask 1.2: Defining gravity vector\n');
g_ned = [0;0;constants.GRAVITY];

% Subtask 1.3: Compute Earth rate in NED
fprintf('Subtask 1.3: Computing Earth rotation rate\n');
omega_ie = constants.EARTH_RATE; % [rad/s]
omega_ie_ned = omega_ie * [cosd(lat0); 0; sind(lat0)];

% Subtask 1.4: Validate magnitudes
assert(abs(norm(g_ned) - constants.GRAVITY) < 1e-3, 'Gravity magnitude check failed');
assert(abs(norm(omega_ie_ned) - omega_ie) < 1e-6, 'Earth rate magnitude check failed');

% Subtask 1.5: Plot lat0, lon0 on world map
fprintf('Subtask 1.5: Plotting initial location\n');
load coastlines
fig = figure('Visible','off');
plot(coastlon, coastlat, 'k.'); hold on; grid on;
plot(lon0, lat0, 'ro','MarkerSize',8,'LineWidth',2);
title('Initial Location'); xlabel('Longitude'); ylabel('Latitude');
set(gcf,'PaperPositionMode','auto');
loc_file = fullfile(results_dir, 'Task1_location_map.pdf');
saveas(fig, loc_file);
close(fig);

%% ========================================================================
%% Task 2: Measure vectors in the body frame
%% ========================================================================
fprintf('\nTASK 2: Measure vectors in the body frame\n');

% Subtask 2.1: Load IMU file
fprintf('Subtask 2.1: Loading IMU file %s ...\n', imu_file);
try
    imu = readmatrix(imu_file);
catch ME
    error('Failed to read %s: %s', imu_file, ME.message);
end
if size(imu,2) < 8; error('Unexpected IMU format'); end

% Convert increments to rates
if size(imu,1) > 1
    dt_imu = mean(diff(imu(1:100,2)));
else
    dt_imu = 1/400; % default
end
accel = imu(:,6:8) / dt_imu; % m/s^2
gyro  = imu(:,3:5) / dt_imu; % rad/s

% Subtask 2.2: Use first 4000 samples for static bias estimation
fprintf('Subtask 2.2: Using first 4000 samples for static bias\n');
N_static = min(4000, size(accel,1));
fprintf(' -> N_{static} = %d samples\n', N_static);

% Additionally compute a static mask over the entire dataset for ZUPT
win = 200; % window for variance computation
acc_var  = sliding_variance(accel, win);
gyro_var = sliding_variance(gyro,  win);
stat_mask = max(acc_var,[],2) < 0.01 & max(gyro_var,[],2) < 1e-6;

% Subtask 2.3: Compute mean accel and gyro over static window
accel_meas = mean(accel(1:N_static,:),1)';
gyro_meas  = mean(gyro(1:N_static,:),1)';

% Subtask 2.4: Convert to body vectors and report biases
fprintf('Subtask 2.4: Computing body-frame vectors and biases\n');
g_body = -accel_meas;
omega_ie_body = gyro_meas;
accel_bias = accel_meas;
gyro_bias  = gyro_meas;
fprintf(' -> accel bias [m/s^2]: [% .4e % .4e % .4e]\n', accel_bias);
fprintf(' -> gyro  bias [rad/s]: [% .4e % .4e % .4e]\n', gyro_bias);

%% ========================================================================
%% Task 3: Solve Wahba''s problem for initial attitude
%% ========================================================================
fprintf('\nTASK 3: Solve Wahba\''s problem for initial attitude\n');

% Subtask 3.1: Assemble vector pairs
v1_n = g_ned;       v2_n = omega_ie_ned;
v1_b = g_body;      v2_b = omega_ie_body;

% Subtask 3.2: TRIAD method
fprintf('Subtask 3.2: Computing TRIAD solution\n');
R_tri = triad_method(v1_n, v2_n, v1_b, v2_b);

% Subtask 3.3: Davenport q-method
fprintf('Subtask 3.3: Computing Davenport q-method\n');
R_dav = davenport_method(v1_n, v2_n, v1_b, v2_b);

% Subtask 3.4: SVD-based Procrustes
fprintf('Subtask 3.4: Computing SVD Procrustes solution\n');
R_svd = svd_procrustes(v1_n, v2_n, v1_b, v2_b);

% Subtask 3.5: Convert each DCM to quaternion and Euler angles
quat_tri = dcm2quat_custom(R_tri); eul_tri = quat2euler(quat_tri);
quat_dav = dcm2quat_custom(R_dav); eul_dav = quat2euler(quat_dav);
quat_svd = dcm2quat_custom(R_svd); eul_svd = quat2euler(quat_svd);
fprintf(' -> TRIAD  Euler [deg]: %7.3f %7.3f %7.3f\n', rad2deg(eul_tri));
fprintf(' -> Davenport Euler [deg]: %7.3f %7.3f %7.3f\n', rad2deg(eul_dav));
fprintf(' -> SVD     Euler [deg]: %7.3f %7.3f %7.3f\n', rad2deg(eul_svd));

% Subtask 3.6: Compute alignment errors
err_tri = [norm(R_tri*g_body - g_ned), norm(R_tri*omega_ie_body - omega_ie_ned)];
err_dav = [norm(R_dav*g_body - g_ned), norm(R_dav*omega_ie_body - omega_ie_ned)];
err_svd = [norm(R_svd*g_body - g_ned), norm(R_svd*omega_ie_body - omega_ie_ned)];

% Subtask 3.7: Plot error comparisons and quaternion components
fprintf('Subtask 3.7: Plotting Task 3 results\n');
fig = figure('Visible','off');
bar([err_tri; err_dav; err_svd]);
set(gca,'XTickLabel',{'TRIAD','Davenport','SVD'});
legend({'Gravity','Earth-rate'}); ylabel('Error');
title('Alignment Errors'); grid on;
set(gcf,'PaperPositionMode','auto');
err_file = fullfile(results_dir,'Task3_errors.pdf');
saveas(fig, err_file); close(fig);

fig = figure('Visible','off');
quat_mat = [quat_tri; quat_dav; quat_svd];
bar(quat_mat);
set(gca,'XTickLabel',{'q0','q1','q2','q3'});
legend({'TRIAD','Davenport','SVD'}); grid on;
title('Quaternion Components');
set(gcf,'PaperPositionMode','auto');
q_file = fullfile(results_dir,'Task3_quaternions.pdf');
saveas(fig, q_file); close(fig);

% Subtask 3.8: Save all DCMs/quaternions
fprintf('Subtask 3.8: Saving Task 3 results\n');
task3_results.TRIAD.R = R_tri; task3_results.TRIAD.q = quat_tri;
task3_results.Davenport.R = R_dav; task3_results.Davenport.q = quat_dav;
task3_results.SVD.R = R_svd; task3_results.SVD.q = quat_svd;
save(fullfile(results_dir,'Task3_results.mat'),'task3_results');

% Store method names and rotation matrices for later tasks
methods = {'TRIAD','Davenport','SVD'};
R_methods = {R_tri, R_dav, R_svd};
method_colors = {'r','g','b'};

%% ========================================================================
%% Task 4: GNSS + IMU dead-reckoning comparison
%% ========================================================================
fprintf('\nTASK 4: GNSS + IMU dead-reckoning comparison\n');

% Subtask 4.1: Load DCM from Task 3 (TRIAD)
R_nb = task3_results.TRIAD.R;

% Subtask 4.2: Reload GNSS and convert all ECEF to NED
pos_ecef = [gnss_tbl.X_ECEF_m gnss_tbl.Y_ECEF_m gnss_tbl.Z_ECEF_m];
vel_ecef = [gnss_tbl.VX_ECEF_mps gnss_tbl.VY_ECEF_mps gnss_tbl.VZ_ECEF_mps];
C_e2n = compute_C_ECEF_to_NED(deg2rad(lat0), deg2rad(lon0));
r0 = [x0; y0; z0];
pos_ned = (C_e2n*(pos_ecef' - r0))';
vel_ned = (C_e2n*vel_ecef')';
dt_gnss = [diff(gnss_tbl.Posix_Time); mean(diff(gnss_tbl.Posix_Time))];
acc_ned_gnss = [zeros(1,3); diff(vel_ned)./dt_gnss(1:end-1)];

% Subtask 4.3 is GNSS accel computed above

% Subtask 4.4: Subtract biases and rotate accelerations for each method
f_b = accel - accel_bias';
a_ned = cell(size(methods));
vel_ins = cell(size(methods));
pos_ins = cell(size(methods));
for m=1:numel(methods)
    a_ned{m} = (R_methods{m} * f_b')' + repmat([0 0 constants.GRAVITY],size(f_b,1),1);
    vel_ins{m} = zeros(size(a_ned{m}));
    pos_ins{m} = zeros(size(a_ned{m}));
    for k=2:size(a_ned{m},1)
        vel_ins{m}(k,:) = vel_ins{m}(k-1,:) + a_ned{m}(k-1,:)*dt_imu;
        pos_ins{m}(k,:) = pos_ins{m}(k-1,:) + vel_ins{m}(k-1,:)*dt_imu + 0.5*a_ned{m}(k-1,:)*dt_imu^2;
    end
end

% Subtask 4.5: Plot GNSS vs INS for all methods (NED)
fprintf('Subtask 4.5: Plotting dead-reckoning comparison for all methods\n');
t_imu = (0:size(f_b,1)-1)*dt_imu;
t_gnss = gnss_tbl.Posix_Time - gnss_tbl.Posix_Time(1);
fprintf(' -> gnss_time range: %.2f to %.2f s\n', min(t_gnss), max(t_gnss));
fprintf(' -> imu_time  range: %.2f to %.2f s\n', t_imu(1), t_imu(end));
fig = figure('Visible','off','Units','pixels','Position',[0 0 1200 900]);
tl = tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
labels = {'North [m]','East [m]','Down [m]'};
rowTitle = {'Position','Velocity','Acceleration'};
for j=1:3
    nexttile(j); hold on; grid on;
    plot(t_gnss, pos_ned(:,j),'k--','DisplayName','GNSS');
    for m=1:numel(methods)
        plot(t_imu, pos_ins{m}(:,j),'Color',method_colors{m},'DisplayName',methods{m});
    end
    if j==1, ylabel(rowTitle{1}); end
    title(labels{j}); if j==3, legend; end
    nexttile(3+j); hold on; grid on;
    plot(t_gnss, vel_ned(:,j),'k--');
    for m=1:numel(methods)
        plot(t_imu, vel_ins{m}(:,j),'Color',method_colors{m});
    end
    if j==1, ylabel(rowTitle{2}); end
    nexttile(6+j); hold on; grid on;
    plot(t_gnss, acc_ned_gnss(:,j),'k--');
    for m=1:numel(methods)
        plot(t_imu, a_ned{m}(:,j),'Color',method_colors{m});
    end
    if j==1, ylabel(rowTitle{3}); end
end
xlabel(tl,'Time [s]');
comp_file = fullfile(results_dir,'Task4_comparison_ned.pdf');
print(fig, comp_file, '-dpdf', '-bestfit');
close(fig);

for mi=1:numel(methods)
    outfile = fullfile(results_dir, sprintf('%s_%s_Task4_3x3.pdf', imu_name, methods{mi}));
    save_pva_grid(t_imu, pos_ins{mi}, vel_ins{mi}, a_ned{mi}, outfile);
end

% Subtask 4.6: 2-D path plot (TRIAD only for legacy)
fig = figure('Visible','off');
plot(pos_ned(:,1), pos_ned(:,2),'k','DisplayName','GNSS'); hold on;
plot(pos_ins{1}(:,1), pos_ins{1}(:,2),'r','DisplayName','INS TRIAD');
legend; xlabel('North [m]'); ylabel('East [m]'); axis equal; grid on;
title('Dead-Reckoning Comparison');
set(gcf,'PaperPositionMode','auto');
ins_file = fullfile(results_dir,'Task4_path.pdf');
saveas(fig, ins_file); close(fig);

% Subtask 4.7: Save dead-reckoning results
save(fullfile(results_dir,'Task4_results.mat'), 'pos_ins', 'vel_ins', ...
     'pos_ned', 'vel_ned');

%% ========================================================================
%% Task 5: GNSS/INS fusion with 9-state Kalman filter
%% ========================================================================
fprintf('\nTASK 5: GNSS/INS fusion with 9-state Kalman filter\n');

% Subtask 5.1: Initialize covariance and process noise
P0 = eye(9); Q = 1e-3*eye(9);

% Subtask 5.2: Measurement models
R_zupt = 1e-2*eye(3); R_gnss = blkdiag(5^2*eye(3), 0.5^2*eye(3));

% Storage for logs
N = size(imu,1); x_log = cell(size(methods)); eul_log = cell(size(methods));

for m=1:numel(methods)
    x_log{m} = zeros(9,N);
    eul_log{m} = zeros(3,N);
    pos_f = zeros(3,1); vel_f = zeros(3,1); R_f = R_methods{m}; x = zeros(9,1); P = P0;
    gnss_idx = 1; gnss_time = gnss_tbl.Posix_Time;
    for k=2:N
        dt = dt_imu;
        w_ib = gyro(k,:)' - gyro_bias;
        f_ib = accel(k,:)' - accel_bias;
        R_f = R_f * expm(skew(w_ib*dt));
        f_n = R_f * f_ib + g_ned;
        vel_f = vel_f + f_n*dt;
        pos_f = pos_f + vel_f*dt + 0.5*f_n*dt^2;
        F = [zeros(3), eye(3), zeros(3); zeros(3), zeros(3), -skew(f_n); zeros(3), zeros(3), zeros(3)];
        Phi = eye(9) + F*dt; P = Phi*P*Phi' + Q*dt;
        if stat_mask(k)
            H = [zeros(3), eye(3), zeros(3)]; z = zeros(3,1);
            [x,P] = kalman_update(x,P,z,H,R_zupt);
            vel_f = vel_f + x(4:6); pos_f = pos_f + x(1:3); R_f = (eye(3)-skew(x(7:9)))*R_f; x(:)=0;
        end
        if gnss_idx < length(gnss_time) && abs((k-1)*dt - (gnss_time(gnss_idx+1)-gnss_time(1)))<dt/2
            z = [pos_ned(gnss_idx+1,:)'; vel_ned(gnss_idx+1,:)'];
            H = [eye(3) zeros(3,6); zeros(3) eye(3) zeros(3)];
            [x,P] = kalman_update(x,P,z-[pos_f;vel_f],H,R_gnss);
            pos_f = pos_f + x(1:3); vel_f = vel_f + x(4:6); R_f = (eye(3)-skew(x(7:9)))*R_f; x(:)=0;
            gnss_idx = gnss_idx + 1;
        end
        x_log{m}(:,k) = [pos_f; vel_f; zeros(3,1)];
        eul_log{m}(:,k) = quat2euler(dcm2quat_custom(R_f));
    end
    rmse_pos(m) = sqrt(mean(vecnorm((x_log{m}(1:3,1:gnss_idx) - pos_ned(1:gnss_idx,:)').^2)));
    final_err(m) = norm(pos_f - pos_ned(end,:)');
end

% Subtask 5.4: Log estimated state done above

% Subtask 5.5: Compute RMSE and final error for each method
for m=1:numel(methods)
    fprintf('Method %s: RMSE %.3f m, final error %.3f m\n', methods{m}, rmse_pos(m), final_err(m));
end

% Subtask 5.6: Plot fused position
fprintf('Subtask 5.6: Plotting fusion results for all methods\n');
fig = figure('Visible','off');
plot(pos_ned(:,1), pos_ned(:,2),'k','DisplayName','GNSS'); hold on;
for m=1:numel(methods)
    plot(x_log{m}(1,:), x_log{m}(2,:), method_colors{m}, 'DisplayName', methods{m});
end
grid on; legend;
xlabel('North [m]'); ylabel('East [m]'); axis equal;
title('KF Position Comparison');
set(gcf,'PaperPositionMode','auto');
file_kf = fullfile(results_dir,'Task5_results_all_methods.pdf');
saveas(fig, file_kf); close(fig);

% Subtask 5.6b: Plot fused position, velocity and acceleration for all methods
% together with the GNSS measurements and the true trajectory if available.
fprintf('Subtask 5.6b: Plotting all methods with truth trajectory\n');

% Interpolate GNSS data to the IMU timeline
gnss_pos_i = interp1(t_gnss, pos_ned, t_imu, 'linear', 'extrap');
gnss_vel_i = interp1(t_gnss, vel_ned, t_imu, 'linear', 'extrap');
gnss_acc_i = interp1(t_gnss, acc_ned_gnss, t_imu, 'linear', 'extrap');

% Pre-compute fused velocity and acceleration for each method
fused_pos = cell(size(methods));
fused_vel = cell(size(methods));
fused_acc = cell(size(methods));
for mi = 1:numel(methods)
    fused_pos{mi} = x_log{mi}(1:3,:)';
    fused_vel{mi} = x_log{mi}(4:6,:)';
    fused_acc{mi} = [zeros(1,3); diff(fused_vel{mi})./diff(t_imu)];
end

% Load truth trajectory when available
truth_file = fullfile(root_dir, 'STATE_X001.txt');
truth_pos_i = [];
truth_vel_i = [];
truth_acc_i = [];
if exist(truth_file,'file')
    Ttruth = readmatrix(truth_file);
    t_truth = Ttruth(:,2) - Ttruth(1,2);
    pos_truth = (C_e2n*(Ttruth(:,3:5)' - r0))';
    vel_truth = (C_e2n*Ttruth(:,6:8)')';
    acc_truth = [zeros(1,3); diff(vel_truth)./diff(t_truth)];
    truth_pos_i = interp1(t_truth, pos_truth, t_imu, 'linear', 'extrap');
    truth_vel_i = interp1(t_truth, vel_truth, t_imu, 'linear', 'extrap');
    truth_acc_i = interp1(t_truth, acc_truth, t_imu, 'linear', 'extrap');
end

labels = {'North','East','Down'};
fig = figure('Visible','off');
for j = 1:3
    % Position
    subplot(3,3,j); hold on; grid on;
    plot(t_imu, gnss_pos_i(:,j), 'k', 'DisplayName', 'GNSS');
    if ~isempty(truth_pos_i)
        plot(t_imu, truth_pos_i(:,j), 'm--', 'DisplayName', 'Truth');
    end
    for mi = 1:numel(methods)
        plot(t_imu, fused_pos{mi}(:,j), method_colors{mi}, 'DisplayName', methods{mi});
    end
    xlabel('Time [s]'); ylabel('Position [m]');
    title(['Position ' labels{j}]);
    if j == 1
        legend('show');
    end
    % Velocity
    subplot(3,3,3+j); hold on; grid on;
    plot(t_imu, gnss_vel_i(:,j), 'k', 'HandleVisibility','off');
    if ~isempty(truth_vel_i)
        plot(t_imu, truth_vel_i(:,j), 'm--', 'HandleVisibility','off');
    end
    for mi = 1:numel(methods)
        plot(t_imu, fused_vel{mi}(:,j), method_colors{mi}, 'HandleVisibility','off');
    end
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    title(['Velocity ' labels{j}]);
    % Acceleration
    subplot(3,3,6+j); hold on; grid on;
    plot(t_imu, gnss_acc_i(:,j), 'k', 'HandleVisibility','off');
    if ~isempty(truth_acc_i)
        plot(t_imu, truth_acc_i(:,j), 'm--', 'HandleVisibility','off');
    end
    for mi = 1:numel(methods)
        plot(t_imu, fused_acc{mi}(:,j), method_colors{mi}, 'HandleVisibility','off');
    end
    xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
    title(['Acceleration ' labels{j}]);
end
sgtitle('Task 5 Comparison - All Methods with Truth');
set(gcf,'PaperPositionMode','auto');
file_alltruth = fullfile(results_dir,'task5_results_all_methods_truth.pdf');
saveas(fig, file_alltruth); close(fig);
fprintf('Saved plot: %s\n', file_alltruth);

for mi=1:numel(methods)
    outfile = fullfile(results_dir, sprintf('%s_%s_Task5_3x3.pdf', imu_name, methods{mi}));
    save_pva_grid(t_imu, fused_pos{mi}, fused_vel{mi}, fused_acc{mi}, outfile);
end

% Subtask 5.7: Save results
save(fullfile(results_dir,'Task5_results.mat'), 'x_log','eul_log');

fprintf('All tasks completed. Results stored in %s\n', results_dir);

%% -----------------------------------------------------------------------
%% Print summary metrics for each method (similar to Python script)
%% -----------------------------------------------------------------------
for mi = 1:numel(methods)
    pos_est = fused_pos{mi};
    vel_est = fused_vel{mi};
    acc_est = fused_acc{mi};
    res_pos = pos_est - gnss_pos_i;
    res_vel = vel_est - gnss_vel_i;
    rmse_pos_m  = sqrt(mean(sum(res_pos.^2,2)));
    rmse_vel_m  = sqrt(mean(sum(res_vel.^2,2)));
    final_pos_m = norm(pos_est(end,:) - gnss_pos_i(end,:));
    final_vel_m = norm(vel_est(end,:) - gnss_vel_i(end,:));
    final_acc_m = norm(acc_est(end,:) - gnss_acc_i(end,:));
    rms_resid_pos = sqrt(mean(res_pos.^2,'all'));
    rms_resid_vel = sqrt(mean(res_vel.^2,'all'));
    max_resid_pos = max(vecnorm(res_pos,2,2));
    max_resid_vel = max(vecnorm(res_vel,2,2));
    min_resid_pos = min(vecnorm(res_pos,2,2)); %#ok<NASGU>
    min_resid_vel = min(vecnorm(res_vel,2,2)); %#ok<NASGU>
    summary_line = sprintf(['[SUMMARY] method=%s imu=%s gnss=%s rmse_pos=%6.2fm ' ...
        'final_pos=%6.2fm rms_resid_pos=%6.2fm max_resid_pos=%6.2fm ' ...
        'rms_resid_vel=%6.2fm max_resid_vel=%6.2fm accel_bias=%.4f gyro_bias=%.4f ' ...
        'ZUPT_count=%d'], methods{mi}, imu_files{1}, gnss_files{1}, ...
        rmse_pos_m, final_pos_m, rms_resid_pos, max_resid_pos, ...
        rms_resid_vel, max_resid_vel, norm(accel_bias), norm(gyro_bias), sum(stat_mask));
    fprintf('%s\n', summary_line);
end

%% ========================================================================
%% Helper functions
%% ========================================================================


function var_win = sliding_variance(x, w)
    N=size(x,1); var_win=zeros(N-w+1,size(x,2));
    cs=cumsum([zeros(1,size(x,2)); x]);
    cs2=cumsum([zeros(1,size(x,2)); x.^2]);
    for i=w:N
        sumx=cs(i+1,:)-cs(i-w+1,:); sumx2=cs2(i+1,:)-cs2(i-w+1,:);
        var_win(i-w+1,:)=(sumx2 - sumx.^2/w)/w;
    end
    % pad to original length
    var_win=[var_win; repmat(var_win(end,:), w-1,1)];
end

function [i0,i1] = longest_true_segment(mask)
    d=[false;mask(:);false];
    s=find(diff(d)==1); e=find(diff(d)==-1)-1;
    [~,idx]=max(e-s+1); i0=s(idx); i1=e(idx);
end

function R = triad_method(v1_n,v2_n,v1_b,v2_b)
    t1_b = v1_b/norm(v1_b); t2_b = cross(t1_b,v2_b); t2_b=t2_b/norm(t2_b); t3_b=cross(t1_b,t2_b);
    t1_n = v1_n/norm(v1_n); t2_n = cross(t1_n,v2_n); t2_n=t2_n/norm(t2_n); t3_n=cross(t1_n,t2_n);
    R = [t1_n t2_n t3_n]*[t1_b t2_b t3_b]';
end

function R = davenport_method(v1_n,v2_n,v1_b,v2_b)
    B = v1_n*v1_b' + v2_n*v2_b';
    S = B + B'; sigma = trace(B); Z=[B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)];
    K = [sigma Z'; Z S - sigma*eye(3)];
    [V,D] = eig(K); [~,idx]=max(diag(D)); q=V(:,idx); if q(1)<0, q=-q; end
    R = quat2dcm_custom(q');
end

function R = svd_procrustes(v1_n,v2_n,v1_b,v2_b)
    B = v1_n*v1_b' + v2_n*v2_b'; [U,~,V] = svd(B); M=diag([1 1 sign(det(U*V'))]);
    R = U*M*V';
end

function q = dcm2quat_custom(R)
    tr = trace(R);
    if tr > 0
        S = sqrt(tr+1.0)*2; q0=0.25*S; q1=(R(3,2)-R(2,3))/S; q2=(R(1,3)-R(3,1))/S; q3=(R(2,1)-R(1,2))/S;
    else
        [~,i] = max([R(1,1),R(2,2),R(3,3)]);
        switch i
            case 1
                S=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2; q0=(R(3,2)-R(2,3))/S; q1=0.25*S; q2=(R(1,2)+R(2,1))/S; q3=(R(1,3)+R(3,1))/S;
            case 2
                S=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2; q0=(R(1,3)-R(3,1))/S; q1=(R(1,2)+R(2,1))/S; q2=0.25*S; q3=(R(2,3)+R(3,2))/S;
            case 3
                S=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2; q0=(R(2,1)-R(1,2))/S; q1=(R(1,3)+R(3,1))/S; q2=(R(2,3)+R(3,2))/S; q3=0.25*S;
        end
    end
    q=[q0 q1 q2 q3];
end

function R = quat2dcm_custom(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R=[1-2*(q2^2+q3^2) 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
       2*(q1*q2+q0*q3) 1-2*(q1^2+q3^2) 2*(q2*q3-q0*q1);...
       2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*(q1^2+q2^2)];
end

function eul = quat2euler(q)
    R = quat2dcm_custom(q);
    phi = atan2(R(3,2), R(3,3));
    theta = -asin(R(3,1));
    psi = atan2(R(2,1), R(1,1));
    eul = [phi; theta; psi];
end

function S = skew(w)
    S=[  0   -w(3)  w(2); w(3)   0   -w(1); -w(2) w(1)  0];
end


function save_pva_grid(t, pos_ned, vel_ned, acc_ned, outfile)
    fig = figure('Visible','off','Units','pixels','Position',[0 0 1200 900]);
    tl  = tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
    labels = {'North [m]','East [m]','Down [m]'};
    yData  = {pos_ned, vel_ned, acc_ned};
    rowTitle = {'Position','Velocity','Acceleration'};
    for row = 1:3
        for col = 1:3
            nexttile((row-1)*3+col);
            plot(t, yData{row}(:,col), 'LineWidth',1.1);
            if row == 1, title(labels{col}); end
            if col == 1, ylabel(sprintf('%s', rowTitle{row})); end
            grid on
        end
    end
    xlabel(tl, 'Time [s]');
    print(fig, outfile, '-dpdf', '-bestfit');
    close(fig);
end
