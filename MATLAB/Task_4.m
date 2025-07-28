function Task_4( imu_path, gnss_path, method )
%TASK_4 Integrate GNSS and IMU data in NED, ECEF and body frames.
%   Task_4(IMU_PATH, GNSS_PATH, METHOD) loads the previous task results,
%   integrates the IMU specific force using the rotation matrix from Task 3
%   and compares it with the GNSS trajectory. Results and comparison plots
%   are saved under the ``results`` directory returned by GET_RESULTS_DIR.
%
%   This implementation mirrors the Python pipeline ``run_triad_only.py``
%   for cross-language parity.
%
%   See also GET_RESULTS_DIR.

if nargin < 3
    method = 'TRIAD';
end

results_dir = get_results_dir();
if ~exist(results_dir,'dir'); mkdir(results_dir); end

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
tag = sprintf('%s_%s_%s', imu_name, gnss_name, method);

% ------------------------------------------------------------------
% Subtask 4.1: Load outputs from previous tasks
% ------------------------------------------------------------------

task1_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', tag));
task2_file = fullfile(results_dir, sprintf('Task2_body_%s.mat', tag));
task3_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', tag));
out_mat    = fullfile(results_dir, sprintf('Task4_results_%s.mat', tag));
out_mat2   = fullfile(results_dir, sprintf('%s_task4_results.mat', tag));

load(task1_file,'lat0_rad','lon0_rad','gravity_ned');
load(task2_file,'accel_bias','gyro_bias','accel_scale','static_start','static_end');
load(task3_file,'task3_results');
C_b_n = task3_results.TRIAD.R;
fprintf('Subtask 4.1: Accessing rotation matrices from Task 3.\n');
fprintf(' -> Loaded C_b_n for method %s\n', method);

% ------------------------------------------------------------------
% Subtask 4.3: Load GNSS data
% ------------------------------------------------------------------
fprintf('Subtask 4.3: Loading GNSS data.\n');

gnss = readtable(gnss_path);
fprintf('Subtask 4.4: GNSS data shape: %d x %d\n', size(gnss,1), 3);

r0 = [gnss.X_ECEF_m(1); gnss.Y_ECEF_m(1); gnss.Z_ECEF_m(1)];
C_e_n = compute_C_ECEF_to_NED(lat0_rad, lon0_rad);
fprintf('Subtask 4.5: Reference point (ECEF) = [%.1f, %.1f, %.1f]\n', r0);

pos_ecef = [gnss.X_ECEF_m, gnss.Y_ECEF_m, gnss.Z_ECEF_m]';
pos_ned  = C_e_n*(pos_ecef - r0);
vel_ecef = [gnss.VX_ECEF_mps, gnss.VY_ECEF_mps, gnss.VZ_ECEF_mps]';
vel_ned  = C_e_n*vel_ecef;
fprintf('Subtask 4.7: Converted GNSS to NED: first=[%.2f,%.2f,%.2f], last=[%.2f,%.2f,%.2f]\n', ...
    pos_ned(1,1),pos_ned(2,1),pos_ned(3,1),pos_ned(1,end),pos_ned(2,end),pos_ned(3,end));

dt_gnss = mean(diff(gnss.Posix_Time));
acc_ned = diff(vel_ned,1,2) / dt_gnss;
fprintf('Subtask 4.8: GNSS accel RMS = %.4f m/s^2\n', rms(acc_ned(:)) );

% ------------------------------------------------------------------
% Subtask 4.9: Load IMU data and correct
% ------------------------------------------------------------------
fprintf('Subtask 4.9: Loading IMU data.\n');
imu_raw = readmatrix(imu_path);
dt = mean(diff(imu_raw(1:100,2)));
accel_raw = imu_raw(:,6:8) / dt;
gyro_raw  = imu_raw(:,3:5) / dt;
accel = (accel_raw - accel_bias') * accel_scale;
gyro  = gyro_raw - gyro_bias';
fprintf('Subtask 4.9: IMU data corrected: accel_bias=[%.4f %.4f %.4f], scale=%.4f\n', accel_bias, accel_scale);
fprintf('           gyro_bias=[%.4e %.4e %.4e]\n', gyro_bias);

% ------------------------------------------------------------------
% Subtask 4.10: Integrate IMU to NED
% ------------------------------------------------------------------
fprintf('Subtask 4.10: IMU dt = %.4f s\n', dt);
fprintf('Subtask 4.11: Initializing output arrays.\n');

n = size(accel,1);
pos_imu = zeros(3,n); vel_imu = zeros(3,n); acc_imu = zeros(3,n);

for k=2:n
    C_n_b = C_e_n * C_b_n;
    f_n   = C_n_b * accel(k,:)' + gravity_ned(:);
    vel_imu(:,k) = vel_imu(:,k-1) + f_n*dt;
    pos_imu(:,k) = pos_imu(:,k-1) + vel_imu(:,k-1)*dt + 0.5*f_n*dt^2;
    acc_imu(:,k) = f_n;
end
fprintf('Subtask 4.12: Integrated IMU data using %s method.\n', method);

% ------------------------------------------------------------------
% Subtask 4.13: Plot comparison
% ------------------------------------------------------------------
fprintf('Subtask 4.13: Plotting GNSS vs IMU in NED frame.\n');
fig = figure('Visible','off');
plot_ned_comparison(pos_ned, pos_imu, vel_ned, vel_imu, acc_ned, acc_imu);
file_ned = fullfile(results_dir, sprintf('%s_task4_NED.pdf', tag));
saveas(fig, file_ned);
close(fig);
fprintf('Saved plot: %s\n', file_ned);

% -- ECEF frame --
fig = figure('Visible','off');
C_n_e = C_e_n';
pos_imu_ecef = C_n_e*pos_imu + r0;
vel_imu_ecef = C_n_e*vel_imu;
pos_gnss_ecef = pos_ecef;
vel_gnss_ecef = vel_ecef;
plot_ned_comparison(pos_gnss_ecef,pos_imu_ecef,vel_gnss_ecef,vel_imu_ecef,acc_ned,acc_imu);
file_ecef = fullfile(results_dir, sprintf('%s_task4_ECEF.pdf', tag));
saveas(fig,file_ecef); close(fig);
fprintf('Saved plot: %s\n', file_ecef);

% -- Body frame --
fig = figure('Visible','off');
plot_body_frame(accel, gyro);
file_body = fullfile(results_dir, sprintf('%s_task4_BODY.pdf', tag));
saveas(fig,file_body); close(fig);
fprintf('Saved plot: %s\n', file_body);

save(out_mat, 'pos_ned','vel_ned','acc_ned','pos_imu','vel_imu','acc_imu');
movefile(out_mat,out_mat2,'f');
fprintf('Saved Task 4 results to %s\n', out_mat2);
end

% ------------------------------------------------------------------
function plot_ned_comparison(p1,p2,v1,v2,a1,a2)
    dims = {'N','E','D'};
    for i=1:3
        subplot(3,3,i); hold on; plot(p1(i,:), 'k'); plot(p2(i,:), 'b'); hold off; grid on; ylabel('m'); title(['Pos ' dims{i}]);
        subplot(3,3,i+3); hold on; plot(v1(i,:), 'k'); plot(v2(i,:), 'b'); hold off; grid on; ylabel('m/s'); title(['Vel ' dims{i}]);
        if i==3
            subplot(3,3,i+6); hold on; plot([0 size(a1,2)-1], [0 0], 'k:'); hold on; plot(a2(i,:), 'b'); hold off; grid on; ylabel('m/s^2'); title(['Acc ' dims{i}]);
        else
            subplot(3,3,i+6); hold on; plot(a1(i,:), 'k'); plot(a2(i,:), 'b'); hold off; grid on; ylabel('m/s^2'); title(['Acc ' dims{i}]);
        end
    end
end

function plot_body_frame(accel, gyro)
    dims = {'X','Y','Z'};
    for i=1:3
        subplot(2,3,i); plot(accel(:,i)); grid on; ylabel('m/s^2'); title(['Accel ' dims{i}]);
        subplot(2,3,i+3); plot(gyro(:,i));  grid on; ylabel('rad/s');  title(['Gyro ' dims{i}]);
    end
end
