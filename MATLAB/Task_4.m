function Task_4(imu_file, gnss_file, method, dataset_tag)
%TASK_4 GNSS/IMU integration with robust file handling.
%   TASK_4(IMU_FILE, GNSS_FILE, METHOD, OUTPUT_TAG) loads the MAT files
%   produced by Tasks 1--3, applies the attitude matrix from Task 3 and
%   integrates the IMU specific force.  Plots are produced in NED, ECEF and
%   body frames and saved under ``results/`` for reuse by later tasks.
%
%   IMU_FILE   - path to the raw IMU dataset (``IMU_X002.dat`` for example)
%   GNSS_FILE  - path to the GNSS CSV file
%   METHOD     - Wahba solution method used in Task 3 (e.g. ``'TRIAD'``)
%   OUTPUT_TAG - base name for all saved results.  If empty, a tag of the
%                form ``IMU_X002_GNSS_X002_TRIAD`` is generated from the
%                input filenames.
%
%   Example:
%       Task_4('data/IMU_X002.dat', 'data/GNSS_X002.csv', 'TRIAD', ...
%              'IMU_X002_GNSS_X002_TRIAD');
%
%   See also GET_RESULTS_DIR.

if nargin < 3
    method = 'TRIAD';
end
if nargin < 4 || isempty(dataset_tag)
    [~, imu_name, ~]  = fileparts(imu_file);
    [~, gnss_name, ~] = fileparts(gnss_file);
    dataset_tag = sprintf('%s_%s', imu_name, gnss_name);
else
    [~, imu_name, ~]  = fileparts(imu_file);
    [~, gnss_name, ~] = fileparts(gnss_file);
end

results_dir = get_results_dir();
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

[~, imu_name, ~]  = fileparts(imu_file);
[~, gnss_name, ~] = fileparts(gnss_file);

run_tag = sprintf('%s_%s', dataset_tag, method);

% ------------------------------------------------------------------
% Subtask 4.1: Load outputs from previous tasks
% ------------------------------------------------------------------

task1_file = fullfile(results_dir, sprintf('Task1_%s_%s.mat', dataset_tag, method));
task2_file = fullfile(results_dir, sprintf('Task2_%s_%s.mat', dataset_tag, method));
task3_file = fullfile(results_dir, sprintf('Task3_%s_%s.mat', dataset_tag, method));
out_mat    = fullfile(results_dir, sprintf('Task4_%s_%s.mat', dataset_tag, method));

fprintf('Task 4: loading previous task outputs:\n');
fprintf('  Task 1 -> %s\n', task1_file);
fprintf('  Task 2 -> %s\n', task2_file);
fprintf('  Task 3 -> %s\n', task3_file);

% ---- Task 1 ----
if isfile(task1_file)
    S1 = load(task1_file);
else
    error('Task_4:MissingTask1', 'Task 1 result file not found: %s', task1_file);
end
if isfield(S1,'lat0_rad');      lat0_rad = S1.lat0_rad;
elseif isfield(S1,'lat');        lat0_rad = deg2rad(S1.lat);
else
    error('Task_4:MissingField','Latitude field missing from %s', task1_file);
end
if isfield(S1,'lon0_rad');      lon0_rad = S1.lon0_rad;
elseif isfield(S1,'lon');        lon0_rad = deg2rad(S1.lon);
else
    error('Task_4:MissingField','Longitude field missing from %s', task1_file);
end
if isfield(S1,'gravity_ned');   gravity_ned = S1.gravity_ned(:);
elseif isfield(S1,'g_NED');      gravity_ned = S1.g_NED(:);
else
    error('Task_4:MissingField','Gravity vector missing from %s', task1_file);
end
if isfield(S1,'ref_r0'); r0_init = S1.ref_r0(:); else; r0_init = []; end

% ---- Task 2 ----
if isfile(task2_file)
    S2 = load(task2_file);
else
    error('Task_4:MissingTask2', 'Task 2 result file not found: %s', task2_file);
end
if isfield(S2,'accel_bias'); accel_bias = S2.accel_bias(:)';
elseif isfield(S2,'acc_bias');  accel_bias = S2.acc_bias(:)';
else
    error('Task_4:MissingField','Accelerometer bias missing from %s', task2_file);
end
if isfield(S2,'gyro_bias');  gyro_bias = S2.gyro_bias(:)';
else
    error('Task_4:MissingField','Gyroscope bias missing from %s', task2_file);
end
if isfield(S2,'accel_scale'); accel_scale = S2.accel_scale; else; accel_scale = 1; end
if isfield(S2,'static_start'); static_start = S2.static_start; else; static_start = 1; end
if isfield(S2,'static_end');   static_end   = S2.static_end;   else; static_end   = 1; end

% ---- Task 3 ----
if isfile(task3_file)
    S3 = load(task3_file);
else
    error('Task_4:MissingTask3', 'Task 3 result file not found: %s', task3_file);
end
if ~isfield(S3,'task3_results')
    error('Task_4:MissingField','task3_results missing from %s', task3_file);
end
task3_results = S3.task3_results;
if isfield(task3_results, method)
    if isfield(task3_results.(method), 'R')
        C_b_n = task3_results.(method).R;
    else
        error('Task_4:MissingField','Rotation matrix R missing for %s in %s', method, task3_file);
    end
else
    error('Task_4:MissingField','Method %s not found in %s', method, task3_file);
end
fprintf('Loaded rotation matrix for %s.\n', method);

% ------------------------------------------------------------------
% Subtask 4.3: Load GNSS data
% ------------------------------------------------------------------
fprintf('Subtask 4.3: Loading GNSS data.\n');

gnss = readtable(gnss_file);
fprintf('Subtask 4.4: GNSS data shape: %d x %d\n', size(gnss,1), width(gnss));

if ~isempty(r0_init)
    r0 = r0_init;
else
    r0 = [gnss.X_ECEF_m(1); gnss.Y_ECEF_m(1); gnss.Z_ECEF_m(1)];
end
C_e_n = compute_C_ECEF_to_NED(lat0_rad, lon0_rad);
fprintf('Subtask 4.5: Reference point (ECEF) = [%.1f, %.1f, %.1f]\n', r0);

pos_ecef = [gnss.X_ECEF_m, gnss.Y_ECEF_m, gnss.Z_ECEF_m]';
pos_ned  = C_e_n*(pos_ecef - r0);
vel_ecef = [gnss.VX_ECEF_mps, gnss.VY_ECEF_mps, gnss.VZ_ECEF_mps]';
vel_ned  = C_e_n*vel_ecef;
fprintf('Subtask 4.7: Converted GNSS to NED: first=[%.2f,%.2f,%.2f], last=[%.2f,%.2f,%.2f]\n', ...
    pos_ned(1,1),pos_ned(2,1),pos_ned(3,1),pos_ned(1,end),pos_ned(2,end),pos_ned(3,end));

gnss_time = gnss.Posix_Time;
dt_gnss = mean(diff(gnss_time));
acc_ned = diff(vel_ned,1,2) / dt_gnss;
fprintf('Subtask 4.8: GNSS accel RMS = %.4f m/s^2\n', rms(acc_ned(:)) );

% ------------------------------------------------------------------
% Subtask 4.9: Load IMU data and correct
% ------------------------------------------------------------------
fprintf('Subtask 4.9: Loading IMU data.\n');
imu_raw = readmatrix(imu_file);
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
fig = figure();
plot_ned_comparison(pos_ned, pos_imu, vel_ned, vel_imu, acc_ned, acc_imu);
file_ned = fullfile(results_dir, sprintf('%s_task4_results_NED.pdf', run_tag));
saveas(fig, file_ned);
saveas(fig, strrep(file_ned,'.pdf','.png'));
close(fig);
fprintf('Saved plot: %s\n', file_ned);

% -- ECEF frame --
fig = figure();
C_n_e = C_e_n';
pos_imu_ecef = C_n_e*pos_imu + r0;
vel_imu_ecef = C_n_e*vel_imu;
pos_gnss_ecef = pos_ecef;
vel_gnss_ecef = vel_ecef;
plot_ned_comparison(pos_gnss_ecef,pos_imu_ecef,vel_gnss_ecef,vel_imu_ecef,acc_ned,acc_imu);
file_ecef = fullfile(results_dir, sprintf('%s_task4_results_ECEF.pdf', run_tag));
saveas(fig, file_ecef);
saveas(fig, strrep(file_ecef,'.pdf','.png')); close(fig);
fprintf('Saved plot: %s\n', file_ecef);

% -- Body frame --
fig = figure();
plot_body_frame(accel, gyro);
file_body = fullfile(results_dir, sprintf('%s_task4_results_Body.pdf', run_tag));
saveas(fig, file_body);
saveas(fig, strrep(file_body,'.pdf','.png'));
close(fig);
fprintf('Saved plot: %s\n', file_body);

% ------------------------------------------------------------------
% Save key outputs for downstream use
% ------------------------------------------------------------------
pos_est_ned   = pos_imu;
vel_est_ned   = vel_imu;
acc_est_ned   = acc_imu;
pos_est_ecef  = pos_imu_ecef;
vel_est_ecef  = vel_imu_ecef;
save_vars = {'pos_est_ned','vel_est_ned','acc_est_ned','pos_est_ecef', ...
    'vel_est_ecef','lat0_rad','lon0_rad','gravity_ned','accel_scale','C_b_n','C_e_n','r0'};
save(out_mat, save_vars{:});
fprintf('Saved Task 4 results to %s\n', out_mat);
fprintf('Task 4 completed: All frame plots saved, variables ready for Task 5.\n');
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
