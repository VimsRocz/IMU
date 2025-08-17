function result = task7_residual_analysis(gnss_data, task3_results, task4_results, task5_results, results_dir, dt)
%TASK7_RESIDUAL_ANALYSIS  Residual analysis between fused solution and GNSS.
%   RESULT = TASK7_RESIDUAL_ANALYSIS(GNSS_DATA, TASK3_RESULTS, TASK4_RESULTS,
%   TASK5_RESULTS, RESULTS_DIR, DT) computes position and velocity residuals
%   between the fused navigation state produced in Task 5 and the GNSS
%   ground truth prepared in Task 4. Residuals are reported in NED, ECEF and
%   body frames.  Plots of residuals, attitude angles and error norms are
%   saved under RESULTS_DIR. DT specifies the IMU sample interval and
%   defaults to 0.0025 seconds.  The returned structure RESULT contains the
%   residual arrays and summary statistics.
%
%   This mirrors ``src/task7_residual_analysis.py``.
%
%   Example:
%       gnss = readtable('GNSS_X002.csv');
%       res  = task7_residual_analysis(gnss, 'Task3_results_IMU_X002_GNSS_X002.mat', ...
%               'Task4_results_IMU_X002_GNSS_X002.mat', ...
%               'IMU_X002_GNSS_X002_TRIAD_task5_results.mat', ...
%               get_results_dir(), 0.0025);

if nargin < 6 || isempty(dt)
    dt = 0.0025;
end
if nargin < 5 || isempty(results_dir)
    results_dir = get_results_dir();
end
if ~exist(results_dir, 'dir'); mkdir(results_dir); end

%% Load previous results
S5 = load(task5_results, 'x_log', 'pos_ned', 'vel_ned');
S4 = load(task4_results, 'pos_ned_gnss', 'vel_ned_gnss', 'C_e2n', 'C_n2e', 'r0');
S3 = load(task3_results, 'C_b2n');
if isfield(S5,'x_log'); x_log = S5.x_log; else; error('x_log missing'); end
if isfield(S5,'pos_ned'); pos_ned = S5.pos_ned; else; pos_ned = x_log(1:3,:)'; end
if isfield(S5,'vel_ned'); vel_ned = S5.vel_ned; else; vel_ned = x_log(4:6,:)'; end

fprintf('Estimated position size: %d x %d\n', size(pos_ned,1), size(pos_ned,2));

N = size(pos_ned,1);
t = (0:N-1)' * dt;

%% Interpolate GNSS truth to IMU timeline
if istable(gnss_data)
    t_gnss = gnss_data.Posix_Time - gnss_data.Posix_Time(1);
else
    t_gnss = gnss_data(:,7) - gnss_data(1,7); % assume Posix_Time is column 7
end
pos_truth = interp1(t_gnss, S4.pos_ned_gnss, t, 'linear', 'extrap');
vel_truth = interp1(t_gnss, S4.vel_ned_gnss, t, 'linear', 'extrap');
fprintf('Truth position size: %d x %d\n', size(pos_truth,1), size(pos_truth,2));

%% Residuals in NED
pos_residuals = pos_ned - pos_truth;
vel_residuals = vel_ned - vel_truth;

%% Residuals in ECEF and Body frames
pos_residuals_ecef = (S4.C_n2e * pos_residuals')';
vel_residuals_ecef = (S4.C_n2e * vel_residuals')';
pos_residuals_body = (S3.C_b2n' * pos_residuals')';
vel_residuals_body = (S3.C_b2n' * vel_residuals')';

%% Statistics
pos_res_mean = mean(pos_residuals,1);
pos_res_std  = std(pos_residuals,0,1);
vel_res_mean = mean(vel_residuals,1);
vel_res_std  = std(vel_residuals,0,1);

pos_error_norm = vecnorm(pos_residuals,2,2);
vel_error_norm = vecnorm(vel_residuals,2,2);
rmse_pos = sqrt(mean(pos_error_norm.^2));
rmse_vel = sqrt(mean(vel_error_norm.^2));

fprintf('Position residual mean [m]: [%.6f, %.6f, %.6f]\n', pos_res_mean);
fprintf('Velocity residual mean [m/s]: [%.6f, %.6f, %.6f]\n', vel_res_mean);

report_range('NED', pos_residuals, vel_residuals, {'North','East','Down'});
report_range('ECEF', pos_residuals_ecef, vel_residuals_ecef, {'X','Y','Z'});
report_range('Body', pos_residuals_body, vel_residuals_body, {'X','Y','Z'});

%% Attitude angles
att_deg = rad2deg(x_log(7:9,:));

%% Plot residuals in NED
f = figure('Visible','off','Position',[100 100 900 450]);
labels = {'North','East','Down'};
for j=1:3
    subplot(2,3,j); plot(t, pos_residuals(:,j));
    title(labels{j}); ylabel('Pos Residual [m]'); grid on;
    subplot(2,3,3+j); plot(t, vel_residuals(:,j));
    xlabel('Time [s]'); ylabel('Vel Residual [m/s]'); grid on;
end
sgtitle('Task 7 Residuals (NED)');
set(f,'PaperPositionMode','auto');
pdf = fullfile(results_dir, 'task7_residuals_position_velocity.pdf');
print(f,pdf,'-dpdf','-bestfit');
print(f,replace(pdf,'.pdf','.png'),'-dpng');
close(f);

%% Plot attitude angles
f = figure('Visible','off');
angles = {'Roll','Pitch','Yaw'};
for j=1:3
    subplot(3,1,j); plot(t, att_deg(j,:));
    ylabel([angles{j} ' [deg]']); grid on;
    if j==3; xlabel('Time [s]'); end
end
sgtitle('Task 7 Attitude Angles');
set(f,'PaperPositionMode','auto');
pdf = fullfile(results_dir, 'task7_attitude_angles_euler.pdf');
print(f,pdf,'-dpdf','-bestfit');
print(f,replace(pdf,'.pdf','.png'),'-dpng');
close(f);

%% Plot error norms
f = figure('Visible','off');
plot(t, pos_error_norm, 'DisplayName','|pos error|'); hold on;
plot(t, vel_error_norm, 'DisplayName','|vel error|');
xlabel('Time [s]'); ylabel('Error Norm'); legend; grid on;
set(f,'PaperPositionMode','auto');
pdf = fullfile(results_dir, 'task7_error_norms.pdf');
print(f,pdf,'-dpdf','-bestfit');
print(f,replace(pdf,'.pdf','.png'),'-dpng');
close(f);

%% Difference Truth - Fused over time
subtask7_5_plot(t, pos_truth, pos_ned, vel_truth, vel_ned, x_log, S4.C_n2e, results_dir);

%% Save results
out_file = fullfile(results_dir, 'task7_results.mat');
save(out_file, 'pos_residuals', 'vel_residuals', 'pos_residuals_ecef', ...
    'vel_residuals_ecef', 'pos_residuals_body', 'vel_residuals_body', ...
    'rmse_pos', 'rmse_vel', 'pos_error_norm', 'vel_error_norm');

result = struct('pos_residuals', pos_residuals, 'vel_residuals', vel_residuals, ...
    'pos_residuals_ecef', pos_residuals_ecef, 'vel_residuals_ecef', vel_residuals_ecef, ...
    'pos_residuals_body', pos_residuals_body, 'vel_residuals_body', vel_residuals_body, ...
    'rmse_pos', rmse_pos, 'rmse_vel', rmse_vel, 'pos_error_norm', pos_error_norm, ...
    'vel_error_norm', vel_error_norm);
end

function report_range(frame, pos_diff, vel_diff, labels)
    pos_thr = 1.0; vel_thr = 1.0;
    for i=1:3
        dp = pos_diff(:,i); dv = vel_diff(:,i);
        fprintf('%s %s position diff range: %.2f m to %.2f m. ', frame, labels{i}, min(dp), max(dp));
        idx = find(abs(dp) > pos_thr);
        if ~isempty(idx)
            fprintf('%d samples exceed %.1f m\n', numel(idx), pos_thr);
        else
            fprintf('No samples exceed %.1f m\n', pos_thr);
        end
        fprintf('%s %s velocity diff range: %.2f m/s to %.2f m/s. ', frame, labels{i}, min(dv), max(dv));
        idx = find(abs(dv) > vel_thr);
        if ~isempty(idx)
            fprintf('%d samples exceed %.1f m/s\n', numel(idx), vel_thr);
        else
            fprintf('No samples exceed %.1f m/s\n', vel_thr);
        end
    end
end

function subtask7_5_plot(time, truth_pos_ned, fused_pos_ned, truth_vel_ned, fused_vel_ned, x_log, C_n2e, out_dir)
    time = time - time(1);
    diff_pos_ned = truth_pos_ned - fused_pos_ned;
    diff_vel_ned = truth_vel_ned - fused_vel_ned;
    labels = {'North','East','Down'};
    do_plot(diff_pos_ned, diff_vel_ned, labels, 'NED');
    diff_pos_ecef = (C_n2e' * diff_pos_ned')';
    diff_vel_ecef = (C_n2e' * diff_vel_ned')';
    do_plot(diff_pos_ecef, diff_vel_ecef, {'X','Y','Z'}, 'ECEF');
    n = size(diff_pos_ned,1);
    diff_pos_body = zeros(n,3); diff_vel_body = zeros(n,3);
    for k=1:n
        Rb2n = eul_to_dcm(x_log(7:9,k)');
        diff_pos_body(k,:) = (Rb2n' * diff_pos_ned(k,:)')';
        diff_vel_body(k,:) = (Rb2n' * diff_vel_ned(k,:)')';
    end
    do_plot(diff_pos_body, diff_vel_body, {'X','Y','Z'}, 'Body');

    function do_plot(dp, dv, labs, frame)
        f = figure('Visible','off','Position',[100 100 900 450]);
        for j=1:3
            subplot(2,3,j); plot(time, dp(:,j));
            title(labs{j}); ylabel('Difference [m]'); grid on;
            subplot(2,3,3+j); plot(time, dv(:,j));
            xlabel('Time [s]'); ylabel('Difference [m/s]'); grid on;
        end
        sgtitle(['Truth - Fused Differences (' frame ' Frame)']);
        set(f,'PaperPositionMode','auto');
        base = fullfile(out_dir, ['task7_diff_truth_fused_over_time_' frame]);
        print(f,[base '.pdf'],'-dpdf','-bestfit');
        print(f,[base '.png'],'-dpng');
        close(f);
    end
end

function R = eul_to_dcm(eul)
%EUL_TO_DCM Convert roll-pitch-yaw angles to direction cosine matrix.
    r = eul(1); p = eul(2); y = eul(3);
    cr = cos(r); sr = sin(r);
    cp = cos(p); sp = sin(p);
    cy = cos(y); sy = sin(y);
    R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
         -sp,   cp*sr,            cp*cr];
end
