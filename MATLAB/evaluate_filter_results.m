function evaluate_filter_results(npz_file, output_dir, tag)
%EVALUATE_FILTER_RESULTS  Analyse filter residuals saved in NPZ format.
%
%   evaluate_filter_results(npz_file, output_dir, tag) mirrors the Python
%   ``run_evaluation_npz`` function. Residual position and velocity as well as
%   quaternion attitude history are loaded from ``npz_file`` and basic
%   statistics and plots are produced under ``output_dir``. ``tag`` is an
%   optional prefix for the output filenames. All plots use a time vector
%   shifted to start from zero so that Task 7 output aligns with Task 6.
%   The routine is divided into Subtasks 7.1-7.5 for clearer debugging output.

if nargin < 2 || isempty(output_dir)
    output_dir = get_results_dir();
end
if nargin < 3
    tag = '';
end

if ~exist(output_dir, 'dir'); mkdir(output_dir); end
prefix = '';
if ~isempty(tag); prefix = [tag '_']; end

fprintf('--- Task 7, Subtask 7.1: Loading data and checking dimensions ---\n');

data = py.numpy.load(string(npz_file));
res_pos = double(data{'residual_pos'});
res_vel = double(data{'residual_vel'});
% time_residuals historically corresponded to GNSS update times.  To
% analyse the complete trajectory we interpolate these residuals to the
% estimator time vector when available.
t_res = double(data{'time_residuals'});
if isKey(data, 'time')
    t_full = double(data{'time'});
elseif isKey(data, 'imu_time')
    t_full = double(data{'imu_time'});
elseif isKey(data, 'time_s')
    t_full = double(data{'time_s'});
else
    t_full = t_res;
end
% Ensure column vectors
t_full = t_full(:);
t_res = t_res(:);
% Interpolate residuals to the full time vector when lengths differ
if numel(t_res) ~= numel(t_full)
    res_pos = interp1(t_res, res_pos, t_full, 'linear', 'extrap');
    res_vel = interp1(t_res, res_vel, t_full, 'linear', 'extrap');
    t = t_full;
else
    t = t_res;
end
quat = double(data{'attitude_q'});
if isKey(data, 'ref_lat_rad');
    ref_lat = double(data{'ref_lat_rad'});
elseif isKey(data, 'ref_lat');
    ref_lat = double(data{'ref_lat'});
else
    ref_lat = NaN;
end
if isKey(data, 'ref_lon_rad');
    ref_lon = double(data{'ref_lon_rad'});
elseif isKey(data, 'ref_lon');
    ref_lon = double(data{'ref_lon'});
else
    ref_lon = NaN;
end
if (isnan(ref_lat) || isnan(ref_lon)) && isKey(data, 'pos_ecef_m')
    first_ecef = double(data{'pos_ecef_m'}(1,:));
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(first_ecef(1), first_ecef(2), first_ecef(3));
    if isnan(ref_lat); ref_lat = deg2rad(lat_deg); end
    if isnan(ref_lon); ref_lon = deg2rad(lon_deg); end
end

n = min([size(res_pos,1), size(res_vel,1), numel(t), size(quat,1)]);
res_pos = res_pos(1:n,:);
res_vel = res_vel(1:n,:);
t = t(1:n);
quat = quat(1:n,:);

% Shift time so plots start at zero like Task 6
t_rel = t - t(1);

fprintf('Loaded %d samples. res_pos size %s, res_vel size %s\n', n, mat2str(size(res_pos)), mat2str(size(res_vel)));

fprintf('--- Task 7, Subtask 7.2: Computing residuals ---\n');

if isKey(data, 'time') && isKey(data,'pos_ned') && isKey(data,'vel_ned')
    fused_t = double(data{'time'});
    fused_pos = double(data{'pos_ned'});
    fused_vel = double(data{'vel_ned'});
    pos_interp = interp1(fused_t, fused_pos, t, 'linear', 'extrap');
    vel_interp = interp1(fused_t, fused_vel, t, 'linear', 'extrap');
    truth_pos = pos_interp - res_pos;
    truth_vel = derive_velocity(t, truth_pos);
    res_pos = pos_interp - truth_pos;
    res_vel = vel_interp - truth_vel;
else
    fused_t = t;
    pos_interp = nan(size(res_pos));
    vel_interp = nan(size(res_vel));
    truth_pos = nan(size(res_pos));
    truth_vel = nan(size(res_vel));
end

mean_pos = mean(res_pos, 1);
std_pos = std(res_pos, [], 1);
mean_vel = mean(res_vel, 1);
std_vel = std(res_vel, [], 1);
fprintf('Position residual mean [m]: %s\n', mat2str(mean_pos,3));
fprintf('Position residual std  [m]: %s\n', mat2str(std_pos,3));
fprintf('Velocity residual mean [m/s]: %s\n', mat2str(mean_vel,3));
fprintf('Velocity residual std  [m/s]: %s\n', mat2str(std_vel,3));

fprintf('--- Task 7, Subtask 7.3: Saving residual and norm plots ---\n');

labels = {'X','Y','Z'};
f = figure('Visible','off','Position',[100 100 900 450]);
for i = 1:3
    subplot(2,3,i); plot(t_rel, res_pos(:,i)); title(labels{i}); ylabel('Pos Residual [m]'); grid on;
    subplot(2,3,i+3); plot(t_rel, res_vel(:,i)); xlabel('Time [s]'); ylabel('Vel Residual [m/s]'); grid on;
end
sgtitle('Task 7 - GNSS - Predicted Residuals');
set(f,'PaperPositionMode','auto');
pdf = fullfile(output_dir, sprintf('%stask7_3_residuals_position_velocity.pdf', prefix));
print(f, pdf, '-dpdf', '-bestfit');
close(f); fprintf('Saved %s\n', pdf);

fprintf('--- Task 7, Subtask 7.4: Plotting attitude angles ---\n');
eul = rad2deg(quat2eul(quat(:,[2 3 4 1])));
f = figure('Visible','off','Position',[100 100 600 500]);
names = {'Roll','Pitch','Yaw'};
for i = 1:3
    subplot(3,1,i); plot(t_rel, eul(:,i)); ylabel([names{i} ' [deg]']); grid on;
end
xlabel('Time [s]'); sgtitle('Task 7 - Attitude Angles');
set(f,'PaperPositionMode','auto');
pdf_att = fullfile(output_dir, sprintf('%stask7_4_attitude_angles_euler.pdf', prefix));
print(f, pdf_att, '-dpdf', '-bestfit'); close(f); fprintf('Saved %s\n', pdf_att);

norm_pos = vecnorm(res_pos,2,2);
norm_vel = vecnorm(res_vel,2,2);

f = figure('Visible','off','Position',[100 100 600 400]);
plot(t_rel, norm_pos, 'DisplayName','|pos error|'); hold on;
plot(t_rel, norm_vel, 'DisplayName','|vel error|');
xlabel('Time [s]'); ylabel('Error Norm'); legend; grid on;
set(f,'PaperPositionMode','auto');
pdf_norm = fullfile(output_dir, sprintf('%stask7_3_error_norms.pdf', prefix));
print(f, pdf_norm, '-dpdf', '-bestfit'); close(f); fprintf('Saved %s\n', pdf_norm);

% Subtask 7.5: difference truth - fused over time
fprintf('--- Task 7, Subtask 7.5: Plotting Truth - Fused differences ---\n');
if ~any(isnan(truth_pos(:))) && ~any(isnan(pos_interp(:)))
    run_id = strrep(tag, filesep, '_');
    if isempty(run_id); run_id = 'run'; end
    out_dir = fullfile(get_results_dir(), 'task7', run_id);
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    subtask7_5_diff_plot(t_rel, pos_interp, truth_pos, vel_interp, truth_vel, quat, ref_lat, ref_lon, run_id, out_dir);
else
    fprintf('Truth or fused data missing, skipping Subtask 7.5.\n');
end

rmse_pos = sqrt(mean(norm_pos.^2));
rmse_vel = sqrt(mean(norm_vel.^2));
final_pos = norm_pos(end);
final_vel = norm_vel(end);

fprintf('[SUMMARY] rmse_pos=%.3f m final_pos=%.3f m rmse_vel=%.3f m/s final_vel=%.3f m/s\n', ...
    rmse_pos, final_pos, rmse_vel, final_vel);

end


function [diff_pos_ned, diff_vel_ned] = subtask7_5_diff_plot(time, fused_pos_ned, truth_pos_ned, fused_vel_ned, truth_vel_ned, quat, ref_lat, ref_lon, run_id, out_dir)
%SUBTASK7_5_DIFF_PLOT Plot and analyse Truth minus Fused differences in NED frame.
%   [diff_pos_ned, diff_vel_ned] = SUBTASK7_5_DIFF_PLOT(time, fused_pos_ned,
%   truth_pos_ned, fused_vel_ned, truth_vel_ned, quat, ref_lat, ref_lon, run_id, out_dir)
%   computes truth_pos_ned - fused_pos_ned and truth_vel_ned - fused_vel_ned
%   and plots the results in NED, ECEF and Body frames.

time = time - time(1); % ensure relative time
diff_pos_ned = truth_pos_ned - fused_pos_ned;
diff_vel_ned = truth_vel_ned - fused_vel_ned;

% helper function to plot and report
    function do_plot(dp, dv, labs, frame)
        f = figure('Visible','off','Position',[100 100 900 450]);
        for j = 1:3
            subplot(2,3,j); plot(time, dp(:,j));
            title(labs{j}); ylabel('Difference [m]'); grid on;
            subplot(2,3,3+j); plot(time, dv(:,j));
            xlabel('Time [s]'); ylabel('Difference [m/s]'); grid on;
        end
        sgtitle(['Truth - Fused Differences (' frame ' Frame)']);
        set(f,'PaperPositionMode','auto');
        base = fullfile(out_dir, [run_id '_task7_5_diff_truth_fused_over_time_' frame]);
        print(f, [base '.pdf'], '-dpdf', '-bestfit');
        print(f, [base '.png'], '-dpng');
        close(f); fprintf('Saved %s.pdf\n', base);

        pos_thr = 1.0; vel_thr = 1.0;
        for j = 1:3
            dpp = dp(:,j); dvv = dv(:,j);
            fprintf('%s %s position diff range: %.2f m to %.2f m. ', frame, labs{j}, min(dpp), max(dpp));
            idx_p = find(abs(dpp) > pos_thr);
            if ~isempty(idx_p)
                fprintf('%d samples exceed %.1fm\n', numel(idx_p), pos_thr);
            else
                fprintf('No samples exceed %.1fm\n', pos_thr);
            end
            fprintf('%s %s velocity diff range: %.2f m/s to %.2f m/s. ', frame, labs{j}, min(dvv), max(dvv));
            idx_v = find(abs(dvv) > vel_thr);
            if ~isempty(idx_v)
                fprintf('%d samples exceed %.1fm/s\n', numel(idx_v), vel_thr);
            else
                fprintf('No samples exceed %.1fm/s.\n', vel_thr);
            end
        end
    end

% NED
do_plot(diff_pos_ned, diff_vel_ned, {'North','East','Down'}, 'NED');

% ECEF
if ~isnan(ref_lat) && ~isnan(ref_lon)
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)';
else
    C = eye(3);
end
diff_pos_ecef = (C * diff_pos_ned')';
diff_vel_ecef = (C * diff_vel_ned')';
do_plot(diff_pos_ecef, diff_vel_ecef, {'X','Y','Z'}, 'ECEF');

% Body
n = size(diff_pos_ned,1);
diff_pos_body = zeros(size(diff_pos_ned));
diff_vel_body = zeros(size(diff_vel_ned));
for k = 1:n
    Rb2n = quat2dcm_custom(quat(k,:));
    diff_pos_body(k,:) = (Rb2n' * diff_pos_ned(k,:)')';
    diff_vel_body(k,:) = (Rb2n' * diff_vel_ned(k,:)')';
end
do_plot(diff_pos_body, diff_vel_body, {'X','Y','Z'}, 'Body');
end
