function quat_logs = task6_plot_fused_trajectory(method, imu_file, gnss_file, quat_logs)
%TASK6_PLOT_FUSED_TRAJECTORY  Plot fused trajectory against truth.
%
%   quat_logs = TASK6_PLOT_FUSED_TRAJECTORY(method, imu_file, gnss_file, quat_logs)
%   replicates ``plot_task6_fused_trajectory`` from the Python code. The fused
%   estimate ``<imu_file>_<gnss_file>_<method>.mat`` and corresponding truth
%   ``<imu_file>_<gnss_file>_truth.mat`` must exist under ``output_matlab/``. Overlay
%   plots in NED and ECEF are produced together with a position error plot. The
%   quaternion history for the method is returned in ``quat_logs``.

if nargin < 4 || isempty(quat_logs)
    quat_logs = struct();
end

fused_path = fullfile('output_matlab', sprintf('%s_%s_%s.mat', imu_file, gnss_file, method));
truth_path = fullfile('output_matlab', sprintf('%s_%s_truth.mat', imu_file, gnss_file));

if ~isfile(fused_path)
    warning('Missing fused data %s', fused_path);
    return;
end
if ~isfile(truth_path)
    warning('Missing truth data %s', truth_path);
    return;
end

F = load(fused_path);
T = load(truth_path);

try
    fused_pos_ned = F.fused_pos;
    fused_vel_ned = F.fused_vel;
    fused_time = F.time_s(:);
    ref_lat = double(F.ref_lat_rad);
    ref_lon = double(F.ref_lon_rad);
catch ME
    warning('KeyError: %s in %s', ME.message, fused_path);
    return;
end

if isfield(F, 'quat_log') && ~isempty(F.quat_log)
    quat_logs.(method) = F.quat_log;
end

truth_pos_ned = T.pos_ned_m;
truth_time = T.time_s(:);

if numel(truth_time) ~= numel(fused_time) || any(abs(truth_time - fused_time) > eps)
    truth_pos_ned = interp1(truth_time, truth_pos_ned, fused_time, 'linear', 'extrap');
    truth_time = fused_time;
end

if isfield(T, 'vel_ned_ms')
    truth_vel_ned = T.vel_ned_ms;
    if size(truth_vel_ned,1) ~= numel(fused_time)
        truth_vel_ned = interp1(T.time_s, truth_vel_ned, fused_time, 'linear', 'extrap');
    end
else
    warning('derive truth velocity from position');
    truth_vel_ned = derive_velocity(truth_time, truth_pos_ned);
end

[fused_pos_ecef, fused_vel_ecef] = ned_to_ecef(fused_pos_ned, fused_vel_ned, ref_lat, ref_lon);
[truth_pos_ecef, truth_vel_ecef] = ned_to_ecef(truth_pos_ned, truth_vel_ned, ref_lat, ref_lon);

error_pos = fused_pos_ned - truth_pos_ned;
error_vel = fused_vel_ned - truth_vel_ned;
rmse_pos = sqrt(mean(sum(error_pos.^2, 2)));
rmse_vel = sqrt(mean(sum(error_vel.^2, 2)));
final_err = norm(error_pos(end, :));

for frame_name = {'NED','ECEF'}
    fr = frame_name{1};
    if strcmp(fr,'NED')
        p_f = fused_pos_ned; v_f = fused_vel_ned;
        p_t = truth_pos_ned; v_t = truth_vel_ned;
        labels = {'North','East','Down'};
    else
        p_f = fused_pos_ecef; v_f = fused_vel_ecef;
        p_t = truth_pos_ecef; v_t = truth_vel_ecef;
        labels = {'X','Y','Z'};
    end

    f1 = figure('Visible','off','Position',[100 100 600 700]);
    for i = 1:3
        subplot(3,1,i); hold on;
        plot(fused_time, p_f(:,i), 'b', 'DisplayName', sprintf('%s Fused', method));
        plot(fused_time, p_t(:,i), 'r--', 'DisplayName', 'Truth');
        title(sprintf('%s Position %s (%s)', method, labels{i}, fr));
        xlabel('Time (s)'); ylabel('Position (m)'); grid on; legend;
    end
    tightfig();
    out_name = sprintf('%s_%s_%s_task6_fused_position_%s', imu_file, gnss_file, method, lower(fr));
    print(f1, fullfile('output_matlab',[out_name '.pdf']), '-dpdf', '-bestfit');
    close(f1);

    f2 = figure('Visible','off','Position',[100 100 600 700]);
    for i = 1:3
        subplot(3,1,i); hold on;
        plot(fused_time, v_f(:,i), 'b', 'DisplayName', sprintf('%s Fused', method));
        plot(fused_time, v_t(:,i), 'r--', 'DisplayName', 'Truth');
        title(sprintf('%s Velocity %s (%s)', method, labels{i}, fr));
        xlabel('Time (s)'); ylabel('Velocity (m/s)'); grid on; legend;
    end
    tightfig();
    out_name = sprintf('%s_%s_%s_task6_fused_velocity_%s', imu_file, gnss_file, method, lower(fr));
    print(f2, fullfile('output_matlab',[out_name '.pdf']), '-dpdf', '-bestfit');
    close(f2);
end

f3 = figure('Visible','off','Position',[100 100 600 700]);
labels_ned = {'North','East','Down'};
for i = 1:3
    subplot(3,1,i); hold on;
    plot(fused_time, error_pos(:,i), 'g', 'DisplayName', sprintf('%s Error', method));
    title(sprintf('%s Position Error %s (NED)', method, labels_ned{i}));
    xlabel('Time (s)'); ylabel('Error (m)'); grid on; legend;
end
tightfig();
out_name = sprintf('%s_%s_%s_task6_position_error_ned', imu_file, gnss_file, method);
print(f3, fullfile('output_matlab',[out_name '.pdf']), '-dpdf', '-bestfit');
close(f3);

fprintf('Task 6: %s final position error %.3f m, RMSEpos %.3f m, RMSEvel %.3f m/s\n', ...
    method, final_err, rmse_pos, rmse_vel);

end

% -------------------------------------------------------------------------
function [pos_ecef, vel_ecef] = ned_to_ecef(pos_ned, vel_ned, lat_rad, lon_rad)
%NED_TO_ECEF Convert NED coordinates to ECEF frame.

slat = sin(lat_rad); clat = cos(lat_rad);
slon = sin(lon_rad); clon = cos(lon_rad);
r_e2n = [-slat*clon, -slat*slon, clat; -slon, clon, 0; -clat*clon, -clat*slon, -slat];
r_n2e = r_e2n.';
pos_ecef = pos_ned * r_n2e;
vel_ecef = vel_ned * r_n2e;
end

% -------------------------------------------------------------------------
function tightfig()
%TIGHTFIG Minimal figure trimming helper.
set(gcf,'PaperPositionMode','auto');
end

