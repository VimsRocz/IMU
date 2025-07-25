function summary = task7_fused_truth_error_analysis(est_file, truth_file, out_dir)
%TASK7_FUSED_TRUTH_ERROR_ANALYSIS  Residual analysis of fused state vs. truth.
%   SUMMARY = TASK7_FUSED_TRUTH_ERROR_ANALYSIS(EST_FILE, TRUTH_FILE, OUT_DIR)
%   loads the fused estimator result and ground truth trajectory (MAT or NPZ
%   files). Residual position, velocity and acceleration are computed in the
%   ECEF frame.  When the estimate only contains NED states, it is converted
%   using the stored reference latitude, longitude and origin.  Plots of the
%   residual components and their norms are saved under OUT_DIR and a
%   structure of summary statistics is returned.

if nargin < 3 || isempty(out_dir)
    out_dir = 'results';
end
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

[t_est, pos_est, vel_est, acc_est] = load_est(est_file);
[t_tru, pos_tru, vel_tru, acc_tru] = load_est(truth_file);

pos_tru_i = interp1(t_tru, pos_tru, t_est, 'linear', 'extrap');
vel_tru_i = interp1(t_tru, vel_tru, t_est, 'linear', 'extrap');
acc_tru_i = interp1(t_tru, acc_tru, t_est, 'linear', 'extrap');

res_pos = pos_est - pos_tru_i;
res_vel = vel_est - vel_tru_i;
res_acc = acc_est - acc_tru_i;

mean_pos = mean(res_pos, 1);
std_pos  = std(res_pos, [], 1);
mean_vel = mean(res_vel, 1);
std_vel  = std(res_vel, [], 1);
mean_acc = mean(res_acc, 1);
std_acc  = std(res_acc, [], 1);

fprintf('Position residual mean [m]: %s\n', mat2str(mean_pos,3));
fprintf('Position residual std  [m]: %s\n', mat2str(std_pos,3));
fprintf('Velocity residual mean [m/s]: %s\n', mat2str(mean_vel,3));
fprintf('Velocity residual std  [m/s]: %s\n', mat2str(std_vel,3));
fprintf('Acceleration residual mean [m/s^2]: %s\n', mat2str(mean_acc,3));
fprintf('Acceleration residual std  [m/s^2]: %s\n', mat2str(std_acc,3));

plot_residuals(t_est, res_pos, res_vel, res_acc, out_dir);
plot_norms(t_est, res_pos, res_vel, res_acc, out_dir);

summary.rmse_pos = sqrt(mean(vecnorm(res_pos,2,2).^2));
summary.final_pos = norm(res_pos(end,:));
summary.rmse_vel = sqrt(mean(vecnorm(res_vel,2,2).^2));
summary.final_vel = norm(res_vel(end,:));
summary.rmse_acc = sqrt(mean(vecnorm(res_acc,2,2).^2));
summary.final_acc = norm(res_acc(end,:));

tbl = {
    'Position [m]',    summary.final_pos, summary.rmse_pos;
    'Velocity [m/s]',  summary.final_vel, summary.rmse_vel;
    'Acceleration [m/s^2]', summary.final_acc, summary.rmse_acc;
    };
hdr = sprintf('%-18s %-12s %-10s', 'Metric', 'Final Error', 'RMSE');
table_lines = cell(size(tbl,1),1);
for ii = 1:size(tbl,1)
    table_lines{ii} = sprintf('%-18s %10.3f %10.3f', tbl{ii,1}, tbl{ii,2}, tbl{ii,3});
end
table_str = strjoin([{hdr}; table_lines], '\n');
table_str = sprintf('%s\n', table_str); % ensure trailing newline
fprintf('%s\n', table_str);

txt_file = fullfile(out_dir,'task7_metrics.txt');
fid = fopen(txt_file,'w');
if fid>0
    fprintf(fid,'%s\n', table_str);
    fclose(fid);
end

[~, tag] = fileparts(out_dir);
parts = strsplit(tag,'_');
method = parts{end};
fprintf('[SUMMARY] method=%s rmse_pos=%.3f m final_pos=%.3f m rmse_vel=%.3f m/s final_vel=%.3f m/s\n', ...
    method, summary.rmse_pos, summary.final_pos, summary.rmse_vel, summary.final_vel);

end

% -------------------------------------------------------------------------
function [t, pos, vel, acc] = load_est(file)
%LOAD_EST Load NPZ or MAT estimate containing ECEF position and velocity.
    f = string(file);
    if endsWith(f,'.npz')
        d = py.numpy.load(f);
        t = double(d{'time_s'});
        pos = double(d{'pos_ecef_m'});
        vel = double(d{'vel_ecef_ms'});
        if isKey(d,'acc_ecef_ms2')
            acc = double(d{'acc_ecef_ms2'});
        else
            acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
        end
    else
        if endsWith(f,'.txt')
            raw = read_state_file(f);
            t = raw(:,1);
            pos = raw(:,2:4);
            vel = raw(:,5:7);
            acc = [zeros(1,3); diff(vel)./diff(t)];
        else
            S = load(f);
            if isfield(S,'time_s');
                t = S.time_s(:);
            elseif isfield(S,'time');
                t = S.time(:);
            elseif isfield(S,'imu_time')
                t = S.imu_time(:);
            else
                t = (0:size(S.pos_ecef,1)-1)';
            end
            if isfield(S,'pos_ecef_m')
                pos = S.pos_ecef_m;
                vel = S.vel_ecef_ms;
            elseif isfield(S,'pos_ecef')
                pos = S.pos_ecef;
                vel = S.vel_ecef;
            elseif isfield(S,'pos_ned') && isfield(S,'vel_ned')
                % Derive ECEF using reference parameters
                if isfield(S,'ref_lat'); lat = S.ref_lat; elseif isfield(S,'ref_lat_rad'); lat = S.ref_lat_rad; else; lat = 0; end
                if isfield(S,'ref_lon'); lon = S.ref_lon; elseif isfield(S,'ref_lon_rad'); lon = S.ref_lon_rad; else; lon = 0; end
                if isfield(S,'ref_r0'); r0 = S.ref_r0; elseif isfield(S,'ref_r0_m'); r0 = S.ref_r0_m; else; r0 = [0 0 0]; end
                C = compute_C_ECEF_to_NED(lat, lon);
                pos = (C' * S.pos_ned')' + r0(:)';
                vel = (C' * S.vel_ned')';
            else
                error('Task7:BadData','Estimate lacks ECEF or NED position fields');
            end
            if isfield(S,'acc_ecef_ms2')
                acc = S.acc_ecef_ms2;
            else
                if exist('vel','var') && numel(t) > 1
                    acc = [zeros(1,3); diff(vel)./diff(t)];
                else
                    acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
                end
            end
        end
    end
end

% -------------------------------------------------------------------------
function plot_residuals(t, res_pos, res_vel, res_acc, out_dir)
%PLOT_RESIDUALS Plot residual components.
    labels = {'X','Y','Z'};
    f = figure('Visible','off','Position',[100 100 900 700]);
    for i = 1:3
        for j = 1:3
            ax = subplot(3,3,(i-1)*3+j); hold on;
            switch i
                case 1; arr = res_pos; ylab = 'Position Residual [m]';
                case 2; arr = res_vel; ylab = 'Velocity Residual [m/s]';
                otherwise; arr = res_acc; ylab = 'Acceleration Residual [m/s^2]';
            end
            plot(t, arr(:,j));
            if i==1; title(labels{j}); end
            if j==1; ylabel(ylab); end
            if i==3; xlabel('Time [s]'); end
            grid on;
        end
    end
    sgtitle('Task 7 Residuals');
    set(f,'PaperPositionMode','auto');
    pdf = fullfile(out_dir,'task7_residuals.pdf');
    print(f,pdf,'-dpdf');
    close(f);
end

% -------------------------------------------------------------------------
function plot_norms(t, res_pos, res_vel, res_acc, out_dir)
%PLOT_NORMS Plot norms of the residual vectors.
    f = figure('Visible','off');
    plot(t, vecnorm(res_pos,2,2), 'DisplayName','|pos|'); hold on;
    plot(t, vecnorm(res_vel,2,2), 'DisplayName','|vel|');
    plot(t, vecnorm(res_acc,2,2), 'DisplayName','|acc|');
    xlabel('Time [s]'); ylabel('Residual Norm'); legend; grid on;
    set(f,'PaperPositionMode','auto');
    pdf = fullfile(out_dir,'task7_residual_norms.pdf');
    print(f,pdf,'-dpdf');
    close(f);
end
