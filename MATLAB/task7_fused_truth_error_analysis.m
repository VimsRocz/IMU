function summary = task7_fused_truth_error_analysis(est_file, truth_file, out_dir)
%TASK7_FUSED_TRUTH_ERROR_ANALYSIS  Residual analysis of fused state vs. truth.
%   SUMMARY = TASK7_FUSED_TRUTH_ERROR_ANALYSIS(EST_FILE, TRUTH_FILE, OUT_DIR)
%   loads the fused estimator result and ground truth trajectory (MAT or NPZ
%   files). Both trajectories are aligned in the local NED frame before
%   computing residuals. The truth ECEF data is converted using
%   ``compute_C_ECEF_to_NED`` with the estimator reference latitude,
%   longitude and origin. When loading NPZ files the function accepts either
%   ``pos_ecef_m``/``vel_ecef_ms`` or ``pos_ecef``/``vel_ecef`` keys.  When the
%   estimate only contains NED states, it is converted to ECEF and then to
%   the common NED frame. Plots of the residual components and their norms
%   are saved under OUT_DIR and a structure of summary statistics is returned.

if nargin < 3 || isempty(out_dir)
    out_dir = get_results_dir();
end
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
start_time = tic;

[t_est, pos_est_ecef, vel_est_ecef, acc_est_ecef, lat, lon, r0] = load_est(est_file);
[t_tru, pos_tru_ecef, vel_tru_ecef, acc_tru_ecef] = load_est(truth_file);

% ------------------------------------------------------------------
% The estimator may use absolute UNIX time stamps while the truth data
% typically starts from zero.  To ensure consistent interpolation we
% normalise both time vectors so that ``t=0`` corresponds to the first
% sample.  This mirrors ``load_estimate(..., times=t_truth)`` in the
% Python implementation.
% ------------------------------------------------------------------
t_est = t_est - t_est(1);
t_tru = t_tru - t_tru(1);

% ------------------------------------------------------------------
% Convert both trajectories to a common NED frame using the estimator
% reference parameters.  When the estimator file lacks reference
% information, fall back to the first truth sample.
% ------------------------------------------------------------------
if isnan(lat) || isnan(lon) || any(isnan(r0))
    % Use the first truth sample as the shared origin when the estimator
    % does not provide reference parameters. ``ecef_to_geodetic`` returns
    % degrees so convert to radians for ``compute_C_ECEF_to_NED``.
    r0 = pos_tru_ecef(1,:);
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(r0(1), r0(2), r0(3));
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);
else
    % Some log files store reference lat/lon in degrees.  Detect this and
    % convert to radians so both MATLAB and Python interpret them identically.
    if abs(lat) > pi
        lat = deg2rad(lat);
    end
    if abs(lon) > pi
        lon = deg2rad(lon);
    end
end
C = compute_C_ECEF_to_NED(lat, lon);
% Rotate both trajectories into the same NED frame using the **shared**
% origin ``r0`` so that residuals are computed consistently.  This
% matches the behaviour of ``validate_with_truth.py`` in Python.
pos_est = (C * (pos_est_ecef' - r0)).';
vel_est = (C*vel_est_ecef.').';
acc_est = (C*acc_est_ecef.').';
pos_tru = (C * (pos_tru_ecef' - r0)).';
vel_tru = (C*vel_tru_ecef.').';
acc_tru = (C*acc_tru_ecef.').';

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

plot_residuals(t_est, res_pos, res_vel, out_dir);
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
runtime = toc(start_time);
fprintf('[SUMMARY] method=%s rmse_pos=%.3fm final_pos=%.3fm rmse_vel=%.3fm/s final_vel=%.3fm/s runtime=%.2fs\n', ...
    method, summary.rmse_pos, summary.final_pos, summary.rmse_vel, summary.final_vel, runtime);

end

% -------------------------------------------------------------------------
function [t, pos, vel, acc, lat, lon, r0] = load_est(file)
%LOAD_EST Load NPZ or MAT estimate containing ECEF position and velocity.
    f = string(file);
    lat = NaN; lon = NaN; r0 = [NaN NaN NaN];
    if endsWith(f,'.npz')
    d = py.numpy.load(f);
    t = double(d{'time_s'});
    if isKey(d, 'pos_ecef_m')
        pos = double(d{'pos_ecef_m'});
    elseif isKey(d, 'pos_ecef')
        pos = double(d{'pos_ecef'});
    else
        error('Task7:BadData', 'NPZ file lacks ECEF position');
    end
    if isKey(d, 'vel_ecef_ms')
        vel = double(d{'vel_ecef_ms'});
    elseif isKey(d, 'vel_ecef')
        vel = double(d{'vel_ecef'});
    else
        error('Task7:BadData', 'NPZ file lacks ECEF velocity');
    end
    if isKey(d,'acc_ecef_ms2')
        acc = double(d{'acc_ecef_ms2'});
    else
        acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
    end
    if isKey(d,'ref_lat_rad'); lat = double(d{'ref_lat_rad'}); elseif isKey(d,'ref_lat'); lat = double(d{'ref_lat'}); end
    if isKey(d,'ref_lon_rad'); lon = double(d{'ref_lon_rad'}); elseif isKey(d,'ref_lon'); lon = double(d{'ref_lon'}); end
    if isKey(d,'ref_r0_m'); r0 = double(d{'ref_r0_m'}); elseif isKey(d,'ref_r0'); r0 = double(d{'ref_r0'}); end
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
                if isfield(S,'ref_lat'); lat = S.ref_lat; elseif isfield(S,'ref_lat_rad'); lat = S.ref_lat_rad; else; lat = NaN; end
                if isfield(S,'ref_lon'); lon = S.ref_lon; elseif isfield(S,'ref_lon_rad'); lon = S.ref_lon_rad; else; lon = NaN; end
                if isfield(S,'ref_r0'); r0 = S.ref_r0; elseif isfield(S,'ref_r0_m'); r0 = S.ref_r0_m; else; r0 = [NaN NaN NaN]; end
                Ctmp = compute_C_ECEF_to_NED(lat, lon);
                pos = (Ctmp' * S.pos_ned')' + r0(:)';
                vel = (Ctmp' * S.vel_ned')';
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
function plot_residuals(t, res_pos, res_vel, out_dir)
%PLOT_RESIDUALS Plot position and velocity residuals in NED.
    labels = {'North','East','Down'};
    f = figure('Visible','off','Position',[100 100 900 450]);
    for j = 1:3
        subplot(2,3,j); plot(t, res_pos(:,j));
        title(labels{j}); ylabel('Pos Residual [m]'); grid on;
        subplot(2,3,3+j); plot(t, res_vel(:,j));
        xlabel('Time [s]'); ylabel('Vel Residual [m/s]'); grid on;
    end
    sgtitle('Task 7 - GNSS - Predicted Residuals');
    set(f,'PaperPositionMode','auto');
    pdf = fullfile(out_dir,'task7_3_residuals_position_velocity.pdf');
    print(f,pdf,'-dpdf','-bestfit');
    close(f);
end

% -------------------------------------------------------------------------
function plot_norms(t, res_pos, res_vel, res_acc, out_dir)
%PLOT_NORMS Plot norms of the residual vectors.
    f = figure('Visible','off');
    plot(t, vecnorm(res_pos,2,2), 'DisplayName','|pos error|'); hold on;
    plot(t, vecnorm(res_vel,2,2), 'DisplayName','|vel error|');
    plot(t, vecnorm(res_acc,2,2), 'DisplayName','|acc error|');
    xlabel('Time [s]'); ylabel('Error Norm'); legend; grid on;
    set(f,'PaperPositionMode','auto');
    pdf = fullfile(out_dir,'task7_3_error_norms.pdf');
    print(f,pdf,'-dpdf','-bestfit');
    close(f);
end
