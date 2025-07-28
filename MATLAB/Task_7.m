function Task_7(task5_file, truth_file, tag)
%TASK_7  Residual analysis (Fused vs Truth) and summary metrics.
%   TASK_7(TASK5_FILE, TRUTH_FILE, TAG) loads the fused navigation output
%   from Task 5 and the matching ground truth trajectory. Position and
%   velocity residuals are computed in the NED, ECEF and body frames.
%   Time vectors must match exactly.  Per-frame RMSE and final errors are
%   printed and saved alongside residual plots under ``results/<tag>/``.
%
%   Usage:
%       Task_7('Task5_IMU_X002_GNSS_X002.mat', 'STATE_X001.txt', 'run1')

    if nargin < 3 || isempty(tag)
        tag = 'task7';
    end
    if ~isfile(task5_file)
        error('Task_7:FileNotFound','Task 5 file not found: %s',task5_file);
    end
    if ~isfile(truth_file)
        error('Task_7:TruthNotFound','Truth file not found: %s',truth_file);
    end

    S = load(task5_file);
    truth = read_truth_file(truth_file);

    time = S.time(:);
    if numel(time) ~= numel(truth.time)
        error('Task_7:LengthMismatch','Estimator and truth length differ');
    end

    outdir = fullfile(get_results_dir(), tag);
    if ~exist(outdir,'dir'); mkdir(outdir); end

    % Truth vectors in NED
    pos_truth_ned = truth.pos_ned;
    vel_truth_ned = truth.vel_ned;

    % Reference parameters from estimator
    C_e2n = compute_C_ECEF_to_NED(S.ref_lat, S.ref_lon);
    r0 = S.ref_r0(:);

    [pos_truth_ecef, vel_truth_ecef] = ned2ecef_series(pos_truth_ned, vel_truth_ned, C_e2n, r0);
    [pos_truth_body, vel_truth_body] = ned2body_series(pos_truth_ned, vel_truth_ned, S.C_BN_log);

    % Residuals
    res.pos_ned  = S.pos_fused_ned - pos_truth_ned;
    res.vel_ned  = S.vel_fused_ned - vel_truth_ned;
    res.pos_ecef = S.pos_fused_ecef - pos_truth_ecef;
    res.vel_ecef = S.vel_fused_ecef - vel_truth_ecef;
    res.pos_body = S.pos_fused_body - pos_truth_body;
    res.vel_body = S.vel_fused_body - vel_truth_body;
    res.err_norm_pos = vecnorm_if_missing(res.pos_ned,2,2);
    res.err_norm_vel = vecnorm_if_missing(res.vel_ned,2,2);

    plot_residuals_frame(time, res.pos_ned,  res.vel_ned,  'NED',  tag, outdir);
    plot_residuals_frame(time, res.pos_ecef, res.vel_ecef, 'ECEF', tag, outdir);
    plot_residuals_frame(time, res.pos_body, res.vel_body, 'Body', tag, outdir);
    plot_error_norms(time, res.err_norm_pos, res.err_norm_vel, tag, outdir);

    statsNED  = compute_stats(res.pos_ned,  res.vel_ned);
    statsECEF = compute_stats(res.pos_ecef, res.vel_ecef);
    statsBody = compute_stats(res.pos_body, res.vel_body);

    summary = struct( ...
        'frame',   {'NED';'ECEF';'Body'}, ...
        'rmse_pos',  [statsNED.rmse_pos; statsECEF.rmse_pos; statsBody.rmse_pos], ...
        'final_pos', [statsNED.final_pos; statsECEF.final_pos; statsBody.final_pos], ...
        'rmse_vel',  [statsNED.rmse_vel; statsECEF.rmse_vel; statsBody.rmse_vel], ...
        'final_vel', [statsNED.final_vel; statsECEF.final_vel; statsBody.final_vel] );

    disp(struct2table(summary));
    save(fullfile(outdir,[tag '_task7_summary.mat']),'summary');
    writetable(struct2table(summary), fullfile(outdir,[tag '_task7_summary.csv']));

    fprintf('[SUMMARY] %s rmse_pos=%6.2f m final_pos=%6.2f m rmse_vel=%6.2f m/s final_vel=%6.2f m/s\n', ...
        tag, summary.rmse_pos(1), summary.final_pos(1), summary.rmse_vel(1), summary.final_vel(1));
    fprintf('Task 7 residual plots saved in %s\n', outdir);
end

% -------------------------------------------------------------------------
function [pos_ecef, vel_ecef] = ned2ecef_series(pos_ned, vel_ned, C_e2n, r0)
    pos_ecef = (C_e2n' * pos_ned')' + r0.';
    vel_ecef = (C_e2n' * vel_ned')';
end

% -------------------------------------------------------------------------
function [pos_body, vel_body] = ned2body_series(pos_ned, vel_ned, C_BN_log)
    N = size(pos_ned,1);
    pos_body = zeros(N,3); vel_body = zeros(N,3);
    for k = 1:N
        C_B_N = C_BN_log(:,:,k);
        pos_body(k,:) = (C_B_N' * pos_ned(k,:)')';
        vel_body(k,:) = (C_B_N' * vel_ned(k,:)')';
    end
end

% -------------------------------------------------------------------------
function plot_residuals_frame(t, pos_res, vel_res, frame, tag, outdir)
    labels = {'X','Y','Z'};
    f = figure('Visible','off');
    for j = 1:3
        subplot(2,3,j); plot(t, pos_res(:,j)); hold on; yline(0,'k:');
        title(labels{j}); ylabel('Position Residual [m]'); grid on;
        subplot(2,3,3+j); plot(t, vel_res(:,j)); hold on; yline(0,'k:');
        xlabel('Time [s]'); ylabel('Velocity Residual [m/s]'); grid on;
    end
    sgtitle(sprintf('Task 7 \x2013 Residuals (Fused - Truth) \x2014 %s Frame', frame));
    pdf = fullfile(outdir, sprintf('%s_task7_residuals_%s.pdf', tag, lower(frame)));
    png = fullfile(outdir, sprintf('%s_task7_residuals_%s.png', tag, lower(frame)));
    set(f,'PaperPositionMode','auto');
    print(f,pdf,'-dpdf','-bestfit');
    print(f,png,'-dpng');
    close(f);
end

% -------------------------------------------------------------------------
function plot_error_norms(t, pos_norm, vel_norm, tag, outdir)
    rmse_pos = sqrt(mean(pos_norm.^2));
    rmse_vel = sqrt(mean(vel_norm.^2));
    f = figure('Visible','off');
    subplot(2,1,1); plot(t, pos_norm); hold on; yline(rmse_pos,'r--','RMSE');
    ylabel('|pos error| [m]'); grid on;
    subplot(2,1,2); plot(t, vel_norm); hold on; yline(rmse_vel,'r--','RMSE');
    xlabel('Time [s]'); ylabel('|vel error| [m/s]'); grid on;
    sgtitle('Task 7 Error Norms');
    pdf = fullfile(outdir, sprintf('%s_task7_error_norms.pdf', tag));
    set(f,'PaperPositionMode','auto');
    print(f,pdf,'-dpdf','-bestfit');
    close(f);
end

% -------------------------------------------------------------------------
function stats = compute_stats(pos_res, vel_res)
    stats.rmse_pos  = sqrt(mean(sum(pos_res.^2,2)));
    stats.final_pos = norm(pos_res(end,:));
    stats.rmse_vel  = sqrt(mean(sum(vel_res.^2,2)));
    stats.final_vel = norm(vel_res(end,:));
end

% -------------------------------------------------------------------------
function n = vecnorm_if_missing(x,p,dim)
    if exist('vecnorm','file')
        if nargin < 2; p = 2; end
        if nargin < 3; dim = 2; end
        n = vecnorm(x,p,dim);
    else
        if nargin < 2; p = 2; end
        if nargin < 3; dim = 2; end
        n = sum(abs(x).^p,dim).^(1/p);
    end
end
