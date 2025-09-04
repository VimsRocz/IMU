function rmse_pos = Task_5_try_once(cfg, vel_q_scale, vel_r)
%TASK_5_TRY_ONCE Call Task_5 once with given Q/R and return RMSE position.
%   RMSE_POS = TASK_5_TRY_ONCE(CFG, VEL_Q_SCALE, VEL_R) runs Task_5 with a
%   limited step count and plotting/console output disabled, returning the scalar RMSE of
%   position error for autotuning.

    if nargin < 3, error('Task_5_try_once:args','cfg, vel_q_scale, vel_r required'); end
    % Use at most first 200k IMU steps for speed during tuning
    max_steps = 200000;
    % Determine total IMU samples to avoid requesting more steps than exist
    fid = fopen(cfg.imu_path, 'r');
    if fid == -1
        error('Task_5_try_once:file', 'Could not open IMU file %s', cfg.imu_path);
    end
    total_samples = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        total_samples = total_samples + 1;
    end
    fclose(fid);
    steps = min(max_steps, total_samples);
    % Clone cfg locally and disable all plotting/saving for tuning runs
    cfg_local = cfg;
    if ~isfield(cfg_local, 'plots') || ~isstruct(cfg_local.plots)
        cfg_local.plots = struct();
    end
    cfg_local.plots.popup_figures = false;
    cfg_local.plots.save_pdf = false;
    cfg_local.plots.save_png = false;
    cfg = cfg_local; %#ok<NASGU> ensure Task_5 sees modified cfg
    try
        % For Task_5, pass the GNSS CSV path as the 2nd argument (MATLAB Task 5 uses GNSS as measurement source)
        res = Task_5(cfg_local.imu_path, cfg_local.gnss_path, cfg_local.method, [], ...
            'vel_q_scale', vel_q_scale, 'vel_r', vel_r, ...
            'trace_first_n', 0, 'max_steps', steps, 'dryrun', true); % dryrun suppresses plots/logging
        if isstruct(res) && isfield(res,'rmse_pos')
            rmse_pos = res.rmse_pos;
        else
            rmse_pos = NaN;
        end
    catch ME
        warning('Task_5_try_once failed (q=%.3f r=%.3f): %s', vel_q_scale, vel_r, ME.message);
        rmse_pos = NaN;
    end
end
