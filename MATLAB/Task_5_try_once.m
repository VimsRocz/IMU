function rmse_pos = Task_5_try_once(cfg, vel_q_scale, vel_r)
%TASK_5_TRY_ONCE Call Task_5 once with given Q/R and return RMSE position.
%   RMSE_POS = TASK_5_TRY_ONCE(CFG, VEL_Q_SCALE, VEL_R) runs Task_5 with a
%   limited step count and plotting/printing disabled, returning the scalar RMSE of
%   position error for autotuning.

    if nargin < 3, error('Task_5_try_once:args','cfg, vel_q_scale, vel_r required'); end
    % Use at most first 200k IMU steps for speed during tuning
    max_steps = 200000;
    % Determine total IMU samples to avoid requesting more steps than exist
    total_samples = size(readmatrix(cfg.imu_path), 1);
    steps = min(max_steps, total_samples);
    try
        res = Task_5(cfg.imu_path, cfg.gnss_path, cfg.method, [], ...
            'vel_q_scale', vel_q_scale, 'vel_r', vel_r, ...
            'trace_first_n', 0, 'max_steps', steps, 'dryrun', true);
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

