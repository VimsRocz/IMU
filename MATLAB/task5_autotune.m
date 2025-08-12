function [best_q, best_r, report] = task5_autotune(imu_path, gnss_path, method, grid_q, grid_r, verbose)
%TASK5_AUTOTUNE Grid-search vel_q_scale and vel_r to minimise RMSE_pos.
%   [BEST_Q, BEST_R, REPORT] = TASK5_AUTOTUNE(IMU_PATH, GNSS_PATH, METHOD,
%   GRID_Q, GRID_R) runs Task_5 over the Cartesian product of GRID_Q and
%   GRID_R and selects the pair that minimises RMSE_pos (computed against
%   GNSS). REPORT contains fields {table, best_rmse, best_rmse_q,
%   best_rmse_r}. This function can be slow on large datasets; consider
%   running with a reduced time span if needed.

    if nargin < 4 || isempty(grid_q), grid_q = [5, 10, 20, 40]; end
    if nargin < 5 || isempty(grid_r), grid_r = [0.25, 0.5, 1.0]; end
    if nargin < 6 || isempty(verbose), verbose = false; end

    results = [];
    k = 1;
    best_rmse = inf; best_q = grid_q(1); best_r = grid_r(1);
    % Use a reduced step count during tuning to keep runtime manageable
    max_steps = 120000;  % ~5 minutes of IMU at 400 Hz
    for q = grid_q
        for r = grid_r
            if verbose
                fprintf('[Autotune] Trying vel_q_scale=%.3f  vel_r=%.3f ...\n', q, r);
            end
            try
                res = Task_5(imu_path, gnss_path, method, [], 'vel_q_scale', q, 'vel_r', r, 'trace_first_n', 0, 'max_steps', max_steps);
                rmse = res.rmse_pos;
                results(k).vel_q_scale = q; %#ok<AGROW>
                results(k).vel_r = r;      %#ok<AGROW>
                results(k).rmse_pos = rmse; %#ok<AGROW>
                if rmse < best_rmse
                    best_rmse = rmse; best_q = q; best_r = r;
                end
            catch ME
                warning('[Autotune] Failed for q=%.3f r=%.3f: %s', q, r, ME.message);
                results(k).vel_q_scale = q; results(k).vel_r = r; results(k).rmse_pos = inf; %#ok<AGROW>
            end
            k = k + 1;
        end
    end

    % Build a simple report
    T = struct2table(results);
    report = struct('table', T, 'best_rmse', best_rmse, ...
        'best_rmse_q', best_q, 'best_rmse_r', best_r);
    disp(T);
    fprintf('[Autotune] Best vel_q_scale=%.3f  vel_r=%.3f  RMSE=%.3f\n', ...
        best_q, best_r, best_rmse);
end
