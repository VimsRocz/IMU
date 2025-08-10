function [best_q, best_r, report] = task5_autotune(imu_path, gnss_path, method, grid_q, grid_r)
%TASK5_AUTOTUNE Minimal stub to silence autotune dependency.
%   Returns the first provided candidates and a report echoing defaults
%   consistent with the Python pipeline's typical values.
if nargin < 4 || isempty(grid_q), grid_q = 10.0; end
if nargin < 5 || isempty(grid_r), grid_r = 0.25; end
best_q = grid_q(1);
best_r = grid_r(1);
report = struct('best_rmse', NaN, 'best_rmse_q', best_q, 'best_rmse_r', best_r);
end

