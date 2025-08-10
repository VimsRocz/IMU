%CHECK_LATEST_PYTHON_MATLAB  Compare the latest MATLAB and Python results
%   CHECK_LATEST_PYTHON_MATLAB searches the repository results directory for
%   the newest *_kf_output.mat file and loads the matching *_kf_output.npz
%   produced by the Python pipeline. The script interpolates the MATLAB
%   position and velocity onto the Python time base when necessary,
%   computes RMSE metrics, and prints a short summary.
%
%   This helper is intended for quick parity checks and is used by
%   RUN_ALL_DATASETS_MATLAB after each dataset is processed.
%
%   See also RUN_ALL_DATASETS_MATLAB, COMPARE_PYTHON_MATLAB_RESULTS

results_dir = get_results_dir();
mat_list = dir(fullfile(results_dir, '*_kf_output.mat'));
if isempty(mat_list)
    fprintf('No MATLAB results found in %s\n', results_dir);
    return;
end
[~, idx] = max([mat_list.datenum]);
mat_file = fullfile(results_dir, mat_list(idx).name);
[~, base_name, ~] = fileparts(mat_file);
npz_file = fullfile(results_dir, [base_name '.npz']);
if ~exist(npz_file, 'file')
    fprintf('Python NPZ not found for %s\n', base_name);
    return;
end

mat = load(mat_file);
% Extract MATLAB position and velocity
if isfield(mat, 'pos_ned')
    pos_mat = mat.pos_ned;
elseif isfield(mat, 'pos')
    pos_mat = mat.pos;
elseif isfield(mat, 'x_log')
    pos_mat = mat.x_log(1:3, :)';
else
    error('MATLAB result missing position data');
end
if isfield(mat, 'vel_ned')
    vel_mat = mat.vel_ned;
elseif isfield(mat, 'vel')
    vel_mat = mat.vel;
elseif isfield(mat, 'x_log')
    vel_mat = mat.x_log(4:6, :)';
else
    error('MATLAB result missing velocity data');
end
if isfield(mat, 'time')
    t_mat = mat.time;
else
    t_mat = (0:size(pos_mat,1)-1)';
end

% Load Python NPZ
data = py.numpy.load(npz_file, pyargs('allow_pickle', true));
keys = cell(data.keys());
S = struct();
for k = 1:numel(keys)
    S.(keys{k}) = double(data{keys{k}});
end
if isfield(S, 'fused_pos')
    pos_py = S.fused_pos;
elseif isfield(S, 'pos')
    pos_py = S.pos;
elseif isfield(S, 'pos_ned')
    pos_py = S.pos_ned;
else
    error('Python result missing position data');
end
if isfield(S, 'fused_vel')
    vel_py = S.fused_vel;
elseif isfield(S, 'vel')
    vel_py = S.vel;
elseif isfield(S, 'vel_ned')
    vel_py = S.vel_ned;
else
    error('Python result missing velocity data');
end
if isfield(S, 'time_residuals')
    t_py = S.time_residuals;
elseif isfield(S, 'time')
    t_py = S.time;
else
    t_py = (0:size(pos_py,1)-1)';
end

% Interpolate MATLAB results onto Python time base
pos_mat_i = interp1(t_mat, pos_mat, t_py, 'linear', 'extrap');
vel_mat_i = interp1(t_mat, vel_mat, t_py, 'linear', 'extrap');

pos_diff = pos_py - pos_mat_i;
vel_diff = vel_py - vel_mat_i;

rmse_pos = sqrt(mean(sum(pos_diff.^2, 2)));
rmse_vel = sqrt(mean(sum(vel_diff.^2, 2)));
final_pos = norm(pos_diff(end, :));
final_vel = norm(vel_diff(end, :));

fprintf('Latest MATLAB vs Python comparison for %s\n', base_name);
fprintf('RMSE position difference: %.4f m\n', rmse_pos);
fprintf('RMSE velocity difference: %.4f m/s\n', rmse_vel);
fprintf('Final position difference: %.4f m\n', final_pos);
fprintf('Final velocity difference: %.4f m/s\n', final_vel);

