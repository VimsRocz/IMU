function compare_python_matlab_results(imu_file, gnss_file, method)
%COMPARE_PYTHON_MATLAB_RESULTS  Compare MATLAB and Python fusion outputs.
%   COMPARE_PYTHON_MATLAB_RESULTS(IMU_FILE, GNSS_FILE, METHOD) loads the
%   <IMU>_<GNSS>_<METHOD>_kf_output.mat files produced by both the
%   Python and MATLAB pipelines and computes the RMSE and final position
%   difference between them. The function prints the metrics in the same
%   style as ``src/compare_python_matlab.py`` for parity checking.
%
%   Example:
%       compare_python_matlab_results('IMU_X001.dat','GNSS_X001.csv','TRIAD')
%
%   Results are expected under the repository's ``results`` directory.
%
%   See also: compare_python_matlab.py

if nargin < 1 || isempty(imu_file);  imu_file  = 'IMU_X001.dat';  end
if nargin < 2 || isempty(gnss_file); gnss_file = 'GNSS_X001.csv'; end
if nargin < 3 || isempty(method);    method    = 'TRIAD';        end

imu_stem  = erase(imu_file, '.dat');
gnss_stem = erase(gnss_file, '.csv');
res_dir   = 'MATLAB/results';
py_file   = fullfile(res_dir, sprintf('%s_%s_%s_kf_output.mat', ...
    imu_stem, gnss_stem, method));
mat_file  = fullfile(res_dir, sprintf('%s_%s_%s_task5_results.mat', ...
    imu_stem, gnss_stem, method));

if ~isfile(py_file)
    error('Python output not found: %s', py_file);
end
if ~isfile(mat_file)
    error('MATLAB output not found: %s', mat_file);
end

py = load(py_file);
mat = load(mat_file);

% Extract common fields
if isfield(py, 'time');          t_py  = py.time; else; t_py = (0:size(py.pos_ned,1)-1)'; end
if isfield(mat, 'time');         t_mat = mat.time; else; t_mat = (0:size(mat.pos_ned,1)-1)'; end

pos_py = py.pos_ned;
vel_py = py.vel_ned;

if isfield(mat, 'pos_ned');
    pos_mat = mat.pos_ned;
elseif isfield(mat, 'x_log');
    pos_mat = mat.x_log(1:3,:)';
else
    error('MATLAB result missing position');
end

if isfield(mat, 'vel_ned');
    vel_mat = mat.vel_ned;
elseif isfield(mat, 'x_log');
    vel_mat = mat.x_log(4:6,:)';
else
    error('MATLAB result missing velocity');
end

% Interpolate MATLAB data onto Python time base
pos_mat_i = interp1(t_mat, pos_mat, t_py, 'linear', 'extrap');
vel_mat_i = interp1(t_mat, vel_mat, t_py, 'linear', 'extrap');

pos_diff = pos_py - pos_mat_i;
vel_diff = vel_py - vel_mat_i;

rmse_pos = sqrt(mean(sum(pos_diff.^2,2)));
rmse_vel = sqrt(mean(sum(vel_diff.^2,2)));
final_pos = norm(pos_diff(end,:));
final_vel = norm(vel_diff(end,:));

fprintf('Python vs MATLAB comparison for %s %s %s\n', imu_file, gnss_file, method);
fprintf('RMSE position difference: %.4f m\n', rmse_pos);
fprintf('RMSE velocity difference: %.4f m/s\n', rmse_vel);
fprintf('Final position difference: %.4f m\n', final_pos);
fprintf('Final velocity difference: %.4f m/s\n', final_vel);
end
