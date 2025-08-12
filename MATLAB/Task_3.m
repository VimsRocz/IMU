function task3_results = Task_3(imu_path, gnss_path, method)
% TASK_3 - Solve Wahba (TRIAD / Davenport / SVD) and save task3_results
%   task3_results = Task_3(imu_path, gnss_path, method) computes an initial
%   attitude using reference vectors from Task 1 and body-frame measurements
%   from Task 2.  Results are stored in the MATLAB-only results directory.
%
%   Usage:
%       task3_results = Task_3(imu_path, gnss_path, method)
%
%   If Task 1 or Task 2 outputs are missing they are regenerated
%   automatically to keep the pipeline deterministic.

% paths
p = project_paths();                      % has fields: root, matlab_results, etc.
results_dir = p.matlab_results;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

% ids
[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);
imu_id  = erase(imu_base, '.dat');
gnss_id = erase(gnss_base, '.csv');

% ensure Task1/Task2 outputs exist (run them if missing)
t1 = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method));
t2 = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method));

% Check for Task1 file - try Task1_init format first, then fall back to task1_results format
if ~isfile(t1)
    % Try alternative task1_results format
    dataset_name = sprintf('%s_%s', imu_id, gnss_id);
    t1_alt = fullfile(results_dir, sprintf('%s_%s_task1_results.mat', dataset_name, method));
    if isfile(t1_alt)
        t1 = t1_alt;
        fprintf('Task 3: Using task1_results file: %s\n', t1);
    else
        % Neither file exists, run Task_1
        Task_1(imu_path, gnss_path, method);
    end
end

if ~isfile(t2), Task_2(imu_path, gnss_path, method); end

S1 = load(t1);   % expects ref vectors etc.
S2 = load(t2);   % expects body vectors etc.
if isfield(S2, 'body_data')
    bd = S2.body_data;
else
    bd = S2;
end

% ---- compute DCMs (use existing implementations) ----
% Handle different field names for gravity and omega vectors
if isfield(S1, 'g_NED')
    g_ned = S1.g_NED(:);          % reference gravity in NED
elseif isfield(S1, 'Task1') && isfield(S1.Task1, 'gravity_ned')
    g_ned = S1.Task1.gravity_ned(:);
else
    error('Task_3: No gravity vector found in Task1 file');
end

if isfield(S1, 'omega_NED')
    w_ned = S1.omega_NED(:);      % earth rotation in NED
elseif isfield(S1, 'Task1') && isfield(S1.Task1, 'omega_ie_ned')
    w_ned = S1.Task1.omega_ie_ned(:);
else
    error('Task_3: No omega vector found in Task1 file');
end
g_body = bd.g_body(:);         % measured gravity in body
w_body = bd.omega_ie_body(:);  % measured earth rotation in body

R_tri = triad(g_ned, w_ned, g_body, w_body);
q_tri = rotm2quat(R_tri);

try
    R_dav = davenport_q_method([g_ned w_ned], [g_body w_body]);
    q_dav = rotm2quat(R_dav);
catch
    R_dav = R_tri;
    q_dav = q_tri;
end

try
    R_svd = svd_wahba([g_ned w_ned], [g_body w_body]);
    q_svd = rotm2quat(R_svd);
catch
    R_svd = R_tri;
    q_svd = q_tri;
end

% ---- pack canonical struct that Task_4/5 expect ----
methods = {'TRIAD','Davenport','SVD'};
Rbn = cat(3, R_tri, R_dav, R_svd);
q = [q_tri; q_dav; q_svd];
Task3 = struct('methods',{methods}, 'Rbn', Rbn, 'q', q, ...
               'meta', struct('imu_id',imu_id,'gnss_id',gnss_id,'method',method));

base = fullfile(results_dir, sprintf('%s_%s_%s_task3_results', imu_id, gnss_id, method));
TaskIO.save('Task3', Task3, [base '.mat']);
fprintf('Task 3: saved Task3 struct -> %s.mat\n', base);

% Expose in base workspace
assignin('base','Task3', Task3);
end

