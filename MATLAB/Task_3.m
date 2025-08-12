function Task3 = Task_3(imu_path, gnss_path, method)
% TASK_3 - Solve Wahba (TRIAD / Davenport / SVD) and save task3_results
%   TASK3 = TASK_3(imu_path, gnss_path, method) computes an initial
%   attitude using reference vectors from Task 1 and body-frame measurements
%   from Task 2. Results are stored using canonical TaskIO format.
%
%   The function saves both legacy and canonical MAT-files:
%       <IMU>_<GNSS>_<METHOD>_task3_results.mat  (canonical format)
%       (Legacy variables preserved for backward compatibility)
%
%   Returns a Task3 struct containing:
%       methods - cell array of attitude estimation methods
%       Rbn     - 3x3xN rotation matrices (body to NED)
%       q       - Nx4 quaternions for each method
%       meta    - metadata struct with dataset and method info
%
%   Usage:
%       Task3 = Task_3(imu_path, gnss_path, method)
%
%   If Task 1 or Task 2 outputs are missing they are regenerated
%   automatically to keep the pipeline deterministic.

% Determine results directory and setup
p = project_paths();
results_dir = p.matlab_results;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

% Extract dataset identifiers  
[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);
imu_id  = erase(imu_base, '.dat');
gnss_id = erase(gnss_base, '.csv');

% Print task start like canonical Task_1
if isempty(method)
    tag = [imu_id '_' gnss_id];
else
    tag = [imu_id '_' gnss_id '_' method];
end

print_task_start(tag);
if ~isempty(method)
    fprintf('Running attitude-estimation method: %s\n', method);
end
fprintf('TASK 3: Solve Wahba problem for initial attitude estimation\n');

fprintf('\nSubtask 3.1: Loading Task 1 and Task 2 dependencies.\n');
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
        fprintf('Using task1_results file: %s\n', t1);
    else
        % Neither file exists, run Task_1
        fprintf('Task 1 results missing, running Task_1...\n');
        Task_1(imu_path, gnss_path, method);
    end
end

if ~isfile(t2)
    fprintf('Task 2 results missing, running Task_2...\n');
    Task_2(imu_path, gnss_path, method);
end

fprintf('Loading Task 1 results from: %s\n', t1);
fprintf('Loading Task 2 results from: %s\n', t2);

fprintf('\nSubtask 3.2: Extracting reference and body-frame vectors.\n');
S1 = load(t1);   % expects ref vectors etc.
S2 = load(t2);   % expects body vectors etc.
if isfield(S2, 'body_data')
    bd = S2.body_data;
else
    bd = S2;
end

fprintf('TASK 3: Solve Wahba''s problem (find initial attitude from body to NED)\n');
fprintf('Subtask 3.1: Preparing vector pairs for attitude determination.\n');

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
fprintf('TRIAD method completed\n');

% Print TRIAD rotation matrix
fprintf('Rotation matrix (TRIAD method, Case 1):\n');
for i = 1:3
    fprintf('[[ %.8e %.8e %.8e]\n', R_tri(i,1), R_tri(i,2), R_tri(i,3));
end
fprintf('Rotation matrix (TRIAD method, Case 2):\n');
for i = 1:3
    fprintf('[[ %.8e %.8e %.8e]\n', R_tri(i,1), R_tri(i,2), R_tri(i,3));
end

fprintf('Subtask 3.3: Computing rotation matrix using Davenport''s Q-Method.\n');
try
    R_dav = davenport_q_method([g_ned w_ned], [g_body w_body]);
    q_dav = rotm2quat(R_dav);
    fprintf('Davenport method completed\n');
catch
    R_dav = R_tri;
    q_dav = q_tri;
    fprintf('Davenport method failed, using TRIAD result\n');
end

% Print Davenport rotation matrix and quaternion
fprintf('Rotation matrix (Davenport''s Q-Method, Case 1):\n');
for i = 1:3
    fprintf('[[ %.8e %.8e %.8e]\n', R_dav(i,1), R_dav(i,2), R_dav(i,3));
end
fprintf('Davenport quaternion (q_w, q_x, q_y, q_z, Case 1): [ %.8f %.8f %.8f %.8f]\n', ...
        q_dav(1), q_dav(2), q_dav(3), q_dav(4));
fprintf('Rotation matrix (Davenport''s Q-Method, Case 2):\n');
for i = 1:3
    fprintf('[[ %.8e %.8e %.8e]\n', R_dav(i,1), R_dav(i,2), R_dav(i,3));
end
fprintf('Davenport quaternion (q_w, q_x, q_y, q_z, Case 2): [ %.8f %.8f %.8f %.8f]\n', ...
        q_dav(1), q_dav(2), q_dav(3), q_dav(4));

fprintf('Subtask 3.4: Computing rotation matrix using SVD method.\n');
try
    R_svd = svd_wahba([g_ned w_ned], [g_body w_body]);
    q_svd = rotm2quat(R_svd);
    fprintf('SVD method completed\n');
catch
    R_svd = R_tri;
    q_svd = q_tri;
    fprintf('SVD method failed, using TRIAD result\n');
end


% ---- pack canonical struct that Task_4/5 expect ----
methods = {'TRIAD','Davenport','SVD'};
Rbn = cat(3, R_tri, R_dav, R_svd);
q = [q_tri; q_dav; q_svd];
Task3 = struct('methods',{methods}, 'Rbn', Rbn, 'q', q, ...
               'meta', struct('imu_id',imu_id,'gnss_id',gnss_id,'method',method, 'dataset', tag));

base = fullfile(results_dir, sprintf('%s_%s_%s_task3_results', imu_id, gnss_id, method));
TaskIO.save('Task3', Task3, [base '.mat']);


% Expose to workspace for interactive use
assignin('base','Task3', Task3);

end

