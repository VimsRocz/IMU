function Task_3(imu_path, gnss_path, method)
%TASK_3 Solve Wahba's problem and save body->NED rotation matrices.
%   TASK_3(IMU_PATH, GNSS_PATH, METHOD) computes an initial attitude using
%   reference vectors from Task 1 and body-frame measurements from Task 2.
%   Results are stored in the MATLAB-only results directory.
%
%   If Task 1 or Task 2 outputs are missing they are regenerated
%   automatically to keep the pipeline deterministic.

paths = project_paths();
results_dir = paths.matlab_results;

[~, imu_id, ~]  = fileparts(imu_path);
[~, gnss_id, ~] = fileparts(gnss_path);

% File names for Task 1 and Task 2 results
t1 = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method));
t2 = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method));

if ~isfile(t1), Task_1(imu_path, gnss_path, method); end
if ~isfile(t2), Task_2(imu_path, gnss_path, method); end

S1 = load(t1);
S2 = load(t2);
if isfield(S2, 'body_data')
    bd = S2.body_data;
else
    bd = S2;
end

g_ned  = S1.g_NED(:);          % NED +Z is down
w_ned  = S1.omega_NED(:);
g_body = bd.g_body(:);
w_body = bd.omega_ie_body(:);

C_tri = triad(g_ned, w_ned, g_body, w_body);
C_q   = davenport_q_method([g_ned w_ned], [g_body w_body]);
C_svd = svd_wahba([g_ned w_ned], [g_body w_body]);

save(fullfile(results_dir, sprintf('IMU_%s_GNSS_%s_%s_task3_results.mat', imu_id, gnss_id, method)), ...
     'C_tri','C_q','C_svd','g_ned','w_ned','g_body','w_body','-v7');

fprintf('Task 3: attitude matrices saved under %s\n', results_dir);
end
