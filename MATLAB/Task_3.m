
function task3_results = Task_3(imu_path, gnss_path, method)
% TASK 3: Solve Wahba's Problem
% This function estimates the initial body-to-NED attitude using several
% approaches (TRIAD, Davenport, SVD). It loads the reference and measured
% vectors saved by Tasks 1 and 2 and stores the resulting rotation
% matrices for later tasks.
%
% Usage:
%   Task_3(imu_path, gnss_path, method)

if nargin < 1 || isempty(imu_path)
    error('IMU file not specified');
end
if nargin < 2 || isempty(gnss_path)
    error('GNSS file not specified');
end
if nargin < 3
    method = ''; %#ok<NASGU>  % unused but kept for API compatibility
end

if ~isfile(gnss_path)
    error('Task_3:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_3:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
pair_tag = [imu_name '_' gnss_name];
if isempty(method)
    tag = pair_tag;
    method_tag = 'AllMethods';
else
    tag = [pair_tag '_' method];
    method_tag = method;
end
results_dir = get_results_dir();
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

% Load vectors produced by Task 1 and Task 2
task1_file = fullfile(results_dir, ['Task1_init_' tag '.mat']);
task2_file = fullfile(results_dir, ['Task2_body_' tag '.mat']);
if evalin('base','exist(''task1_results'',''var'')')
    init_data = evalin('base','task1_results');
else
    if ~isfile(task1_file)
        error('Task_3:MissingFile', 'Missing Task 1 output: %s', task1_file);
    end
    init_data = load(task1_file);
end
if evalin('base','exist(''task2_results'',''var'')')
    body_data = evalin('base','task2_results');
else
    if ~isfile(task2_file)
        error('Task_3:MissingFile', 'Missing Task 2 output: %s', task2_file);
    end
    S = load(task2_file);
    if isfield(S, 'body_data')
        body_data = S.body_data;
    else
        body_data = S; % fallback for legacy files
    end
end

g_NED = init_data.g_NED;
if isfield(init_data, 'omega_NED')
    omega_ie_NED = init_data.omega_NED;
elseif isfield(init_data, 'omega_ie_NED')
    omega_ie_NED = init_data.omega_ie_NED;
else
    error('Task_3:MissingField', 'Task 1 data missing omega_ie_NED field');
end
if isfield(init_data, 'lat')
    lat = deg2rad(init_data.lat);
elseif isfield(init_data, 'lat_deg')
    lat = deg2rad(init_data.lat_deg);
else
    error('Task_3:MissingField', 'Task 1 data missing latitude field');
end
if isfield(body_data,'g_body_scaled')
    g_body = body_data.g_body_scaled;
else
    g_body = body_data.g_body;
end
omega_ie_body = body_data.omega_ie_body;
if isfield(body_data,'accel_bias')
    accel_bias = body_data.accel_bias;
elseif isfield(body_data,'acc_bias')
    accel_bias = body_data.acc_bias; % backward compatibility
else
    accel_bias = zeros(3,1);
end
if isfield(body_data,'gyro_bias'); gyro_bias = body_data.gyro_bias; else; gyro_bias = zeros(3,1); end

omega_E = constants.EARTH_RATE; % Earth rotation rate [rad/s]

if isempty(method)
    log_tag = '';
else
    log_tag = [' (' method ')'];
end
fprintf('\nTASK 3%s: Solve Wahba\x2019s problem (find initial attitude from body to NED)\n', log_tag);

%% ========================================================================
% Subtask 3.1: Prepare Vector Pairs for Attitude Determination
% =========================================================================
fprintf('\nSubtask 3.1: Preparing vector pairs for attitude determination.\n');
% Case 1: Current implementation vectors
v1_B = g_body / norm(g_body);
if norm(omega_ie_body) > 1e-10
    v2_B = omega_ie_body / norm(omega_ie_body);
else
    v2_B = [1.0; 0.0; 0.0];
end
v1_N = g_NED / norm(g_NED);
v2_N = omega_ie_NED / norm(omega_ie_NED);

% Case 2: Recompute omega_ie,NED using document equation
omega_ie_NED_doc = omega_E * [cos(lat); 0.0; -sin(lat)];
v2_N_doc = omega_ie_NED_doc / norm(omega_ie_NED_doc);
fprintf('-> Case 1 and Case 2 vectors prepared.\n');


%% ========================================================================
% Subtask 3.2: TRIAD Method
% =========================================================================
fprintf('\nSubtask 3.2: Computing rotation matrix using TRIAD method.\n');
% Case 1
M_body = triad_basis(v1_B, v2_B);
M_ned_1 = triad_basis(v1_N, v2_N);
R_tri = M_ned_1 * M_body';
[U,~,V] = svd(R_tri);
R_tri = U*V';
fprintf('Rotation matrix (TRIAD method, Case 1):\n');
disp(R_tri);
fig_tri = figure; plot(R_tri(:)); title('TRIAD Rotation Matrix'); xlabel('Element'); ylabel('Value'); grid on;
save_plot(fig_tri, imu_name, gnss_name, method, 3);
expected_C_b_n = [0.23364698, -0.04540352, 0.971260835; ...
                   0.0106220955, 0.998968728, 0.0441435243; ...
                  -0.972263472, 2.82418914e-06, 0.233888307];
triad_err = norm(R_tri - expected_C_b_n);
tol = 1e-3;  % allow small numerical differences
if triad_err > tol
    warning('TRIAD matrix differs from expected by %.3e (> %.1e)', ...
            triad_err, tol);
end

% Case 2
M_ned_2 = triad_basis(v1_N, v2_N_doc);
R_tri_doc = M_ned_2 * M_body';
[U,~,V] = svd(R_tri_doc);
R_tri_doc = U*V';
fprintf('Rotation matrix (TRIAD method, Case 2):\n');
disp(R_tri_doc);


%% ========================================================================
% Subtask 3.3: Davenport’s Q-Method
% =========================================================================
fprintf('\nSubtask 3.3: Computing rotation matrix using Davenport’s Q-Method.\n');
% Case 1
[R_dav, q_dav] = davenport_q_method(v1_B, v2_B, v1_N, v2_N);
fprintf('Rotation matrix (Davenport’s Q-Method, Case 1):\n');
disp(R_dav);
fprintf('Davenport quaternion (Case 1): [%.6f, %.6f, %.6f, %.6f]\n', q_dav(1), q_dav(2), q_dav(3), q_dav(4));

% Case 2
[R_dav_doc, q_dav_doc] = davenport_q_method(v1_B, v2_B, v1_N, v2_N_doc);
fprintf('Rotation matrix (Davenport’s Q-Method, Case 2):\n');
disp(R_dav_doc);
fprintf('Davenport quaternion (Case 2): [%.6f, %.6f, %.6f, %.6f]\n', q_dav_doc(1), q_dav_doc(2), q_dav_doc(3), q_dav_doc(4));


%% ========================================================================
% Subtask 3.4: SVD Method
% =========================================================================
fprintf('\nSubtask 3.4: Computing rotation matrix using SVD method.\n');
R_svd = svd_alignment({g_body, omega_ie_body}, {g_NED, omega_ie_NED});
R_svd_doc = R_svd; % In the python script, SVD method is not re-run for Case 2
fprintf('Rotation matrix (SVD method):\n');
disp(R_svd);


%% ========================================================================
% Subtask 3.5: Convert TRIAD and SVD DCMs to Quaternions
% =========================================================================
fprintf('\nSubtask 3.5: Converting TRIAD and SVD DCMs to quaternions.\n');
q_tri = rot_to_quaternion(R_tri);
q_svd = rot_to_quaternion(R_svd);
q_tri_doc = rot_to_quaternion(R_tri_doc);
q_svd_doc = rot_to_quaternion(R_svd_doc);
fprintf('Quaternion (TRIAD, Case 1): [%.6f, %.6f, %.6f, %.6f]\n', q_tri);
fprintf('Quaternion (SVD, Case 1):   [%.6f, %.6f, %.6f, %.6f]\n', q_svd);
fprintf('Quaternion (TRIAD, Case 2): [%.6f, %.6f, %.6f, %.6f]\n', q_tri_doc);
fprintf('Quaternion (SVD, Case 2):   [%.6f, %.6f, %.6f, %.6f]\n', q_svd_doc);

% Display roll/pitch/yaw for TRIAD case 1 (body->NED)
eul_tri = rad2deg(quat_to_euler(q_tri));
fprintf('TRIAD initial attitude (deg): roll=%.3f pitch=%.3f yaw=%.3f\n', ...
        eul_tri(1), eul_tri(2), eul_tri(3));


%% ========================================================================
% Subtask 3.6: Validate Attitude Determination and Compare Methods
% =========================================================================
fprintf('\nSubtask 3.6: Validating attitude determination and comparing methods.\n');
methods = {"TRIAD", "Davenport", "SVD"};
rot_matrices = {R_tri, R_dav, R_svd};
grav_errors = zeros(1, 3);
omega_errors = zeros(1, 3);

fprintf('\nAttitude errors using reference vectors (Case 1):\n');
for i = 1:length(methods)
    [g_err, o_err] = compute_wahba_errors(rot_matrices{i}, g_body, omega_ie_body, g_NED, omega_ie_NED);
    grav_errors(i) = g_err;
    omega_errors(i) = o_err;
    fprintf('%-10s -> Gravity error (deg): %.6f\n', methods{i}, g_err);
    fprintf('%-10s -> Earth rate error (deg):  %.6f\n', methods{i}, o_err);
end

fprintf('\nDetailed Earth-Rate Errors:\n');
for i = 1:length(methods)
    fprintf('  %-10s: %.6f\xB0\n', methods{i}, omega_errors(i));
end
diff_err = max(omega_errors) - min(omega_errors);
tol = 1e-5; % tolerance in degrees
fprintf('\nEarth-rate errors by method:\n');
for i = 1:length(methods)
    fprintf('  %-10s: %.9f\xB0\n', methods{i}, omega_errors(i));
end
fprintf('  \x394 = %.2e\xB0 (tolerance = %.1e)\n', diff_err, tol);
if diff_err < tol
    warning('All Earth-rate errors are very close; differences are within %.1e\xB0', tol);
end

fprintf('\n==== Method Comparison for Case 1 ====\n');
fprintf('%-10s  %-18s  %-22s\n', 'Method', 'Gravity Err (deg)', 'Earth-Rate Err (deg)');
for i = 1:length(methods)
    fprintf('%-10s  %18.4f  %22.4f\n', methods{i}, grav_errors(i), omega_errors(i));
end

%% ========================================================================
% Subtask 3.7: Plot Validation Errors and Quaternion Components
% =========================================================================
fprintf('\nSubtask 3.7: Plotting validation errors and quaternion components.\n');
% Plot 1: Errors for Case 1
figure('Name', 'Attitude Initialization Error Comparison', 'Position', [100, 100, 800, 400]);
subplot(1, 2, 1);
bar(grav_errors);
set(gca, 'xticklabel', methods);
title('Gravity Vector Error'); ylabel('Error (degrees)'); grid on;
subplot(1, 2, 2);
bar(omega_errors);
set(gca, 'xticklabel', methods);
title('Earth Rate Vector Error'); ylabel('Error (degrees)'); grid on;
sgtitle('Attitude Initialization Method Errors (Case 1)');
err_file = fullfile(results_dir, sprintf('%s_%s_%s_Task3_ErrorComparison.pdf', imu_name, gnss_name, method_tag));
set(gcf, 'PaperPositionMode','auto');
print(gcf, err_file, '-dpdf', '-bestfit');
fprintf('Saved plot: %s\n', err_file);

% Plot 2: Quaternion components for both cases
figure('Name', 'Quaternion Component Comparison', 'Position', [100, 600, 1000, 600]);
quats_c1 = [q_tri, q_dav, q_svd];
quats_c2 = [q_tri_doc, q_dav_doc, q_svd_doc];
all_quats = [quats_c1, quats_c2];
labels = {'TRIAD (C1)', 'Davenport (C1)', 'SVD (C1)', 'TRIAD (C2)', 'Davenport (C2)', 'SVD (C2)'};
b = bar(all_quats');
ylabel('Component Value');
title('Quaternion Components for Each Method and Case');
set(gca, 'xticklabel', labels);
xtickangle(45);
legend('q_w (scalar)', 'q_x', 'q_y', 'q_z');
grid on;
quat_file = fullfile(results_dir, sprintf('%s_%s_%s_Task3_QuaternionComparison.pdf', imu_name, gnss_name, method_tag));
set(gcf, 'PaperPositionMode','auto');
print(gcf, quat_file, '-dpdf', '-bestfit');
fprintf('Saved plot: %s\n', quat_file);


%% ========================================================================
% Subtask 3.8: Store Rotation Matrices for Later Tasks
% =========================================================================
fprintf('\nSubtask 3.8: Storing rotation matrices for use in later tasks.\n');
task3_results = struct();
task3_results.TRIAD.R = R_tri;
task3_results.Davenport.R = R_dav;
task3_results.SVD.R = R_svd;
all_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
save(all_file, 'task3_results');
fprintf('-> Task 3 results (all methods) saved to %s\n', all_file);

% Also save a method-specific copy for later tasks using helper
method_results = task3_results.(method_tag);
save_task_results(method_results, imu_name, gnss_name, method_tag, 3);

% Return and store in base workspace
assignin('base', 'task3_results', task3_results);

% -------------------------------------------------------------
% Display stored rotation matrix for verification
task3_file = fullfile(results_dir, 'Task3_results_IMU_X002_GNSS_X002.mat');
if exist(task3_file, 'file')
    data = load(task3_file);
    if isfield(data, 'task3_results') && isfield(data.task3_results, 'TRIAD')
        C_B_N = data.task3_results.TRIAD.R;
        fprintf('\nRotation matrix C_{B}^{N}:\n');
        disp(C_B_N);
    end
end
fprintf('Results saved to %s\n', results_dir);

stored_methods = fieldnames(task3_results); % cell array
fprintf('Task 3 results stored in memory: %s\n', strjoin(stored_methods', ', '));

end


%% ========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function M = triad_basis(v1, v2)
    % Create an orthonormal basis using the TRIAD method
    t1 = v1 / norm(v1);
    t2_temp = cross(t1, v2);
    if norm(t2_temp) < 1e-10
        warning('Vectors are nearly collinear, TRIAD may be unstable.');
        % Create an arbitrary orthogonal vector
        if abs(t1(1)) < abs(t1(2)), temp_v = [1;0;0]; else, temp_v = [0;1;0]; end
        t2_temp = cross(t1, temp_v);
    end
    t2 = t2_temp / norm(t2_temp);
    t3 = cross(t1, t2);
    M = [t1, t2, t3];
end

function [R, q] = davenport_q_method(v1B, v2B, v1N, v2N)
    % Solve for attitude using Davenport's Q-method
    w1 = 0.9999; w2 = 1 - w1;
    B = w1 * (v1N * v1B') + w2 * (v2N * v2B');
    S = B + B';
    sigma = trace(B);
    Z = [B(2,3) - B(3,2); B(3,1) - B(1,3); B(1,2) - B(2,1)];
    K = [sigma, Z'; Z, S - sigma * eye(3)];
    [eigvecs, eigvals] = eig(K, 'vector');
    [~, max_idx] = max(eigvals);
    q_opt = eigvecs(:, max_idx);
    if q_opt(1) < 0, q_opt = -q_opt; end
    q = [q_opt(1); -q_opt(2:4)]; % Conjugate for body-to-NED
    R = quat_to_rot(q);
end

function R = svd_alignment(body_vecs, ref_vecs, weights)
    if nargin < 3, weights = ones(1, length(body_vecs)); end
    B = zeros(3, 3);
    for i = 1:length(body_vecs)
        B = B + weights(i) * (ref_vecs{i} / norm(ref_vecs{i})) * (body_vecs{i} / norm(body_vecs{i}))';
    end
    [U, ~, V] = svd(B);
    M = diag([1, 1, det(U * V')]);
    R = U * M * V';
end

function q = rot_to_quaternion(R)
    tr = trace(R);
    if tr > 0
        S = sqrt(tr + 1.0) * 2; qw = 0.25 * S; qx = (R(3,2) - R(2,3)) / S; qy = (R(1,3) - R(3,1)) / S; qz = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2; qw = (R(3,2) - R(2,3)) / S; qx = 0.25 * S; qy = (R(1,2) + R(2,1)) / S; qz = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2; qw = (R(1,3) - R(3,1)) / S; qx = (R(1,2) + R(2,1)) / S; qy = 0.25 * S; qz = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2; qw = (R(2,1) - R(1,2)) / S; qx = (R(1,3) + R(3,1)) / S; qy = (R(2,3) + R(3,2)) / S; qz = 0.25 * S;
    end
    q = [qw; qx; qy; qz];
    if q(1) < 0, q = -q; end
    q = q / norm(q);
end

function R = quat_to_rot(q)
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    R=[1-2*(qy^2+qz^2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
       2*(qx*qy+qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qw*qx);
       2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx^2+qy^2)];
end

function deg = angle_between(v1, v2)
    cos_theta = max(min(dot(v1, v2) / (norm(v1) * norm(v2)), 1.0), -1.0);
    deg = acosd(cos_theta);
end

function [grav_err, earth_err] = compute_wahba_errors(C_bn, g_body, omega_ie_body, g_ref_ned, omega_ref_ned)
    g_pred_ned = C_bn * g_body;
    omega_pred_ned = C_bn * omega_ie_body;
    grav_err = angle_between(g_pred_ned, g_ref_ned);
    earth_err = angle_between(omega_pred_ned, omega_ref_ned);
end

function eul = quat_to_euler(q)
    R = quat_to_rot(q);
    phi = atan2(R(3,2), R(3,3));
    theta = -asin(R(3,1));
    psi = atan2(R(2,1), R(1,1));
    eul = [phi; theta; psi];
end

