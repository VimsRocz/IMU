function Task_3(imu_path, gnss_path, method)
% TASK_3  Solve Wahba's problem (TRIAD + Davenport + SVD) and save C_bn (body->NED)
% Inputs: imu_path, gnss_path, method (string, e.g. 'TRIAD')
% Requires: utils/triad_matrix.m, utils/dcm_to_quat.m, utils/ecef_ned_rot.m
%
% Outputs saved to: results/Task3_results_<IMU>_<GNSS>.mat
%
% Notes:
% - C_bn is body->NED (consistent across Tasks 4–6)
% - TRIAD bases guaranteed 3x3; no silent defaults

    % --- Housekeeping
    [results_dir, imu_id, gnss_id] = resolve_paths_and_ids(imu_path, gnss_path, method);

    fprintf('\nTASK 3 (TRIAD): Solve Wahba\u2019s problem (find initial attitude from body to NED)\n');

    % --- Load Task 1 + Task 2 outputs
    S1 = load(fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method)));
    S2 = load(fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method)));

    g_ned         = [0 0 S1.gravity_mag];                         % NED, +Z down
    omega_ie_ned  = S1.omega_ie_ned(:).';                         % ensure 1x3
    g_body        = S2.body_data.g_body(:).';
    omega_ie_body = S2.body_data.omega_ie_body(:).';

    % --- Prepare TRIAD vector pairs (enforce 1x3 then to columns in helper)
    fprintf('\nSubtask 3.1: Preparing vector pairs for attitude determination.\n');
    M_ned_1  = triad_matrix(g_ned,        omega_ie_ned);          % 3x3
    M_body_1 = triad_matrix(g_body,       omega_ie_body);         % 3x3
    M_ned_2  = triad_matrix(omega_ie_ned, g_ned);
    M_body_2 = triad_matrix(omega_ie_body,g_body);

    % --- TRIAD solutions
    fprintf('\nSubtask 3.2: Computing rotation matrix using TRIAD method.\n');
    C_bn_triad_1 = M_ned_1 * M_body_1.';  assert(all(size(C_bn_triad_1)==[3 3]));
    C_bn_triad_2 = M_ned_2 * M_body_2.';  assert(all(size(C_bn_triad_2)==[3 3]));

    disp('Rotation matrix (TRIAD method, Case 1):'); disp(round(C_bn_triad_1,4));
    disp('Rotation matrix (TRIAD method, Case 2):'); disp(round(C_bn_triad_2,4));

    % --- Davenport\u2019s Q-Method (use your existing implementation if present)
    fprintf('\nSubtask 3.3: Computing rotation matrix using Davenport\u2019s Q-Method.\n');
    try
        [C_bn_davenport, q_davenport] = davenport_q_method(g_ned, omega_ie_ned, g_body, omega_ie_body);
    catch
        warning('Davenport Q-Method not found; falling back to TRIAD Case 1 for Davenport output.');
        C_bn_davenport = C_bn_triad_1; 
        q_davenport    = dcm_to_quat(C_bn_davenport);
    end

    % --- SVD Wahba (use your existing implementation if present)
    fprintf('\nSubtask 3.4: Computing rotation matrix using SVD method.\n');
    try
        C_bn_svd = svd_wahba(g_ned, omega_ie_ned, g_body, omega_ie_body);
    catch
        warning('SVD Wahba not found; falling back to TRIAD Case 1 for SVD output.');
        C_bn_svd = C_bn_triad_1;
    end

    % --- Quaternions for logging
    fprintf('\nSubtask 3.5: Converting TRIAD and SVD DCMs to quaternions.\n');
    q_triad_1 = dcm_to_quat(C_bn_triad_1);
    q_triad_2 = dcm_to_quat(C_bn_triad_2);
    q_svd     = dcm_to_quat(C_bn_svd);

    % --- Basic validation against reference vectors (angles)
    fprintf('\nSubtask 3.6: Validating attitude determination and comparing methods.\n');
    refN = [g_ned(:), omega_ie_ned(:)]; 
    estB = [g_body(:), omega_ie_body(:)];
    err_deg = @(C) rad2deg(acos( max(-1,min(1, sum( (C*estB) .* refN, 1) ./ (vecnorm(C*estB).*vecnorm(refN)) )) ));
    e_triad1 = err_deg(C_bn_triad_1);
    e_dav    = err_deg(C_bn_davenport);
    e_svd    = err_deg(C_bn_svd);
    fprintf('TRIAD:   Gravity err=%.6f\u00b0, Earth-rate err=%.6f\u00b0\n', e_triad1(1), e_triad1(2));
    fprintf('Davenp.: Gravity err=%.6f\u00b0, Earth-rate err=%.6f\u00b0\n', e_dav(1),    e_dav(2));
    fprintf('SVD:     Gravity err=%.6f\u00b0, Earth-rate err=%.6f\u00b0\n', e_svd(1),    e_svd(2));

    % --- Save results for later tasks
    out3 = struct();
    out3.C_bn = struct('TRIAD_case1', C_bn_triad_1, ...
                       'TRIAD_case2', C_bn_triad_2, ...
                       'Davenport',   C_bn_davenport, ...
                       'SVD',         C_bn_svd);
    out3.q = struct('TRIAD_case1', q_triad_1, ...
                    'TRIAD_case2', q_triad_2, ...
                    'Davenport',   q_davenport, ...
                    'SVD',         q_svd);
    save(fullfile(results_dir, sprintf('Task3_results_%s_%s.mat', imu_id, gnss_id)), '-struct', 'out3', '-v7.3');

    fprintf('Task 3 results stored in %s\n', fullfile(results_dir, sprintf('Task3_results_%s_%s.mat', imu_id, gnss_id)));

end

% ------- helpers local to this file (names MUST differ from Task_3) -------

function [results_dir, imu_id, gnss_id] = resolve_paths_and_ids(imu_path, gnss_path, method)
    [imu_dir, imu_file, ~] = fileparts(imu_path);
    [gnss_dir, gnss_file, ~] = fileparts(gnss_path); %#ok<ASGLU>
    imu_id  = erase(imu_file, {'.dat','.txt','.csv'});
    gnss_id = erase(gnss_file,{'.dat','.txt','.csv'});
    results_dir = fullfile(imu_dir, 'results');
    if ~exist(results_dir, 'dir'); mkdir(results_dir); end
    addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));
    fprintf('Rotation matrices will be saved under %s\n', results_dir);
end

