function Task_3(imu_path, gnss_path, method)
%TASK_3 Solve Wahba's problem and save body->NED rotation matrices.
%   TASK_3(IMU_PATH, GNSS_PATH, METHOD) computes an initial attitude using
%   reference vectors from Task 1 and body-frame measurements from Task 2.
%   Results are written to the MATLAB-only directory returned by
%   GET_MATLAB_RESULTS_DIR. If the Task 1 initialisation file is missing it
%   is regenerated automatically.
%
%   See also TASK_1, TASK_2.

    if nargin < 3
        method = '';
    end

    addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

    results_dir = get_matlab_results_dir();
    [~, imu_id, ~]  = fileparts(imu_path);
    [~, gnss_id, ~] = fileparts(gnss_path);

    init_file = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method));
    if ~isfile(init_file)
        fprintf('Task_3: Missing %s -- re-running Task_1 to generate it...\n', init_file);
        Task_1(imu_path, gnss_path, method);
    end
    S1 = load(init_file);

    body_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method));
    S2 = load(body_file);
    if isfield(S2, 'body_data')
        bd = S2.body_data;
    else
        bd = S2;
    end

    % NED reference vectors
    gN = S1.g_NED(:);          gN = gN / norm(gN);
    wN = S1.omega_NED(:);      wN = wN / norm(wN);
    t1N = gN;
    t2N = cross(gN, wN);       t2N = t2N / norm(t2N);
    t3N = cross(t1N, t2N);
    M_ned = [t1N, t2N, t3N];

    % Body measured vectors
    gB = bd.g_body(:);         gB = gB / norm(gB);
    wB = bd.omega_ie_body(:);  wB = wB / norm(wB);
    t1B = gB;
    t2B = cross(gB, wB);       t2B = t2B / norm(t2B);
    t3B = cross(t1B, t2B);
    M_body = [t1B, t2B, t3B];

    % Body->NED rotation
    R_tri = M_ned * M_body';

    % Davenport and SVD alternatives if available
    try
        [R_dav, q_dav] = davenport_q_method(gN, wN, gB, wB);
    catch
        warning('Davenport Q-Method not found; using TRIAD result.');
        R_dav = R_tri;
        q_dav = dcm_to_quat(R_dav);
    end
    try
        R_svd = svd_wahba(gN, wN, gB, wB);
    catch
        warning('SVD Wahba not found; using TRIAD result.');
        R_svd = R_tri;
    end

    q_tri = dcm_to_quat(R_tri);
    q_svd = dcm_to_quat(R_svd);

    out = struct();
    out.C_bn = struct('TRIAD', R_tri, 'Davenport', R_dav, 'SVD', R_svd);
    out.q = struct('TRIAD', q_tri, 'Davenport', q_dav, 'SVD', q_svd);
    save(fullfile(results_dir, sprintf('Task3_results_%s_%s.mat', imu_id, gnss_id)), '-struct', 'out', '-v7.3');

    fprintf('Rotation matrices will be saved under %s ->\n', results_dir);
end
