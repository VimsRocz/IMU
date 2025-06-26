function Task_3()
    % TASK 3: Solve Wahba's problem (attitude determination)
    fprintf('\nTASK 3: Solve Wahba''s problem\n');

    S1 = load(fullfile('results','Task1_init.mat'));
    S2 = load(fullfile('results','Task2_body.mat'));
    g_NED = S1.g_NED; omega_NED = S1.omega_NED;
    g_body = S2.g_body; omega_ie_body = S2.omega_ie_body;

    v1_B = g_body / norm(g_body);
    v2_B = omega_ie_body / norm(omega_ie_body);
    v1_N = g_NED / norm(g_NED);
    v2_N = omega_NED / norm(omega_NED);

    % TRIAD (body to NED)
    t1b = v1_B;
    t2b = cross(v1_B, v2_B); t2b = t2b / norm(t2b);
    t3b = cross(t1b, t2b);
    t1n = v1_N;
    t2n = cross(v1_N, v2_N); t2n = t2n / norm(t2n);
    t3n = cross(t1n, t2n);
    R_tri = [t1n t2n t3n] * [t1b t2b t3b]';
    q_tri = rotm2quat(R_tri);
    if q_tri(1) < 0, q_tri = -q_tri; end

    % SVD alignment
    B = v1_N*v1_B' + v2_N*v2_B';
    [U,~,V] = svd(B);
    R_svd = U*diag([1 1 det(U*V')])*V';
    q_svd = rotm2quat(R_svd);
    if q_svd(1) < 0, q_svd = -q_svd; end

    fprintf('Quaternion TRIAD: [% .8f % .8f % .8f % .8f]\n', q_tri);
    fprintf('Quaternion SVD:   [% .8f % .8f % .8f % .8f]\n', q_svd);

    save(fullfile('results','Task3_attitude.mat'), 'R_tri', 'R_svd', 'q_tri', 'q_svd');
end
