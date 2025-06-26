function Task_3(imuFile, gnssFile, method)
    % TASK 3: Solve Wahba's problem (attitude determination)
    fprintf('\nTASK 3: Solve Wahba''s problem\n');

    if ~exist('results','dir')
        mkdir('results');
    end
    if nargin < 3 || isempty(method)
        method = 'TRIAD';
    end
    [~, imu_name, ~] = fileparts(imuFile);
    [~, gnss_name, ~] = fileparts(gnssFile);
    tag = [imu_name '_' gnss_name];

    S1 = load(fullfile('results', ['Task1_init_' tag '.mat']));
    S2 = load(fullfile('results', ['Task2_body_' tag '.mat']));
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

    % Davenport's Q-Method
    w_gravity = 0.9999;
    w_omega   = 0.0001;
    B_dav = w_gravity*(v1_N*v1_B') + w_omega*(v2_N*v2_B');
    sigma = trace(B_dav);
    S = B_dav + B_dav';
    Z = [B_dav(2,3)-B_dav(3,2); B_dav(3,1)-B_dav(1,3); B_dav(1,2)-B_dav(2,1)];
    K = [sigma, Z'; Z, S - sigma*eye(3)];
    [Vd, Dd] = eig(K);
    [~, idx] = max(diag(Dd));
    qd = Vd(:, idx);
    if qd(1) < 0, qd = -qd; end
    q_dav = [qd(1); -qd(2:4)]; % conjugate for body to NED
    qw = q_dav(1); qx = q_dav(2); qy = q_dav(3); qz = q_dav(4);
    R_dav = [1-2*(qy^2+qz^2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy);
             2*(qx*qy + qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz - qw*qx);
             2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1-2*(qx^2+qy^2)];

    % SVD alignment
    B = v1_N*v1_B' + v2_N*v2_B';
    [U,~,V] = svd(B);
    R_svd = U*diag([1 1 det(U*V')])*V';
    q_svd = rotm2quat(R_svd);
    if q_svd(1) < 0, q_svd = -q_svd; end

    fprintf('Quaternion TRIAD:     [% .8f % .8f % .8f % .8f]\n', q_tri);
    fprintf('Quaternion Davenport: [% .8f % .8f % .8f % .8f]\n', q_dav);
    fprintf('Quaternion SVD:       [% .8f % .8f % .8f % .8f]\n', q_svd);

    switch lower(method)
        case 'triad'
            R_BN = R_tri; q_BN = q_tri;
        case 'davenport'
            R_BN = R_dav; q_BN = q_dav;
        case 'svd'
            R_BN = R_svd; q_BN = q_svd;
        otherwise
            error('Unknown method: %s', method);
    end

    save(fullfile('results', ['Task3_attitude_' method '_' tag '.mat']), 'R_BN', 'q_BN');
end
