function Task_3(imuFile, gnssFile, method)
    % TASK 3: Solve Wahba's problem (attitude determination)
    if nargin < 3
        method = 'TRIAD';
    end
    fprintf('\nTASK 3 (%s): Solve Wahba''s problem\n', method);

    if ~exist('results','dir')
        mkdir('results');
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

    switch upper(method)
        case 'TRIAD'
            % TRIAD (body to NED)
            t1b = v1_B;
            t2b = cross(v1_B, v2_B); t2b = t2b / norm(t2b);
            t3b = cross(t1b, t2b);
            t1n = v1_N;
            t2n = cross(v1_N, v2_N); t2n = t2n / norm(t2n);
            t3n = cross(t1n, t2n);
            R = [t1n t2n t3n] * [t1b t2b t3b]';
            q = rotm2quat(R);
        case 'DAVENPORT'
            w1 = 0.9999; w2 = 0.0001;
            B = w1*(v1_N*v1_B') + w2*(v2_N*v2_B');
            sigma = trace(B);
            S = B + B';
            Z = [B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)];
            K = [sigma Z'; Z S - sigma*eye(3)];
            [V,D] = eig(K);
            [~,idx] = max(diag(D));
            q = V(:,idx);
            if q(1) < 0, q = -q; end
            q = [q(1); -q(2:4)];
            R = quat2rotm(q');
        case 'SVD'
            B = v1_N*v1_B' + v2_N*v2_B';
            [U,~,V] = svd(B);
            R = U*diag([1 1 det(U*V')])*V';
            q = rotm2quat(R);
        otherwise
            error('Unknown method: %s', method);
    end

    if q(1) < 0, q = -q; end

    fprintf('Quaternion %s: [% .8f % .8f % .8f % .8f]\n', method, q);

    save(fullfile('results', ['Task3_attitude_' method '_' tag '.mat']), 'R', 'q');
end
