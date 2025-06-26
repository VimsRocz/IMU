function Task_5()
    % TASK 5: Kalman filter sensor fusion
    fprintf('\nTASK 5: Kalman filter sensor fusion\n');

    S1 = load(fullfile('results','Task1_init.mat'));
    S3 = load(fullfile('results','Task3_attitude.mat'));
    lat = S1.lat; lon = S1.lon; g_NED = S1.g_NED;
    R_BN = S3.R_tri;

    T = readtable(get_data_file('GNSS_X001.csv'));
    gnss_t = T.Posix_Time - T.Posix_Time(1);
    pos_ecef = [T.X_ECEF_m T.Y_ECEF_m T.Z_ECEF_m];
    C = ecef2ned_matrix(deg2rad(lat), deg2rad(lon));
    r0 = pos_ecef(1,:)';
    % convert GNSS ECEF coordinates to NED relative to the start point
    % use r0' to broadcast the origin across all rows
    pos_ned = (C*(pos_ecef - r0')')';

    imu = load(get_data_file('IMU_X001.dat'));
    dt = mean(diff(imu(1:100,2))); if dt<=0, dt=1/400; end
    acc_body = imu(:,6:8)/dt;
    acc_ned = (R_BN*acc_body')' + g_NED';
    imu_t = (0:size(acc_ned,1)-1)*dt;

    N = length(imu_t);
    x = zeros(6,1); P = eye(6);
    Q = eye(6)*0.01; Rm = eye(3)*1;
    gnss_idx = 1;
    fused_pos = zeros(N,3); fused_vel = zeros(N,3);
    for i=2:N
        dti = imu_t(i) - imu_t(i-1);
        F = [eye(3) eye(3)*dti; zeros(3) eye(3)];
        B = [0.5*dti^2*eye(3); dti*eye(3)];
        x = F*x + B*acc_ned(i,:)';
        P = F*P*F' + Q;
        if gnss_idx <= length(gnss_t) && abs(imu_t(i)-gnss_t(gnss_idx)) < dti/2
            z = pos_ned(gnss_idx,:)';
            H = [eye(3) zeros(3)];
            y = z - H*x;
            S = H*P*H' + Rm;
            K = P*H'/S;
            x = x + K*y;
            P = (eye(6)-K*H)*P;
            gnss_idx = gnss_idx + 1;
        end
        fused_pos(i,:) = x(1:3)';
        fused_vel(i,:) = x(4:6)';
    end

    figure; subplot(2,1,1); plot(imu_t,fused_pos); hold on; plot(gnss_t,pos_ned,'k.');
    legend('x','y','z','GNSS'); ylabel('Position (m)');
    subplot(2,1,2); plot(imu_t,fused_vel); ylabel('Velocity (m/s)'); xlabel('Time (s)');
    saveas(gcf, fullfile('results','Task5_fused.png')); close;

    save(fullfile('results','Task5_fused.mat'),'fused_pos','fused_vel');
end

function C = ecef2ned_matrix(lat, lon)
    sphi = sin(lat); cphi = cos(lat); sl = sin(lon); cl = cos(lon);
    C = [ -sphi*cl -sphi*sl cphi; -sl cl 0; -cphi*cl -cphi*sl -sphi];
end
