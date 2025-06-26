function Task_4(imuFile, gnssFile)
    % TASK 4: GNSS + IMU data comparison
    fprintf('\nTASK 4: GNSS and IMU data comparison\n');

    if ~exist('results','dir')
        mkdir('results');
    end
    [~, imu_name, ~] = fileparts(imuFile);
    [~, gnss_name, ~] = fileparts(gnssFile);
    tag = [imu_name '_' gnss_name];

    S1 = load(fullfile('results', ['Task1_init_' tag '.mat']));
    S2 = load(fullfile('results', ['Task3_attitude_' tag '.mat']));
    lat = S1.lat; lon = S1.lon; g_NED = S1.g_NED;
    R_BN = S2.R_tri; % use TRIAD result

    opts = detectImportOptions(get_data_file(gnssFile), 'NumHeaderLines',0);
    T = readtable(get_data_file(gnssFile), opts);
    time = T.Posix_Time - T.Posix_Time(1);
    pos_ecef = [T.X_ECEF_m T.Y_ECEF_m T.Z_ECEF_m];
    vel_ecef = [T.VX_ECEF_mps T.VY_ECEF_mps T.VZ_ECEF_mps];

    C = ecef2ned_matrix(deg2rad(lat), deg2rad(lon));
    r0 = pos_ecef(1,:)';
    pos_ned = (C*(pos_ecef - r0')')';
    vel_ned = (C*vel_ecef')';

    data = load(get_data_file(imuFile));
    dt = mean(diff(data(1:100,2))); if dt<=0, dt=1/400; end
    acc_body = data(:,6:8)/dt;
    acc_ned = (R_BN * acc_body')' + g_NED';
    vel_est = cumtrapz(dt, acc_ned);
    pos_est = cumtrapz(dt, vel_est);

    % Interpolate GNSS to IMU time for comparison
    imu_t = (0:size(acc_ned,1)-1)*dt;
    pos_gnss_interp = interp1(time, pos_ned, imu_t, 'linear', 'extrap');
    diff_max = max(abs(pos_gnss_interp - pos_est), [], 'all');
    fprintf('Max NED position diff vs GNSS: %.6f m\n', diff_max);

    figure; subplot(3,1,1); plot(time,pos_ned(:,1),'k'); hold on; plot((0:length(pos_est)-1)*dt,pos_est(:,1),'r'); ylabel('North (m)');
    subplot(3,1,2); plot(time,pos_ned(:,2),'k'); hold on; plot((0:length(pos_est)-1)*dt,pos_est(:,2),'r'); ylabel('East (m)');
    subplot(3,1,3); plot(time,pos_ned(:,3),'k'); hold on; plot((0:length(pos_est)-1)*dt,pos_est(:,3),'r'); ylabel('Down (m)'); xlabel('Time (s)'); legend('GNSS','IMU');
    saveas(gcf, fullfile('results', ['Task4_compare_' tag '.png'])); close;

    save(fullfile('results', ['Task4_compare_' tag '.mat']), 'pos_ned','pos_est');
end

function C = ecef2ned_matrix(lat, lon)
    sphi = sin(lat); cphi = cos(lat); sl = sin(lon); cl = cos(lon);
    C = [ -sphi*cl -sphi*sl cphi; -sl cl 0; -cphi*cl -cphi*sl -sphi];
end
