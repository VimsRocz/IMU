function result = Task_5(imu_path, gnss_path, method, gnss_pos_ned, truthFile)
%TASK_5  Run 9-state KF using IMU & GNSS NED positions
    if nargin < 1 || isempty(imu_path)
        error('IMU path not specified');
    end
    if nargin < 2 || isempty(gnss_path)
        error('GNSS path not specified');
    end
    if nargin < 3 || isempty(method)
        method = 'TRIAD';
    end
    if nargin < 4
        gnss_pos_ned = [];
    end
    if nargin < 5
        truthFile = '';
    end

    results_dir = 'results';
    if ~exist(results_dir,'dir')
        mkdir(results_dir);
    end
    if ~isfile(gnss_path)
        error('Task_5:GNSSFileNotFound', ...
              'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
              gnss_path);
    end
    if ~isfile(imu_path)
        error('Task_5:IMUFileNotFound', ...
              'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
              imu_path);
    end
    [~, imu_name, ~] = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
    pair_tag = [imu_name '_' gnss_name];
    tag = [pair_tag '_' method];

    if isempty(method)
        log_tag = '';
    else
        log_tag = [' (' method ')'];
    end
    fprintf('\nTASK 5%s: Sensor Fusion with Kalman Filter\n', log_tag);

    % Load attitude estimate from Task 3 results
    results_file = fullfile(results_dir, sprintf('Task3_results_%s.mat', pair_tag));
    if evalin('base','exist(''task3_results'',''var'')')
        task3_results = evalin('base','task3_results');
    else
        data = load(results_file);
        task3_results = data.task3_results;
    end
    if ~isfield(task3_results, method)
        error('Method %s not found in task3_results', method);
    end
    C_B_N = task3_results.(method).R;
    C_N_B = C_B_N';

    % Load GNSS data to obtain time and velocity
    gnss_tbl = readtable(gnss_path);
    gnss_time = gnss_tbl.Posix_Time;
    vel_cols = {'VX_ECEF_mps','VY_ECEF_mps','VZ_ECEF_mps'};
    pos_cols = {'X_ECEF_m','Y_ECEF_m','Z_ECEF_m'};
    gnss_pos_ecef = gnss_tbl{:, pos_cols};
    gnss_vel_ecef = gnss_tbl{:, vel_cols};
    first_idx = find(gnss_pos_ecef(:,1) ~= 0, 1, 'first');
    ref_r0 = gnss_pos_ecef(first_idx, :)';
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
    C_ECEF_to_NED = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
    omega_E = 7.2921159e-5;
    omega_ie_NED = omega_E * [cosd(lat_deg); 0; -sind(lat_deg)];
    if nargin < 4 || isempty(gnss_pos_ned)
        gnss_pos_ned = (C_ECEF_to_NED * (gnss_pos_ecef' - ref_r0))';
    end
    gnss_vel_ned = (C_ECEF_to_NED * gnss_vel_ecef')';
    dt_gnss = diff(gnss_time);
    gnss_accel_ned = [zeros(1,3); diff(gnss_vel_ned) ./ dt_gnss];
    gnss_accel_ecef = [zeros(1,3); diff(gnss_vel_ecef) ./ dt_gnss];
    N_gnss = size(gnss_pos_ned,1);  % number of available GNSS samples

    % Load IMU data
    imu_raw = readmatrix(imu_path);
    dt_imu = mean(diff(imu_raw(1:100,2)));
    if dt_imu <= 0 || isnan(dt_imu)
        dt_imu = 1/400;
    end
    imu_time = (0:size(imu_raw,1)-1)' * dt_imu + gnss_time(1);
    gyro_body_raw = imu_raw(:,3:5) / dt_imu;
    acc_body_raw = imu_raw(:,6:8) / dt_imu;

    % Load biases estimated in Task 2
    task2_file = fullfile(results_dir, ['Task2_body_' tag '.mat']);
    if isfile(task2_file)
        t2 = load(task2_file);
        if isfield(t2, 'accel_bias')
            accel_bias = t2.accel_bias;
        else
            accel_bias = t2.acc_bias; % backward compatibility
        end
        gyro_bias = t2.gyro_bias;
    else
        warning('Task 2 results not found, estimating biases from first samples');
        N_static = min(4000, size(acc_body_raw,1));
        accel_bias = mean(acc_body_raw(1:N_static,:),1)';
        gyro_bias = mean(gyro_body_raw(1:N_static,:),1)';
    end
    fprintf('Using accelerometer bias: [%.4f %.4f %.4f]\n', accel_bias);
    fprintf('Using gyroscope bias:     [%.6f %.6f %.6f]\n', gyro_bias);

    % Apply bias correction to IMU data
    gyro_body_raw = gyro_body_raw - gyro_bias';
    acc_body_raw  = acc_body_raw  - accel_bias';



%% ========================================================================
% Subtask 5.1-5.5: Configure and Initialize 15-State Filter
% =========================================================================
fprintf('\nSubtask 5.1-5.5: Configuring and Initializing 15-State Kalman Filter.\n');
results4 = fullfile(results_dir, sprintf('Task4_results_%s.mat', pair_tag));
if nargin < 4 || isempty(gnss_pos_ned)
    if evalin('base','exist(''task4_results'',''var'')')
        gnss_pos_ned = evalin('base','task4_results.gnss_pos_ned');
    else
        if ~isfile(results4)
            error('Task_5:MissingResults', ...
                  'Task 4 must run first and save gnss_pos_ned.');
        end
        S = load(results4,'gnss_pos_ned');
        if ~isfield(S,'gnss_pos_ned')
            error('Task_5:BadMATfile', ...
                  '''gnss_pos_ned'' not found in %s', results4);
        end
        gnss_pos_ned = S.gnss_pos_ned;
    end
end
% State vector x: [pos; vel; euler; accel_bias; gyro_bias] (15x1)
init_eul = quat_to_euler(rot_to_quaternion(C_B_N));
x = zeros(15, 1);
x(1:3)  = gnss_pos_ned(1,:)';
x(4:6)  = gnss_vel_ned(1,:)';
x(7:9)  = init_eul;
x(10:12) = accel_bias(:);
x(13:15) = gyro_bias(:);

R = eye(6) * 0.1;
H = [eye(6), zeros(6,9)];

% Covariance matrices for Kalman filter
P = eye(15);          % initial state covariance
Q = 1e-2 * eye(15);   % process noise covariance

% --- Attitude Initialization ---
q_b_n = rot_to_quaternion(C_B_N); % Initial attitude quaternion

% Gravity vector in NED frame
g_NED = [0; 0; 9.81];

% Trapezoidal integration state
prev_a_ned = zeros(3,1);
prev_vel = x(4:6);

% --- Pre-allocate Log Arrays ---
num_imu_samples = length(imu_time);
x_log = zeros(15, num_imu_samples);
euler_log = zeros(3, num_imu_samples);
zupt_log = zeros(1, num_imu_samples);
acc_log = zeros(3, num_imu_samples); % Acceleration from propagated IMU data
zupt_count = 0;
fprintf('-> 15-State filter initialized.\n');

% Interpolate GNSS measurements to IMU timestamps for continuous updates
gnss_pos_interp = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear', 'extrap');
gnss_vel_interp = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear', 'extrap');

%% ========================================================================
% Subtask 5.6: Kalman Filter for Sensor Fusion
% =========================================================================
fprintf('\nSubtask 5.6: Running Kalman Filter for sensor fusion.\n');

% --- Main Filter Loop ---
fprintf('-> Starting filter loop over %d IMU samples...\n', num_imu_samples);
for i = 1:num_imu_samples
    % --- 1. State Propagation (Prediction) ---
    F = eye(15);
    F(1:3, 4:6) = eye(3) * dt_imu;
    P = F * P * F' + Q * dt_imu;

    % --- 2. Attitude Propagation ---
    corrected_gyro = gyro_body_raw(i,:)' - x(13:15);
    corrected_accel = acc_body_raw(i,:)' - x(10:12);
    current_omega_ie_b = C_N_B * omega_ie_NED;
    w_b = corrected_gyro - current_omega_ie_b;
    q_b_n = propagate_quaternion(q_b_n, w_b, dt_imu);
    C_B_N = quat_to_rot(q_b_n);
    a_ned = C_B_N * corrected_accel + g_NED;
    if i > 1
        vel_new = prev_vel + 0.5 * (a_ned + prev_a_ned) * dt_imu;
        pos_new = x(1:3) + 0.5 * (vel_new + prev_vel) * dt_imu;
    else
        vel_new = x(4:6);
        pos_new = x(1:3);
    end
    x(4:6) = vel_new;
    x(1:3) = pos_new;
    x(7:9) = quat_to_euler(q_b_n);
    acc_log(:,i) = a_ned;
    % --- 3. Measurement Update (Correction) ---
    z = [gnss_pos_interp(i,:)'; gnss_vel_interp(i,:)'];
    y = z - H * x;
    S = H * P * H' + R;
    K = (P * H') / S;
    x = x + K * y;
    P = (eye(15) - K * H) * P;

    % update integrator history after correction
    prev_vel = x(4:6);
    prev_a_ned = a_ned;
    % --- 4. Zero-Velocity Update (ZUPT) ---
    win_size = 80;
    if i >= win_size
        acc_win  = acc_body_raw(i-win_size+1:i, :);
        gyro_win = gyro_body_raw(i-win_size+1:i, :);
        if is_static(acc_win, gyro_win)
            zupt_count = zupt_count + 1;
            zupt_log(i) = 1;
            H_z = [zeros(3,3), eye(3), zeros(3,9)];
            R_z = eye(3) * 1e-6;
            y_z = -H_z * x;
            S_z = H_z * P * H_z' + R_z;
            K_z = (P * H_z') / S_z;
            x = x + K_z * y_z;
            P = (eye(15) - K_z * H_z) * P;
        end

        % close win_size check
    end


    % --- Log State and Attitude ---
    x_log(:, i) = x;
    euler_log(:, i) = quat_to_euler(q_b_n);
end  % end of main filter loop
fprintf('-> Filter loop complete. Total ZUPT applications: %d\n', zupt_count);

%% ========================================================================
% Subtask 5.7: Handle Event at 5000s
% =========================================================================
fprintf('\nSubtask 5.7: No event handling needed as time < 5000s.\n');

%% ========================================================================
% Subtask 5.8: Plotting Results
% =========================================================================
fprintf('\nSubtask 5.8: Plotting Kalman filter results.\n');

% Ensure array sizes match for plotting
if numel(imu_time) ~= size(x_log,2)
    N = min(numel(imu_time), size(x_log,2));
    imu_time = imu_time(1:N);
    x_log = x_log(:,1:N);
    euler_log = euler_log(:,1:N);
    zupt_log = zupt_log(1:N);
end
if numel(gnss_time) ~= size(gnss_pos_ned,1)
    N = min(numel(gnss_time), size(gnss_pos_ned,1));
    gnss_time = gnss_time(1:N);
    gnss_pos_ned = gnss_pos_ned(1:N,:);
    gnss_vel_ned = gnss_vel_ned(1:N,:);
    gnss_accel_ned = gnss_accel_ned(1:N,:);
end

% Extract velocity states and derive acceleration from them
vel_log = x_log(4:6, :);
if numel(imu_time) > 1
    dt_vec = diff(imu_time);
    accel_from_vel = [zeros(3,1), diff(vel_log,1,2) ./ dt_vec'];
else
    accel_from_vel = zeros(3, numel(imu_time));
end

% --- IMU-only integration for comparison ---
imu_pos = zeros(3, numel(imu_time));
imu_vel = zeros(3, numel(imu_time));
imu_acc = acc_log;
imu_pos(:,1) = gnss_pos_ned(1,:)';
imu_vel(:,1) = gnss_vel_ned(1,:)';
for k = 2:numel(imu_time)
    dt = imu_time(k) - imu_time(k-1);
    imu_vel(:,k) = imu_vel(:,k-1) + 0.5*(imu_acc(:,k)+imu_acc(:,k-1))*dt;
    imu_pos(:,k) = imu_pos(:,k-1) + 0.5*(imu_vel(:,k)+imu_vel(:,k-1))*dt;
end

% --- Convert fused and IMU-only states to ECEF ---
C_NED_to_ECEF = C_ECEF_to_NED';
pos_ecef_fused = (C_NED_to_ECEF * x_log(1:3,:) + ref_r0)';
vel_ecef_fused = (C_NED_to_ECEF * vel_log)';
acc_ecef_fused = (C_NED_to_ECEF * accel_from_vel)';
pos_ecef_imu = (C_NED_to_ECEF * imu_pos + ref_r0)';
vel_ecef_imu = (C_NED_to_ECEF * imu_vel)';
acc_ecef_imu = (C_NED_to_ECEF * imu_acc)';

% --- Convert all states to body frame ---
N = numel(imu_time);
pos_body_fused = zeros(N,3); vel_body_fused = zeros(N,3); acc_body_fused = zeros(N,3);
pos_body_imu = zeros(N,3);   vel_body_imu = zeros(N,3);   acc_body_imu = zeros(N,3);
gnss_pos_body = zeros(N,3);  gnss_vel_body = zeros(N,3);  gnss_acc_body = zeros(N,3);
for k = 1:N
    C_B_Nk = euler_to_rot(euler_log(:,k));
    C_N_Bk = C_B_Nk';
    pos_body_fused(k,:) = (C_N_Bk * x_log(1:3,k))';
    vel_body_fused(k,:) = (C_N_Bk * vel_log(:,k))';
    acc_body_fused(k,:) = (C_N_Bk * accel_from_vel(:,k))';
    pos_body_imu(k,:)   = (C_N_Bk * imu_pos(:,k))';
    vel_body_imu(k,:)   = (C_N_Bk * imu_vel(:,k))';
    acc_body_imu(k,:)   = (C_N_Bk * imu_acc(:,k))';
    if k <= N_gnss
        gnss_pos_body(k,:) = (C_N_Bk * gnss_pos_ned(k,:)')';
        gnss_vel_body(k,:) = (C_N_Bk * gnss_vel_ned(k,:)')';
        gnss_acc_body(k,:) = (C_N_Bk * gnss_accel_ned(k,:)')';
    else
        gnss_pos_body(k,:) = [NaN NaN NaN];
        gnss_vel_body(k,:) = [NaN NaN NaN];
        gnss_acc_body(k,:) = [NaN NaN NaN];
    end
end


% --- Frame comparisons with Truth ----------------------------------
token = regexp(imu_name,'IMU_(X\d+)','tokens');
dataset_id = token{1}{1};
if ~isempty(truthFile)
    truth_candidates = {truthFile};
else
    data_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'Data');
    truth_candidates = {fullfile(data_dir, sprintf('STATE_%s.txt', dataset_id))};
end
truth_data = [];
% Pre-allocate truth arrays to avoid "undefined variable" errors when no
% reference solution is available. These will remain empty if truth data is
% not found and downstream plotting code will simply skip drawing the Truth
% overlay.
pos_t_ecef = []; vel_t_ecef = []; acc_t_ecef = [];
pos_t_ned  = []; vel_t_ned  = []; acc_t_ned  = [];
pos_t_body = []; vel_t_body = []; acc_t_body = [];
for i=1:numel(truth_candidates)
    if exist(truth_candidates{i},'file')
        truth_data = load(truth_candidates{i});
        break;
    end
end
if ~isempty(truthFile) && isempty(truth_data)
    warning('Reference file %s not found. Skipping overlay.', truthFile);
end
if ~isempty(truth_data)
    t_truth = truth_data(:,2) + gnss_time(1);
    pos_t_ecef = truth_data(:,3:5);
    vel_t_ecef = truth_data(:,6:8);
    dt_t = [0; diff(t_truth)];
    acc_t_ecef = [zeros(1,3); diff(vel_t_ecef)./dt_t(2:end)];
    pos_t_ned = (C_ECEF_to_NED*(pos_t_ecef'-ref_r0))';
    vel_t_ned = (C_ECEF_to_NED*vel_t_ecef')';
    acc_t_ned = (C_ECEF_to_NED*acc_t_ecef')';
    pos_t_body = (C_N_B*pos_t_ned')';
    vel_t_body = (C_N_B*vel_t_ned')';
    acc_t_body = (C_N_B*acc_t_ned')';
    interp_truth = @(d) interp1(t_truth, d, imu_time, 'linear', 'extrap');
else
    interp_truth = @(d) [];
end

gnss_pos_ned_i   = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear','extrap');
gnss_vel_ned_i   = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear','extrap');
gnss_acc_ned_i   = interp1(gnss_time, gnss_accel_ned, imu_time,'linear','extrap');
gnss_pos_ecef_i  = interp1(gnss_time, gnss_pos_ecef, imu_time,'linear','extrap');
gnss_vel_ecef_i  = interp1(gnss_time, gnss_vel_ecef, imu_time,'linear','extrap');
gnss_acc_ecef_i  = interp1(gnss_time, gnss_accel_ecef, imu_time,'linear','extrap');
gnss_pos_body_i  = (C_N_B*gnss_pos_ned_i')';
gnss_vel_body_i  = (C_N_B*gnss_vel_ned_i')';
gnss_acc_body_i  = (C_N_B*gnss_acc_ned_i')';

pos_ecef_fused = (C_NED_to_ECEF*x_log(1:3,:))'+ref_r0';
vel_ecef_fused = (C_NED_to_ECEF*vel_log)';
acc_ecef_fused = (C_NED_to_ECEF*accel_from_vel)';
pos_body_fused = (C_N_B*x_log(1:3,:))';
vel_body_fused = (C_N_B*vel_log)';
acc_body_fused = (C_N_B*accel_from_vel)';

frames = {'NED','ECEF','BODY'};
labels_frames = {{'N','E','D'},{'X','Y','Z'},{'X','Y','Z'}};
for f = 1:3
    frame = frames{f}; labs = labels_frames{f};
    fig = figure('Name',['Compare ' frame],'Position',[100 100 1200 900]);
    tlo = tiledlayout(3,3);
    for r = 1:3
        for c = 1:3
            ax = nexttile; hold(ax,'on');
            switch frame
                case 'NED'
                    plot(imu_time, gnss_pos_ned_i(:,c),'k-','DisplayName','GNSS');
                    plot(imu_time, imu_pos(r,:), '--','Color',[0.5 0.5 0.5],'DisplayName','IMU');
                    plot(imu_time, x_log(r,:), 'b-','DisplayName','Fused');
                    ti = interp_truth(pos_t_ned); if ~isempty(ti), plot(imu_time, ti(:,c),'g-','DisplayName','Truth'); end
                case 'ECEF'
                    plot(imu_time, gnss_pos_ecef_i(:,c),'k-','DisplayName','GNSS');
                    plot(imu_time, pos_ecef_imu(:,c),'--','Color',[0.5 0.5 0.5],'DisplayName','IMU');
                    plot(imu_time, pos_ecef_fused(:,c),'b-','DisplayName','Fused');
                    ti = interp_truth(pos_t_ecef); if ~isempty(ti), plot(imu_time, ti(:,c),'g-','DisplayName','Truth'); end
                case 'BODY'
                    plot(imu_time, gnss_pos_body_i(:,c),'k-','DisplayName','GNSS');
                    plot(imu_time, pos_body_imu(:,c),'--','Color',[0.5 0.5 0.5],'DisplayName','IMU');
                    plot(imu_time, pos_body_fused(:,c),'b-','DisplayName','Fused');
                    ti = interp_truth(pos_t_body); if ~isempty(ti), plot(imu_time, ti(:,c),'g-','DisplayName','Truth'); end
            end
            if r==1
                title(['Position ' labs{c}]); ylabel('[m]');
            elseif r==2
                title(['Velocity ' labs{c}]); ylabel('[m/s]');
            else
                title(['Acceleration ' labs{c}]); ylabel('[m/s^2]');
            end
            xlabel('Time (s)'); grid on; legend;
        end
    end
    sgtitle(['GNSS vs IMU vs Fused vs Truth in ' frame]);
    out_pdf = fullfile(results_dir,sprintf('%s_%s_Truth_GNSS_IMU.pdf',dataset_id,frame));
    set(gcf,'PaperPositionMode','auto');
    print(gcf, out_pdf, '-dpdf', '-bestfit');
end

%% ========================================================================
% Subtask 5.8a: Fusion Comparison Across Methods
% =========================================================================
fprintf('\nSubtask 5.8a: Plotting fused output against GNSS for all methods.\n');

% Interpolate GNSS data to IMU timeline
gnss_pos_i = interp1(gnss_time, gnss_pos_ned, imu_time, 'linear', 'extrap');
gnss_vel_i = interp1(gnss_time, gnss_vel_ned, imu_time, 'linear', 'extrap');
gnss_acc_i = interp1(gnss_time, gnss_accel_ned, imu_time, 'linear', 'extrap');

% Collect fused results for each attitude initialisation method
method_list = {'TRIAD','Davenport','SVD'};
col_map = struct('TRIAD',[0 0.4470 0.7410], ...
                 'Davenport',[0.8500 0.3250 0.0980], ...
                 'SVD',[0.4660 0.6740 0.1880]);
fused = struct();
for mIdx = 1:numel(method_list)
    m = method_list{mIdx};
    res_file = fullfile(results_dir, ...
        sprintf('%s_%s_%s_task5_results.mat', imu_name, gnss_name, m));
    if isfile(res_file)
        dat = load(res_file, 'x_log', 'vel_log', 'accel_from_vel');
        fused.(m).pos = dat.x_log(1:3,:)';
        fused.(m).vel = dat.vel_log';
        fused.(m).acc = dat.accel_from_vel';
    elseif strcmpi(m, method)
        % current run results have not been saved yet
        fused.(m).pos = x_log(1:3,:)';
        fused.(m).vel = x_log(4:6,:)';
        if numel(imu_time) > 1
            dv = diff(fused.(m).vel); dtv = diff(imu_time);
            fused.(m).acc = [zeros(1,3); dv ./ dtv];
        else
            fused.(m).acc = zeros(size(fused.(m).vel));
        end
    end
end

labels = {'N','E','D'};
for mIdx = 1:numel(method_list)
    m = method_list{mIdx};
    if ~isfield(fused, m); continue; end
    data = fused.(m);
    fig = figure('Name',['Fusion ' m],'Position',[100 100 1200 900]);
    tlo = tiledlayout(3,3);
    for r = 1:3
        for c = 1:3
            nexttile; hold on;
            switch r
                case 1
                    plot(imu_time, gnss_pos_i(:,c),'k-','DisplayName','GNSS');
                    plot(imu_time, data.pos(:,c),'Color',col_map.(m), ...
                        'DisplayName',m);
                    ylabel('[m]');
                    title(['Position ' labels{c}]);
                case 2
                    plot(imu_time, gnss_vel_i(:,c),'k-','DisplayName','GNSS');
                    plot(imu_time, data.vel(:,c),'Color',col_map.(m), ...
                        'DisplayName',m);
                    ylabel('[m/s]');
                    title(['Velocity ' labels{c}]);
                otherwise
                    plot(imu_time, gnss_acc_i(:,c),'k-','DisplayName','GNSS');
                    plot(imu_time, data.acc(:,c),'Color',col_map.(m), ...
                        'DisplayName',m);
                    ylabel('[m/s^2]');
                    title(['Acceleration ' labels{c}]);
            end
            xlabel('Time (s)'); grid on;
            if r==1 && c==3, legend; end
        end
    end
    sgtitle(['Task 5: Sensor Fusion Results – ' m]);
    out_pdf = fullfile(results_dir, sprintf('5_Fusion_Results_%s.pdf', m));
    set(gcf,'PaperPositionMode','auto');
    print(gcf, out_pdf, '-dpdf', '-bestfit');
end

% --- Combined figure with all methods ---------------------------------
avail_methods = fieldnames(fused);
if ~isempty(avail_methods)
    fig = figure('Name','Fusion All Methods','Position',[100 100 1200 900]);
    tlo = tiledlayout(3,3);
    for r = 1:3
        for c = 1:3
            nexttile; hold on;
            switch r
                case 1
                    plot(imu_time, gnss_pos_i(:,c),'k-','DisplayName','GNSS');
                    for j=1:numel(avail_methods)
                        m = avail_methods{j};
                        plot(imu_time, fused.(m).pos(:,c),'Color',col_map.(m), ...
                            'DisplayName',m);
                    end
                    ylabel('[m]');
                    title(['Position ' labels{c}]);
                case 2
                    plot(imu_time, gnss_vel_i(:,c),'k-','DisplayName','GNSS');
                    for j=1:numel(avail_methods)
                        m = avail_methods{j};
                        plot(imu_time, fused.(m).vel(:,c),'Color',col_map.(m));
                    end
                    ylabel('[m/s]');
                    title(['Velocity ' labels{c}]);
                otherwise
                    plot(imu_time, gnss_acc_i(:,c),'k-','DisplayName','GNSS');
                    for j=1:numel(avail_methods)
                        m = avail_methods{j};
                        plot(imu_time, fused.(m).acc(:,c),'Color',col_map.(m));
                    end
                    ylabel('[m/s^2]');
                    title(['Acceleration ' labels{c}]);
            end
            xlabel('Time (s)'); grid on;
            if r==1 && c==3, legend; end
        end
    end
    sgtitle('Task 5: Sensor Fusion Results – All Methods');
    out_pdf = fullfile(results_dir,'5_Fusion_Results_All_Methods.pdf');
    set(gcf,'PaperPositionMode','auto');
    print(gcf, out_pdf, '-dpdf', '-bestfit');
end

%% --- End-of-run summary statistics --------------------------------------
% Interpolate filter estimates to GNSS timestamps for residual analysis
% The transpose on x_log ensures interp1 operates over rows (time). The
% result should be Nx3 matching the GNSS matrices, so avoid an extra
% transpose which previously produced a 3xN array and caused dimension
% mismatches when subtracting from gnss_pos_ned.
pos_interp = interp1(imu_time, x_log(1:3,:)', gnss_time, 'linear', 'extrap');
vel_interp = interp1(imu_time, x_log(4:6,:)', gnss_time, 'linear', 'extrap');
res_pos = pos_interp - gnss_pos_ned;
res_vel = vel_interp - gnss_vel_ned;
rmse_pos = sqrt(mean(sum(res_pos.^2,2)));
rmse_vel = sqrt(mean(sum(res_vel.^2,2)));
% Both vectors are 3x1 column vectors so avoid an extra transpose which
% previously produced a 3x3 matrix due to implicit broadcasting.
final_pos_err = norm(x_log(1:3,end) - gnss_pos_ned(end,:)');
final_vel_err = norm(vel_log(:,end) - gnss_vel_ned(end,:)');
final_acc_err = norm(accel_from_vel(:,end) - gnss_accel_ned(end,:)');
rms_resid_pos = sqrt(mean(res_pos.^2,'all'));
rms_resid_vel = sqrt(mean(res_vel.^2,'all'));
max_resid_pos = max(vecnorm(res_pos,2,2));
min_resid_pos = min(vecnorm(res_pos,2,2));
max_resid_vel = max(vecnorm(res_vel,2,2));
min_resid_vel = min(vecnorm(res_vel,2,2));

% --- Plot: Position Residuals ---
figure('Name', 'KF Results: Position Residuals', 'Position', [150 150 1200 600]);
err_labels = {'N', 'E', 'D'};
for i = 1:3
    subplot(3,1,i);
    plot(gnss_time, res_pos(:,i), 'b-');
    grid on; ylabel('[m]'); title(['Residual ' err_labels{i}]);
end
xlabel('Time (s)'); sgtitle('Position Residuals (KF - GNSS)');
err_file = fullfile(results_dir, sprintf('%s_Task5_ErrorAnalysis.pdf', tag));
set(gcf,'PaperPositionMode','auto');
print(gcf, err_file, '-dpdf', '-bestfit');
fprintf('Saved plot: %s\n', err_file);
summary_line = sprintf(['[SUMMARY] method=%s rmse_pos=%.2fm rmse_vel=%.2fm final_pos=%.2fm final_vel=%.2fm/s final_acc=%.2fm/s^2 ' ...
    'mean_resid_pos=%.2f rms_resid_pos=%.2f max_resid_pos=%.2f min_resid_pos=%.2f ' ...
    'mean_resid_vel=%.2f rms_resid_vel=%.2f max_resid_vel=%.2f min_resid_vel=%.2f ' ...
    'accel_bias=%.4f gyro_bias=%.4f ZUPT_count=%d'], ...
    method, rmse_pos, rmse_vel, final_pos_err, final_vel_err, final_acc_err, mean(vecnorm(res_pos,2,2)), rms_resid_pos, ...
    max_resid_pos, min_resid_pos, mean(vecnorm(res_vel,2,2)), rms_resid_vel, ...
    max_resid_vel, min_resid_vel, norm(accel_bias), norm(gyro_bias), zupt_count);
fprintf('%s\n', summary_line);
fprintf('Final position error: %.4f m\n', final_pos_err);
fprintf('Final velocity error: %.4f m/s\n', final_vel_err);
fprintf('Final acceleration error: %.4f m/s^2\n', final_acc_err);
fid = fopen(fullfile(results_dir, [tag '_summary.txt']), 'w');
fprintf(fid, '%s\n', summary_line);
fclose(fid);

% Store summary metrics and biases for later analysis
results = struct('method', method, 'rmse_pos', rmse_pos, 'rmse_vel', rmse_vel, ...
    'final_pos_error', final_pos_err, 'final_vel_error', final_vel_err, ...
    'final_acc_error', final_acc_err, 'accel_bias', accel_bias, 'gyro_bias', gyro_bias);
perf_file = fullfile(results_dir, 'IMU_GNSS_bias_and_performance.mat');
if isfile(perf_file)
    save(perf_file, '-append', 'results');
else
    save(perf_file, 'results');
end

summary_file = fullfile(results_dir, 'IMU_GNSS_summary.txt');
fid_sum = fopen(summary_file, 'a');
fprintf(fid_sum, '%s\n', summary_line);
fclose(fid_sum);

% Persist core results for unit tests and further analysis
results_file = fullfile(results_dir, sprintf('Task5_results_%s.mat', pair_tag));
save(results_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log');
fprintf('Results saved to %s\n', results_file);

method_file = fullfile(results_dir, [tag '_task5_results.mat']);
save(method_file, 'gnss_pos_ned', 'gnss_vel_ned', 'gnss_accel_ned', ...
    'x_log', 'vel_log', 'accel_from_vel', 'euler_log', 'zupt_log');
fprintf('Method-specific results saved to %s\n', method_file);

% Return results structure and store in base workspace
result = results;
assignin('base', 'task5_results', result);


end % end main function

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================
function q_new = propagate_quaternion(q_old, w, dt)
        w_norm = norm(w);
        if w_norm > 1e-9
            axis = w / w_norm;
            angle = w_norm * dt;
            dq = [cos(angle/2); axis * sin(angle/2)];
        else
            dq = [1; 0; 0; 0];
        end
        q_new = quat_multiply(q_old, dq);
    end

function q_out = quat_multiply(q1, q2)
        w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
        w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
        q_out = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
                 w1*x2 + x1*w2 + y1*z2 - z1*y2;
                 w1*y2 - x1*z2 + y1*w2 + z1*x2;
                 w1*z2 + x1*y2 - y1*x2 + z1*w2];
    end

function euler = quat_to_euler(q)
        w = q(1); x = q(2); y = q(3); z = q(4);
        sinr_cosp = 2 * (w * x + y * z);
        cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

        sinp = 2 * (w * y - z * x);
        if abs(sinp) >= 1
            pitch = sign(sinp) * (pi/2);
        else
            pitch = asin(sinp);
        end

        siny_cosp = 2 * (w * z + x * y);
        cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
        euler = [roll; pitch; yaw];
    end

function R = quat_to_rot(q)
        qw = q(1); qx = q(2); qy = q(3); qz = q(4);
        R = [1 - 2 * (qy^2 + qz^2), 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy);
             2 * (qx*qy + qw*qz), 1 - 2 * (qx^2 + qz^2), 2 * (qy*qz - qw*qx);
             2 * (qx*qz - qw*qy), 2 * (qy*qz + qw*qx), 1 - 2 * (qx^2 + qy^2)];
    end

function q = rot_to_quaternion(R)
        tr = trace(R);
        if tr > 0
            S = sqrt(tr + 1.0) * 2;
            qw = 0.25 * S;
            qx = (R(3,2) - R(2,3)) / S;
            qy = (R(1,3) - R(3,1)) / S;
            qz = (R(2,1) - R(1,2)) / S;
        elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
            S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
            qw = (R(3,2) - R(2,3)) / S;
            qx = 0.25 * S;
            qy = (R(1,2) + R(2,1)) / S;
            qz = (R(1,3) + R(3,1)) / S;
        elseif R(2,2) > R(3,3)
            S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
            qw = (R(1,3) - R(3,1)) / S;
            qx = (R(1,2) + R(2,1)) / S;
            qy = 0.25 * S;
            qz = (R(2,3) + R(3,2)) / S;
        else
            S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
            qw = (R(2,1) - R(1,2)) / S;
            qx = (R(1,3) + R(3,1)) / S;
            qy = (R(2,3) + R(3,2)) / S;
            qz = 0.25 * S;
        end
        q = [qw; qx; qy; qz];
        if q(1) < 0, q = -q; end
        q = q / norm(q);
    end

function is_stat = is_static(acc, gyro)
        acc_thresh = 0.01; gyro_thresh = 1e-6;
        is_stat = all(var(acc,0,1) < acc_thresh) && ...
                   all(var(gyro,0,1) < gyro_thresh);
    end

function R = euler_to_rot(eul)
        cr = cos(eul(1)); sr = sin(eul(1));
        cp = cos(eul(2)); sp = sin(eul(2));
        cy = cos(eul(3)); sy = sin(eul(3));
        R = [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy;
             cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy;
             -sp,   sr*cp,            cr*cp];
    end

