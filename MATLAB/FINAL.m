function FINAL(imu_path, gnss_path, varargin)
%FINAL  MATLAB reimplementation of GNSS_IMU_Fusion.py
%   FINAL(IMU_PATH, GNSS_PATH, METHOD) processes a pair of IMU and GNSS
%   files using a selected attitude initialisation method.  The script
%   is a simplified port of the Python reference implementation.  It
%   performs the following steps:
%     1) compute reference vectors in the NED frame from the first GNSS
%        sample (gravity and Earth rotation rate)
%     2) estimate gravity and Earth rate in the body frame from a static
%        interval of IMU data
%     3) determine the initial attitude using the TRIAD, Davenport or SVD
%        method
%     4) integrate IMU acceleration in the NED frame
%     5) run a simple Kalman filter to fuse GNSS position/velocity with
%        inertial data
%   Results are written to results/<tag>_final.mat
%
%   Example:
%      FINAL('IMU_X001.dat','GNSS_X001.csv','Davenport');
%
%   This implementation focusses on clarity rather than completeness and
%   therefore omits some of the advanced functionality of the Python
%   version such as magnetometer handling and extensive plotting.

if numel(dbstack) <= 1
    error(['FINAL must be called as a function with two file names. Example:\n',...
          '    FINAL(''IMU_X001.dat'',''GNSS_X001.csv'');']);
end

if nargin < 2
    error('Usage: FINAL(''IMU_PATH'',''GNSS_PATH'',[METHOD])');
end

if nargin < 3
    method = 'Davenport';
else
    method = varargin{1};
end

%% ----- Task 1: reference vectors in NED -------------------------------
T = readtable(gnss_path);
valid = find(T.X_ECEF_m ~= 0 | T.Y_ECEF_m ~= 0 | T.Z_ECEF_m ~= 0, 1);
if isempty(valid)
    error('No valid GNSS rows in %s', gnss_path);
end
x = T.X_ECEF_m(valid); y = T.Y_ECEF_m(valid); z = T.Z_ECEF_m(valid);
[lat, lon, ~] = ecef2geod(x, y, z);

g_NED = [0;0;9.81];
omegaE = 7.2921159e-5;
omega_ie_NED = omegaE * [cosd(lat);0;-sind(lat)];

%% ----- Task 2: body frame vectors from IMU ----------------------------
D = dlmread(imu_path);
if size(D,2) < 8
    error('Unexpected IMU format');
end
Nstatic = min(4000, size(D,1));
acc = D(1:Nstatic,6:8);
gyro = D(1:Nstatic,3:5);
if size(D,1) > 1
    dt = mean(diff(D(1:100,2)));
else
    dt = 1/400;
end
acc = acc ./ dt;
gyro = gyro ./ dt;
acc_mean = mean(acc,1);
gyro_mean = mean(gyro,1);
g_mag = norm(acc_mean);
if abs(g_mag - 9.81) > 0.5
    scale_factor = 9.81 / g_mag;
    D(:,6:8) = D(:,6:8) * scale_factor;
    acc = D(1:Nstatic,6:8) ./ dt;
    acc_mean = mean(acc,1);
end
acc_bias = acc_mean;
gyro_bias = gyro_mean;
g_body = -acc_bias';
omega_ie_body = gyro_bias';

%% ----- Task 3: attitude initialisation -------------------------------
[g_b_u, w_b_u, g_n_u, w_n_u] = normalise_vectors(g_body, omega_ie_body, g_NED, omega_ie_NED);

switch upper(method)
    case 'TRIAD'
        % classical TRIAD algorithm
        T1_b = g_b_u;
        T2_b = cross(g_b_u, w_b_u); T2_b = T2_b./norm(T2_b);
        T3_b = cross(T1_b, T2_b);
        T1_n = g_n_u;
        T2_n = cross(g_n_u, w_n_u); T2_n = T2_n./norm(T2_n);
        T3_n = cross(T1_n, T2_n);
        R_nb = [T1_n T2_n T3_n] * [T1_b T2_b T3_b]';
    case 'DAVENPORT'
        % Davenport q-method
        B = g_n_u*g_b_u' + w_n_u*w_b_u';
        S = B + B';
        sigma = trace(B);
        Z = [B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)];
        K = [sigma Z'; Z S - sigma*eye(3)];
        [V,Dq] = eig(K);
        [~,idx] = max(diag(Dq));
        q = V(:,idx);
        if q(1) < 0, q = -q; end
        R_nb = quat2rotm([q(1) -q(2:4)']);
    case 'SVD'
        % SVD alignment of two vector pairs
        B = g_n_u*g_b_u' + w_n_u*w_b_u';
        [U,~,V] = svd(B);
        M = diag([1 1 sign(det(U*V'))]);
        R_nb = U*M*V';
    otherwise
        error('Unknown method %s', method);
end

q_nb = rotm2quat(R_nb);  % quaternion form

%% ----- Task 4: simple IMU integration --------------------------------
acc_all = D(:,6:8) ./ dt;
gyro_all = D(:,3:5) ./ dt; %#ok<NASGU>
acc_all = acc_all - acc_bias;
gyro_all = gyro_all - gyro_bias; %#ok<NASGU>
N = size(D,1);
vel = zeros(N,3); pos = zeros(N,3);
for k = 2:N
    f_b = acc_all(k,:)';
    f_n = R_nb * f_b - g_NED;
    vel(k,:) = vel(k-1,:) + (f_n'*dt);
    pos(k,:) = pos(k-1,:) + vel(k-1,:)*dt + 0.5*f_n'*dt^2;
end

%% ----- Task 5: simple Kalman filter fusion with GNSS ------------------
pos_gnss_ecef = [T.X_ECEF_m T.Y_ECEF_m T.Z_ECEF_m];
vel_gnss_ecef = [T.VX_ECEF_mps T.VY_ECEF_mps T.VZ_ECEF_mps];
C = C_ECEF_to_NED(deg2rad(lat), deg2rad(lon));
pos_gnss = (C * (pos_gnss_ecef - [x y z])')';
vel_gnss = (C * vel_gnss_ecef')';

time_imu = D(:,2);
if isempty(time_imu)
    time_imu = (0:N-1)' * dt + T.Posix_Time(1);
end

gyro_all = D(:,3:5) ./ dt;
KF.x = [pos_gnss(1,:) vel_gnss(1,:) zeros(1,3)];
KF.P = eye(9);
Q = 1e-2 * eye(9);
Rk = 1e-1 * eye(6);
fused_pos(1,:) = KF.x(1:3);
fused_vel(1,:) = KF.x(4:6);
q_cur = q_nb; q_log = zeros(N,4); q_log(1,:) = q_cur;
win = 80; acc_win = []; gyro_win = []; zupt_count = 0;
for k = 2:N
    dt_k = time_imu(k) - time_imu(k-1);
    %% propagate quaternion using corrected gyro
    w_b = gyro_all(k,:)' - gyro_bias;
    dq = quat_from_rate(w_b, dt_k);
    q_cur = quat_multiply(q_cur, dq);
    q_cur = q_cur ./ norm(q_cur);
    q_log(k,:) = q_cur;

    %% Kalman time update
    A = eye(9);
    A(1:3,4:6) = eye(3)*dt_k;
    A(4:6,7:9) = eye(3)*dt_k;
    KF.x = (A * KF.x')';
    KF.P = A*KF.P*A' + Q;

    %% Measurement update with GNSS
    z = [pos_gnss(min(k,size(pos_gnss,1)),:) vel_gnss(min(k,size(vel_gnss,1)),:)];
    H = [eye(6) zeros(6,3)];
    K = KF.P*H'/(H*KF.P*H'+Rk);
    KF.x = (KF.x' + K*(z' - H*KF.x'))';
    KF.P = (eye(9)-K*H)*KF.P;

    fused_pos(k,:) = KF.x(1:3);
    fused_vel(k,:) = KF.x(4:6);

    %% ZUPT using rolling variance
    acc_win = [acc_win; acc_all(k,:) - accel_bias'];
    gyro_win = [gyro_win; gyro_all(k,:) - gyro_bias'];
    if size(acc_win,1) > win
        acc_win(1,:) = []; gyro_win(1,:) = [];
    end
    if size(acc_win,1) == win && is_static(acc_win, gyro_win)
        H_z = zeros(3,9); H_z(:,4:6) = eye(3);
        R_z = 1e-4 * eye(3);
        pred_v = KF.x(4:6)';
        S = H_z*KF.P*H_z' + R_z;
        Kz = KF.P*H_z'/S;
        KF.x = (KF.x' + Kz*(-pred_v))';
        KF.P = (eye(9)-Kz*H_z)*KF.P;
        zupt_count = zupt_count + 1;
    end
end

%% ----- Save results ----------------------------------------------------
[~,istem] = fileparts(imu_path); [~,gstem] = fileparts(gnss_path);
here = fileparts(mfilename('fullpath'));
resultsDir = fullfile(here, 'results');
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end
matfile = fullfile(resultsDir, sprintf('%s_%s_%s_final.mat', istem, gstem, method));

summary.q0 = q_nb;
summary.final_pos = norm(fused_pos(end,:) - pos_gnss(end,:));
save(matfile, 'fused_pos', 'fused_vel', 'summary');

fprintf('Saved %s\n', matfile);

% Simple overlay figure using fused results only
plot_overlay(imu_time, fused_pos, fused_vel, acc_log', gnss_time, pos_gnss, vel_gnss, gnss_accel_ned, imu_time, fused_pos, fused_vel, acc_log', 'NED', method, resultsDir);

end

%% Helper functions ------------------------------------------------------
function [lat, lon, alt] = ecef2geod(x,y,z)
a = 6378137.0; e2 = 6.69437999014e-3;
p = sqrt(x.^2 + y.^2);
theta = atan2(z*a, p*(1-e2));
lon = atan2(y,x);
lat = atan2(z + e2*a*sin(theta).^3./(1-e2), p - e2*a*cos(theta).^3);
N = a ./ sqrt(1-e2*sin(lat).^2);
alt = p./cos(lat) - N;
lat = rad2deg(lat); lon = rad2deg(lon);
end

function C = C_ECEF_to_NED(lat, lon)
sphi = sin(lat); cphi = cos(lat);
slam = sin(lon); clam = cos(lon);
C = [ -sphi.*clam, -sphi.*slam,  cphi;...
       -slam,       clam,       0;...
       -cphi.*clam, -cphi.*slam, -sphi];
end

function [g_b_u, w_b_u, g_n_u, w_n_u] = normalise_vectors(g_b,w_b,g_n,w_n)
    g_b_u = g_b./norm(g_b);
    w_b_u = w_b./norm(w_b);
    g_n_u = g_n./norm(g_n);
    w_n_u = w_n./norm(w_n);
end

function dq = quat_from_rate(w, dt)
    w_norm = norm(w);
    if w_norm > 1e-12
        axis = w / w_norm;
        angle = w_norm * dt;
        dq = [cos(angle/2); axis*sin(angle/2)];
    else
        dq = [1;0;0;0];
    end
end

function q_out = quat_multiply(q1, q2)
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
    q_out=[w1*w2 - x1*x2 - y1*y2 - z1*z2;
           w1*x2 + x1*w2 + y1*z2 - z1*y2;
           w1*y2 - x1*z2 + y1*w2 + z1*x2;
           w1*z2 + x1*y2 - y1*x2 + z1*w2];
end

function is_stat = is_static(acc, gyro)
    acc_thresh = 0.01; gyro_thresh = 1e-6;
    is_stat = all(var(acc,0,1) < acc_thresh) && all(var(gyro,0,1) < gyro_thresh);
end
