
%
%   The script assumes the dataset files ``IMU_X001.dat`` and
%   ``GNSS_X001.csv`` are located in the current working directory.
%
% Sections:
%   1. DATA LOADING
%   2. REFERENCE VECTORS
%   3. TRIAD METHOD
%   4. OUTPUT
%
% Project structure and naming follow the repository guidelines so that
% plots, logs and results match between MATLAB and Python.

%% DATA LOADING
imu_file  = 'IMU_X001.dat';
gnss_file = 'GNSS_X001.csv';

imu_data = readmatrix(imu_file);
time_s   = imu_data(:,2);
gyro_inc = imu_data(:,3:5);
acc_inc  = imu_data(:,6:8);

if numel(time_s) > 1
    dt_imu = mean(diff(time_s(1:min(100,end))));
else
    dt_imu = 1/400; % default 400 Hz
end

gyro = gyro_inc / dt_imu; % rad/s
acc  = acc_inc  / dt_imu; % m/s^2

N = min(300, size(acc,1));
static_acc  = mean(acc(1:N,:), 1)';
static_gyro = mean(gyro(1:N,:),1)';
fprintf('Using first %d IMU samples for static interval.\n', N);

%% REFERENCE VECTORS
Tgnss = readtable(gnss_file);
idx = find(Tgnss.X_ECEF_m ~= 0 | Tgnss.Y_ECEF_m ~= 0 | Tgnss.Z_ECEF_m ~= 0, 1, 'first');
[x_ecef,y_ecef,z_ecef] = deal(Tgnss.X_ECEF_m(idx), Tgnss.Y_ECEF_m(idx), Tgnss.Z_ECEF_m(idx));
[lat_deg, lon_deg, h_m] = ecef2geodetic(x_ecef, y_ecef, z_ecef);
lat_rad = deg2rad(lat_deg);

g_NED = [0;0;constants.GRAVITY];
omega_ie_NED = constants.EARTH_RATE * [cos(lat_rad); 0; -sin(lat_rad)];

g_body_raw = -static_acc;
g_body = constants.GRAVITY * g_body_raw / norm(g_body_raw);
omega_ie_body = static_gyro;

%% TRIAD METHOD
M_body = triad_basis(g_body, omega_ie_body);
M_ned  = triad_basis(g_NED,  omega_ie_NED);
C_B_N  = M_ned * M_body';
q_bn   = rot_to_quaternion(C_B_N);
eul_rad = quat_to_euler(q_bn);
eul_deg = rad2deg(eul_rad);

fprintf('\nRotation matrix C_{B}^{N}:\n');
disp(C_B_N);
fprintf('Euler angles [deg]: roll=%.2f pitch=%.2f yaw=%.2f\n', ...
    eul_deg(1), eul_deg(2), eul_deg(3));

%% OUTPUT
out_dir = fullfile('output_matlab', 'IMU_X001_GNSS_X001_TRIAD');
if ~exist(out_dir,'dir'); mkdir(out_dir); end

save(fullfile(out_dir,'initial_attitude.mat'), 'C_B_N', 'q_bn', 'eul_deg', ...
    'lat_deg', 'lon_deg', 'h_m');
writematrix(C_B_N, fullfile(out_dir,'C_B_N.csv'));
writematrix(eul_deg, fullfile(out_dir,'euler_angles_deg.csv'));

fprintf('Results saved to %s\n', out_dir);
end

%% -------------------------------------------------------------------------
% Helper functions
% -------------------------------------------------------------------------
function M = triad_basis(v1, v2)
    t1 = v1 / norm(v1);
    t2_temp = cross(t1, v2);
    if norm(t2_temp) < 1e-10
        if abs(t1(1)) < abs(t1(2)), tmp = [1;0;0]; else, tmp = [0;1;0]; end
        t2_temp = cross(t1, tmp);
    end
    t2 = t2_temp / norm(t2_temp);
    t3 = cross(t1, t2);
    M = [t1, t2, t3];
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

function eul = quat_to_euler(q)
    R = quat_to_rot(q);
    phi = atan2(R(3,2), R(3,3));
    theta = -asin(R(3,1));
    psi = atan2(R(2,1), R(1,1));
    eul = [phi; theta; psi];
end

function R = quat_to_rot(q)
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1-2*(qy^2+qz^2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy);
         2*(qx*qy + qw*qz), 1-2*(qx^2 + qz^2), 2*(qy*qz - qw*qx);
         2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1-2*(qx^2 + qy^2)];
end
