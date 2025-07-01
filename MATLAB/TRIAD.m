function results = TRIAD(imu_path, gnss_path, verbose)
close all force hidden;
if nargin < 3
    verbose = false;
end
if ~verbose
    diary off;
end
%TRIAD  Simple MATLAB implementation of the TRIAD pipeline.
%   TRIAD(IMU_PATH, GNSS_PATH) processes a pair of IMU and GNSS files
%   and saves basic results to results/<tag>_triad.mat.  The
%   implementation mirrors the Python script GNSS_IMU_Fusion.py but in a
%   greatly simplified form.  Only the key steps of the attitude
%   initialisation, IMU integration and Kalman filter fusion are
%   implemented.  The file names are resolved using GET_DATA_FILE so the
%   sample logs bundled with the repository can be referenced by name.
%
%   Example:
%       TRIAD('IMU_X001.dat','GNSS_X001.csv');
%
%   See docs/TRIAD_Task*.md for a detailed description of the algorithm.

% Detect accidental execution as a script. When run with the `run` command
% the file is evaluated line-by-line and `nargin` cannot be called,
% producing the cryptic "You can only call nargin/nargout from within a
% MATLAB function" error.  Use `dbstack` to detect that situation and emit
% a clearer message before reaching the nargin check below.
if numel(dbstack) <= 1
    error(['TRIAD must be called as a function with two file names. Example:\n', ...
           '    TRIAD(''IMU_X001.dat'', ''GNSS_X001.csv'');']);
end

if nargin < 1 || isempty(imu_path)
    imu_path = 'IMU_X001.dat';
end
if nargin < 2 || isempty(gnss_path)
    gnss_path = 'GNSS_X001.csv';
end
if verbose && (nargin < 1 || isempty(imu_path) || nargin < 2 || isempty(gnss_path))
    fprintf('[INFO] Using default files: %s, %s\n', imu_path, gnss_path);
end
here = fileparts(mfilename('fullpath'));
resultsDir = fullfile(here, 'results');
if verbose && ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

% Locate data files bundled with the repository
imu_path = get_data_file(imu_path);
gnss_path = get_data_file(gnss_path);

%% ----- Task 1: Reference vectors in NED frame -------------------------
% Load first valid GNSS ECEF coordinate and convert to geodetic
T = readtable(gnss_path);
valid = find(T.X_ECEF_m ~= 0 | T.Y_ECEF_m ~= 0 | T.Z_ECEF_m ~= 0, 1);
if isempty(valid)
    error('No valid GNSS rows in %s', gnss_path);
end
x = T.X_ECEF_m(valid); y = T.Y_ECEF_m(valid); z = T.Z_ECEF_m(valid);
[lat, lon, ~] = ecef2geod(x, y, z);
% reference vectors
g_NED = [0; 0; 9.81];
omegaE = 7.2921159e-5;
omega_ie_NED = omegaE * [cosd(lat); 0; -sind(lat)];

%% ----- Task 2: Body-frame vectors from IMU ----------------------------
D = dlmread(imu_path);
if size(D,2) < 8
    error('Unexpected IMU format');
end
% assume column 2 is time, 3:5 gyro increments, 6:8 accel increments
% Use first 200 samples for static estimate
Nstatic = min(200, size(D,1));
acc = D(1:Nstatic,6:8);   % m/s increments
gyro = D(1:Nstatic,3:5);  % rad increments
% derive sampling period from time column
if size(D,1) > 1
    dt = mean(diff(D(1:100,2)));
else
    dt = 1/400;
end
acc = acc ./ dt;  % to m/s^2
gyro = gyro ./ dt; % to rad/s
acc_mean = mean(acc,1);  % expected -gravity
gyro_mean = mean(gyro,1);
% body vectors
g_body = -acc_mean';
omega_ie_body = gyro_mean';

%% ----- Task 3: TRIAD attitude solution --------------------------------
[g_body_u, omega_b_u, g_ref_u, omega_ref_u] = normalise_vectors(g_body, omega_ie_body, g_NED, omega_ie_NED);
% triad bases
T1_b = g_body_u;
T2_b = cross(g_body_u, omega_b_u); T2_b = T2_b./norm(T2_b);
T3_b = cross(T1_b, T2_b);
T1_n = g_ref_u;
T2_n = cross(g_ref_u, omega_ref_u); T2_n = T2_n./norm(T2_n);
T3_n = cross(T1_n, T2_n);
R_nb = [T1_n T2_n T3_n] * [T1_b T2_b T3_b]';
q = rotm2quat(R_nb);  % MATLAB aerospace toolbox function

%% ----- Task 4: IMU integration (simple) --------------------------------
acc_all = D(:,6:8) ./ dt;  % m/s^2
N = size(D,1);
vel = zeros(N,3); pos = zeros(N,3);
for k = 2:N
    f_b = acc_all(k,:)';
    f_n = R_nb * f_b - g_NED;
    vel(k,:) = vel(k-1,:) + (f_n' * dt);
    pos(k,:) = pos(k-1,:) + vel(k-1,:) * dt + 0.5 * f_n' * dt^2;
end

%% ----- Task 5: dummy Kalman filter ------------------------------------
% Simplified constant-position Kalman filter just for demonstration
KF.x = [pos(1,:) vel(1,:)];
KF.P = eye(6);
Q = 1e-3 * eye(6);
R = 1e-2 * eye(6);
for k = 2:N
    % predict
    A = [eye(3) eye(3)*dt; zeros(3) eye(3)];
    KF.x = A * KF.x';
    KF.P = A*KF.P*A' + Q;
    % update with pseudo-measurement of position & velocity from integration
    z = [pos(k,:) vel(k,:)];
    H = eye(6);
    K = KF.P*H'/(H*KF.P*H'+R);
    KF.x = KF.x + K*(z' - H*KF.x);
    KF.P = (eye(6)-K*H)*KF.P;
end

%% ----- Save results ----------------------------------------------------
[~,istem] = fileparts(imu_path); [~,gstem] = fileparts(gnss_path);
matfile = fullfile(resultsDir, sprintf('%s_%s_TRIAD_output.mat', istem, gstem));
summary.q0 = q;
summary.final_pos = norm(pos(end,:));

results.pos = pos;
results.vel = vel;
results.q = q;
results.summary = summary;

if verbose
    save(matfile, 'pos', 'vel', 'q', 'summary');
    fprintf('Saved %s\n', matfile);
end

end

%% Helper functions
function [lat, lon, alt] = ecef2geod(x, y, z)
a = 6378137.0; e2 = 6.69437999014e-3;
p = sqrt(x.^2 + y.^2);
theta = atan2(z*a, p*(1-e2));
lon = atan2(y, x);
lat = atan2(z + e2*a*sin(theta).^3./(1-e2), p - e2*a*cos(theta).^3);
N = a ./ sqrt(1-e2*sin(lat).^2);
alt = p./cos(lat) - N;
lat = rad2deg(lat); lon = rad2deg(lon);
end

function C = C_ECEF_to_NED(lat, lon)
sphi = sin(lat); cphi = cos(lat);
slam = sin(lon); clam = cos(lon);
C = [ -sphi.*clam, -sphi.*slam,  cphi;
       -slam,      clam,       0;
       -cphi.*clam, -cphi.*slam, -sphi];
end

function [g_b_u, w_b_u, g_n_u, w_n_u] = normalise_vectors(g_b, w_b, g_n, w_n)
    g_b_u = g_b./norm(g_b);
    w_b_u = w_b./norm(w_b);
    g_n_u = g_n./norm(g_n);
    w_n_u = w_n./norm(w_n);
end
