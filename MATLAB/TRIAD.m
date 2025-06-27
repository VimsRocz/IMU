function TRIAD(imuFile, gnssFile, varargin)
%TRIAD  Simple MATLAB implementation of the TRIAD pipeline.
%   TRIAD(IMUFILE, GNSSFILE) processes a pair of IMU and GNSS files
%   and saves basic results to results/<tag>_triad.mat.  The
%   implementation mirrors the Python script GNSS_IMU_Fusion.py but in a
%   greatly simplified form.  Only the key steps of the attitude
%   initialisation, IMU integration and Kalman filter fusion are
%   implemented.
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

if nargin == 0
    imuFile = 'IMU_X001.dat';
    gnssFile = 'GNSS_X001.csv';
    fprintf('[INFO] No files provided. Using defaults: %s, %s\n', imuFile, gnssFile);
elseif nargin ~= 2 && nargin ~= 3
    error('Usage: TRIAD(''IMUFILE'',''GNSSFILE'') or TRIAD() for defaults');
end

if isempty(varargin)
    resultsDir = 'results';
else
    resultsDir = varargin{1};
end
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

% ensure helper functions are available
helperPath = fullfile(fileparts(mfilename('fullpath')), '..', 'IMU_MATLAB');
if exist(fullfile(helperPath, 'ecef_to_geodetic.m'), 'file')
    addpath(helperPath);
end

%% ----- Task 1: Reference vectors in NED frame -------------------------
% Load first valid GNSS ECEF coordinate and convert to geodetic
T = readtable(gnssFile);
valid = find(T.X_ECEF_m ~= 0 | T.Y_ECEF_m ~= 0 | T.Z_ECEF_m ~= 0, 1);
if isempty(valid)
    error('No valid GNSS rows in %s', gnssFile);
end
x = T.X_ECEF_m(valid); y = T.Y_ECEF_m(valid); z = T.Z_ECEF_m(valid);
[lat, lon, ~] = ecef_to_geodetic(x, y, z);
% reference vectors
g_NED = [0; 0; 9.81];
omegaE = 7.2921159e-5;
omega_ie_NED = omegaE * [cosd(lat); 0; -sind(lat)];

%% ----- Task 2: Body-frame vectors from IMU ----------------------------
D = dlmread(imuFile);
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
[~,istem] = fileparts(imuFile); [~,gstem] = fileparts(gnssFile);
matfile = fullfile(resultsDir, sprintf('%s_%s_TRIAD_output.mat', istem, gstem));
summary.q0 = q;
summary.final_pos = norm(pos(end,:));
save(matfile, 'pos', 'vel', 'q', 'summary');
fprintf('Saved %s\n', matfile);

end

%% Helper functions

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
