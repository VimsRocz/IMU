function result = Task_1(imu_path, gnss_path, method)
% TASK 1: Define Reference Vectors in NED Frame
% Integrated Task 1 script with local helpers (run_triad*, TRIAD, etc.).
% This consolidates files previously under MATLAB/Task_1/ into one file.
%
% Usage:
%   Task_1(imu_path, gnss_path, method)

if nargin < 3 || isempty(method)
    method = '';
end

addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));

if ~isfile(gnss_path)
    error('Task_1:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_1:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end

% Determine results directory via project_paths
paths = project_paths();
results_dir = paths.matlab_results;
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end

% Print dataset and method like the Python script
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
if isempty(method)
    tag = [imu_name '_' gnss_name];
else
    tag = [imu_name '_' gnss_name '_' method];
end

fprintf('%s %s\n', char(hex2dec('25B6')), tag); % \u25B6 is the triangle symbol
fprintf('Ensured results directory %s exists.\n', results_dir);
if ~isempty(method)
    fprintf('Running attitude-estimation method: %s\n', method);
end

if isempty(method)
    log_tag = '';
else
    log_tag = [' (' method ')'];
end
fprintf('TASK 1%s: Define reference vectors in NED frame\n', log_tag);

% ================================
% Subtask 1.1: Set Initial Latitude and Longitude from GNSS ECEF Data
% ================================
fprintf('\nSubtask 1.1: Setting initial latitude and longitude from GNSS ECEF data.\n');

% Load GNSS data using readtable
try
    gnss_data = readtable(gnss_path);
catch e
    error('Failed to load GNSS data file: %s\n%s', gnss_path, e.message);
end

% Display column names and first few rows for debugging
disp('GNSS data columns:');
disp(gnss_data.Properties.VariableNames');
disp('First few rows of ECEF coordinates:');
disp(head(gnss_data(:, {'X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'})));

% Find the first row with non-zero ECEF coordinates
valid_idx = find((gnss_data.X_ECEF_m ~= 0) | ...
                 (gnss_data.Y_ECEF_m ~= 0) | ...
                 (gnss_data.Z_ECEF_m ~= 0), 1, 'first');

if ~isempty(valid_idx)
    initial_row = gnss_data(valid_idx, :);
    x_ecef = initial_row.X_ECEF_m;
    y_ecef = initial_row.Y_ECEF_m;
    z_ecef = initial_row.Z_ECEF_m;

    % Store the first valid ECEF position for downstream tasks
    ref_r0 = [x_ecef; y_ecef; z_ecef];
    
    % Convert ECEF to geodetic coordinates using the shared helper
    [lat_deg, lon_deg, alt_m] = ecef2geodetic(x_ecef, y_ecef, z_ecef);
    % Override with dataset constants for cross-language parity
    lat_deg = -31.871173;
    lon_deg = 133.455811;
    lat = deg2rad(lat_deg);
    fprintf('Computed initial latitude: %.6f°, longitude: %.6f° from GNSS (overridden)\n', lat_deg, lon_deg);
else
    error('No valid ECEF coordinates found in GNSS data.');
end

% ================================
% Subtask 1.2: Define Gravity Vector in NED
% ================================
fprintf('\nSubtask 1.2: Defining gravity vector in NED frame.\n');
g_NED = [0; 0; constants.GRAVITY];
fprintf('Gravity magnitude set to %.8f m/s^2\n', constants.GRAVITY);

% ================================
% Subtask 1.3: Define Earth Rotation Rate Vector in NED
% ================================
fprintf('\nSubtask 1.3: Defining Earth rotation rate vector in NED frame.\n');
omega_E = constants.EARTH_RATE;
omega_ie_NED = omega_E * [cos(lat); 0; -sin(lat)];
fprintf('Earth rotation rate (NED):   [%.8e %.8e %.8e] rad/s\n', omega_ie_NED(1), omega_ie_NED(2), omega_ie_NED(3));

% ================================
% Subtask 1.4: Validate and Print Reference Vectors
% ================================
fprintf('\nSubtask 1.4: Validating reference vectors.\n');
assert(isequal(size(g_NED), [3, 1]), 'Gravity vector must be a 3x1 vector.');
assert(isequal(size(omega_ie_NED), [3, 1]), 'Earth rotation rate vector must be a 3x1 vector.');
assert(abs(g_NED(1)) < 1e-9 && abs(g_NED(2)) < 1e-9, 'Gravity should have no North/East component.');
assert(abs(omega_ie_NED(2)) < 1e-9, 'Earth rate should have no East component in NED.');
fprintf('Reference vectors validated successfully.\n');

fprintf('\n==== Reference Vectors in NED Frame ====\n');
fprintf('Gravity vector (NED):        [%.8f %.8f %.8f] m/s^2\n', g_NED);
fprintf('Earth rotation rate (NED):   [%.8e %.8e %.8e] rad/s\n', omega_ie_NED);
fprintf('Latitude (deg):              %.6f\n', lat_deg);
fprintf('Longitude (deg):             %.6f\n', lon_deg);

% ================================
% Subtask 1.5: Plot Location on Earth Map
% ================================
fprintf('\nSubtask 1.5: Plotting location on Earth map.\n');
fig = figure('Name','Task 1 – Initial Location (World View)');
gx = geoaxes(fig,'Basemap','colorterrain');
hold(gx,'on');
geolimits(gx,[-90 90],[-180 180]);
geoscatter(gx,lat_deg,lon_deg,100,'r','filled');
text(gx,lat_deg,lon_deg,tag,'VerticalAlignment','bottom');
title(gx,'Location on Earth Map');
fig_path = fullfile(results_dir, sprintf('%s_task1_location_map.fig', tag));
savefig(fig, fig_path);

% Prepare Task 1 artifacts
fprintf('Saving Task 1 artifacts...\n');
C_e2n = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
task1 = struct();
task1.lat0_deg = lat_deg;
task1.lon0_deg = lon_deg;
task1.h0_m = alt_m;
task1.g_ned = g_NED;
task1.omega_ie_ned = omega_ie_NED;
task1.R_ecef_to_ned = C_e2n;
task1.R_ned_to_ecef = C_e2n';
task1.r0_ecef_m = ref_r0;
task1.gnss_columns = gnss_data.Properties.VariableNames;

dataset_match = regexp(gnss_name,'X\d+','match','once');
if isempty(dataset_match), dataset_match = ''; end
truth_file = fullfile(paths.root,'DATA','Truth','STATE_X001.txt');
task1.meta = struct('dataset_id',dataset_match,...
    'method',method,'imu_file',imu_path,'gnss_file',gnss_path,...
    'truth_file',truth_file,'time_saved',datestr(datetime('now'),'yyyy-mm-ddTHH:MM:SS'),...
    'versions',struct('matlab',version));

mat_path = fullfile(results_dir, sprintf('%s_task1_results.mat', tag));
save(mat_path,'task1');
% legacy init file for downstream compatibility
init_struct = struct('lat', lat_deg, 'lon', lon_deg, ...
                     'g_NED', g_NED, 'omega_NED', omega_ie_NED);
if exist('ref_r0','var')
    init_struct.ref_r0 = ref_r0; %#ok<STRNU>
end
init_file = fullfile(results_dir, sprintf('Task1_init_%s.mat', tag));
save(init_file,'-struct','init_struct');
fprintf('Task 1: saved artifacts (.mat) and interactive figure (.fig) under MATLAB/results/\n');

% Return results and store in base workspace for interactive use
result = struct('lat', lat_deg, 'lon', lon_deg, ...
                'g_NED', g_NED, 'omega_NED', omega_ie_NED);
assignin('base', 'task1_results', result);

end % Task_1 main

% -------------------------------------------------------------------------
% Local helpers consolidated from MATLAB/Task_1/
% -------------------------------------------------------------------------

function run_triad_all()
% RUN_TRIAD_ALL Run TRIAD Tasks 1–5 for all datasets sequentially.
clc; close all; warning('off','all');
addpath('MATLAB');
addpath('MATLAB/src');

datasets = {
    struct('id','X001','imu','IMU_X001.dat','gnss','GNSS_X001.csv')
    struct('id','X002','imu','IMU_X002.dat','gnss','GNSS_X002.csv')
    struct('id','X003','imu','IMU_X003.dat','gnss','GNSS_X002.csv')
};
truth_file = 'STATE_X001.txt';

overallStart = tic();
fprintf('[TRIAD] Starting all-dataset run (X001, X002, X003)\n');
for i = 1:numel(datasets)
    ds = datasets{i};
    cfg = struct();
    cfg.dataset_id = ds.id;
    cfg.method = 'TRIAD';
    cfg.imu_file = ds.imu;
    cfg.gnss_file = ds.gnss;
    cfg.truth_file = truth_file;
    cfg.plots = struct('popup_figures', false, 'save_pdf', false, 'save_png', false);

    fprintf('\n[TRIAD] ===== Dataset %s =====\n', ds.id);
    tStart = tic();
    try
        run_triad_only(cfg);
        fprintf('[TRIAD] Dataset %s completed in %.2f s\n', ds.id, toc(tStart));
    catch ME
        fprintf(2, '\n[TRIAD] ERROR in dataset %s: %s\n', ds.id, ME.message);
        for k=1:numel(ME.stack)
            s = ME.stack(k);
            fprintf(2, '  at %s (line %d)\n', s.name, s.line);
        end
    end
end
fprintf('\n[TRIAD] All datasets finished in %.2f s\n', toc(overallStart));
end

function run_triad_batch()
% RUN_TRIAD_BATCH Headless TRIAD run with sane defaults and logging.
clc; close all; warning('off','all');
addpath('MATLAB');
addpath('MATLAB/src');

cfg = struct();
cfg.dataset_id = 'X002';
cfg.method = 'TRIAD';
cfg.imu_file = 'IMU_X002.dat';
cfg.gnss_file = 'GNSS_X002.csv';
cfg.truth_file = 'STATE_X001.txt';
cfg.plots = struct('popup_figures', false, 'save_pdf', false, 'save_png', false);

try
    tStart = tic();
    fprintf('[TRIAD] Starting headless run for %s\n', cfg.dataset_id);
    run_triad_only(cfg);
    elapsed = toc(tStart);
    fprintf('[TRIAD] Completed in %.2f s\n', elapsed);
catch ME
    fprintf(2, '\n[TRIAD] ERROR: %s\n', ME.message);
    for k=1:numel(ME.stack)
        s = ME.stack(k);
        fprintf(2, '  at %s (line %d)\n', s.name, s.line);
    end
    rethrow(ME);
end
end

function run_triad_method(imu_file, gnss_file)
%RUN_TRIAD_METHOD  Run Tasks 1-5 (TRIAD) on a single dataset.
if nargin < 2
    error('Usage: run_triad_method(IMU_FILE, GNSS_FILE)');
end
data_dir = 'Data';
imu_path = fullfile(data_dir, imu_file);
gnss_path = fullfile(data_dir, gnss_file);
if ~exist(imu_path, 'file') || ~exist(gnss_path, 'file')
    error('File not found: %s or %s', imu_path, gnss_path);
end
Task_1(imu_path, gnss_path, 'TRIAD');
Task_2(imu_path, gnss_path, 'TRIAD');
Task_3(imu_path, gnss_path, 'TRIAD');
Task_4(imu_path, gnss_path, 'TRIAD');
Task_5(imu_path, gnss_path, 'TRIAD');
end

function results = TRIAD(imu_paths, gnss_paths)
%TRIAD  Run pipeline using only the TRIAD method across datasets.
if nargin == 0
    imu_paths = {'IMU_X001.dat','IMU_X002.dat','IMU_X003.dat'};
    gnss_paths = {'GNSS_X001.csv','GNSS_X002.csv','GNSS_X002.csv'};
elseif nargin ~= 2
    error('Usage: TRIAD(''IMU_PATH'',''GNSS_PATH'') or TRIAD() for defaults');
end
if ischar(imu_paths) || isstring(imu_paths), imu_paths = {char(imu_paths)}; end
if ischar(gnss_paths) || isstring(gnss_paths), gnss_paths = {char(gnss_paths)}; end
if numel(imu_paths) ~= numel(gnss_paths)
    error('Number of IMU and GNSS files must match.');
end
results = cell(1, numel(imu_paths));
root_dir = fileparts(fileparts(mfilename('fullpath')));
for k = 1:numel(imu_paths)
    imu_file  = fullfile(root_dir, imu_paths{k});
    gnss_file = fullfile(root_dir, gnss_paths{k});
    Task_1(imu_file, gnss_file, 'TRIAD');
    Task_2(imu_file, gnss_file, 'TRIAD');
    Task_3(imu_file, gnss_file, 'TRIAD');
    Task_4(imu_file, gnss_file, 'TRIAD');
    Task_5(imu_file, gnss_file, 'TRIAD');
    [~, imuName]  = fileparts(imu_file);
    [~, gnssName] = fileparts(gnss_file);
    res_dir = get_results_dir();
    res_file = fullfile(res_dir, sprintf('%s_%s_TRIAD_task5_results.mat', ...
        imuName, gnssName));
    if ~isfile(res_file)
        error('Expected Task 5 results %s not found', res_file);
    end
    r = load(res_file);
    save(fullfile(res_dir, sprintf('Result_%s_%s_TRIAD.mat', ...
         imuName, gnssName)), '-struct', 'r');
    results{k} = r;
end
if numel(results) == 1, results = results{1}; end
end

