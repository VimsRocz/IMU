function run_triad_only(cfg)
% RUN_TRIAD_ONLY — Execute Tasks 1..7 for a dataset using the TRIAD method
% - MATLAB results go to <repo>/MATLAB/results (independent of Python's <repo>/results).
% - No hidden defaults inside tasks; everything comes from cfg.
% - Task 6 and 7 are MANDATORY (like Python). If truth is missing, we error out.

% --- ensure utils on path -------------------------------------------------
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'src','utils'));
addpath(fullfile(here,'src'));
if exist(fullfile(here,'src','utils','attitude'),'dir')
    addpath(fullfile(here,'src','utils','attitude'));
end
if exist(fullfile(here,'src','utils','frames'),'dir')
    addpath(fullfile(here,'src','utils','frames'));
end

% --- config ---------------------------------------------------------------
if nargin==0 || isempty(cfg)
    cfg = default_cfg();              % visible starting values; tasks get all inputs from cfg
    cfg.dataset_id = 'X002';
    cfg.method     = 'TRIAD';
    cfg.imu_file   = 'IMU_X002.dat';
    cfg.gnss_file  = 'GNSS_X002.csv';
    % IMPORTANT: Task 6/7 are mandatory. Point to the correct truth file for your dataset.
    % If your repo uses a different naming, update this value accordingly.
    cfg.truth_file = 'STATE_X001.txt';
end

% Resolve project paths and MATLAB-only results
cfg.paths = project_paths();                 % adds src/utils to path, returns root + matlab_results
mat_results = cfg.paths.matlab_results;      % <repo>/MATLAB/results
if ~exist(mat_results,'dir'), mkdir(mat_results); end

% Resolve absolute input paths
cfg.imu_path   = fullfile(cfg.paths.root, cfg.imu_file);
cfg.gnss_path  = fullfile(cfg.paths.root, cfg.gnss_file);
cfg.truth_path = fullfile(cfg.paths.root, cfg.truth_file);

% Required inputs must exist (Task 6/7 mandatory like Python)
mustExist(cfg.imu_path,   'IMU file');
mustExist(cfg.gnss_path,  'GNSS file');
mustExist(cfg.truth_path, 'Truth file (required for Task 6/7)');

run_id = sprintf('%s_%s_%s', erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method);
fprintf('>> %s\n', run_id);
fprintf('>> MATLAB results dir: %s\n', mat_results);

% Expected outputs by task (for assertions)
t1_mat = fullfile(mat_results, sprintf('Task1_init_%s_%s_%s.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t2_mat = fullfile(mat_results, sprintf('Task2_body_%s_%s_%s.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t3_mat = fullfile(mat_results, sprintf('IMU_%s_GNSS_%s_%s_task3_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t4_mat = fullfile(mat_results, sprintf('IMU_%s_GNSS_%s_%s_task4_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));
t5_mat = fullfile(mat_results, sprintf('%s_%s_%s_task5_results.mat', ...
    erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));

% Representative Task 6/7 artifacts to assert (adjust if your Task_6/Task_7 filenames differ)
t6_overlay_ned_pdf = fullfile(cfg.paths.root, 'results', ...
    sprintf('IMU_%s_GNSS_%s_%s_task6_overlay_state_NED.pdf', ...
        erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));   % Task_6 saves under Python-style results/
% If your MATLAB Task_6 writes under MATLAB/results/<run_id>, add another check there too if needed.

t7_any_pdf = fullfile(cfg.paths.root, 'results', ...
    sprintf('IMU_%s_GNSS_%s_%s_task7_3_residuals_position_velocity.pdf', ...
        erase(cfg.imu_file,'.dat'), erase(cfg.gnss_file,'.csv'), cfg.method));

% --- Run Tasks 1..5 ------------------------------------------------------
Task_1(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t1_mat, 'Task 1 results');

Task_2(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t2_mat, 'Task 2 results');

Task_3(cfg.imu_path, cfg.gnss_path, cfg.method);   % consumes Task1/2 mats; will re-create if missing
assertFile(t3_mat, 'Task 3 results');

Task_4(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t4_mat, 'Task 4 results');

Task_5(cfg.imu_path, cfg.gnss_path, cfg.method);
assertFile(t5_mat, 'Task 5 results');

% --- Run Tasks 6..7 (MANDATORY; fail if anything missing) ----------------
fprintf('>> Running Task 6 (truth overlay/validation)...\n');
Task_6(t5_mat, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
assertFile(t6_overlay_ned_pdf, 'Task 6 overlay (NED)');

fprintf('>> Running Task 7 (residuals & summary)...\n');
Task_7();   % expects to read mats from its standard locations
assertFile(t7_any_pdf, 'Task 7 residuals plot');

% --- Manifest for traceability ------------------------------------------
manifest = struct();
manifest.when        = datestr(now, 31);
manifest.run_id      = run_id;
manifest.cfg         = cfg;
manifest.outputs     = struct('task1',t1_mat,'task2',t2_mat,'task3',t3_mat,'task4',t4_mat,'task5',t5_mat, ...
                              'task6_overlay_ned_pdf', t6_overlay_ned_pdf, ...
                              'task7_any_pdf', t7_any_pdf);
save(fullfile(mat_results, sprintf('%s_manifest.mat', run_id)), '-struct','manifest','-v7');

fprintf('>> Tasks 1..7 complete. MATLAB outputs in: %s\n', mat_results);
drawnow;

end % function

% --- helpers --------------------------------------------------------------
function mustExist(p, label)
if ~isfile(p)
    error('%s not found: %s', label, p);
end
end

function assertFile(p, label)
if ~isfile(p)
    error('%s was not created: %s', label, p);
else
    fprintf('   OK: %s\n', p);
end
end

