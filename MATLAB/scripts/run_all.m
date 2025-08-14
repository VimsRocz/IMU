function run_all(gnss_file, imu_file, run_name, truth_file)
%RUN_ALL Execute Tasks 1-7
addpath(genpath('..'));

if nargin < 3 || isempty(run_name)
    run_name = infer_run_name(gnss_file, imu_file);
end

if nargin < 4 || isempty(truth_file)
    cand = fullfile('DATA', 'Truth', sprintf('STATE_%s.txt', run_name));
    if isfile(cand)
        truth_file = cand;
    else
        truth_file = resolve_truth_path();
    end
end

run_task1(gnss_file, imu_file, run_name);
run_task2(gnss_file, imu_file, run_name);
run_task3(gnss_file, imu_file, run_name);
run_task4(gnss_file, imu_file, run_name);

method = 'TRIAD';
Task_5(imu_file, gnss_file, method);

results_dir = get_matlab_results_dir();
[~, imu_name, ~]  = fileparts(imu_file);
[~, gnss_name, ~] = fileparts(gnss_file);
run_id = sprintf('%s_%s_%s', imu_name, gnss_name, method);
task5_file = fullfile(results_dir, sprintf('%s_task5_results.mat', run_id));

try
    Task_6(task5_file, imu_file, gnss_file, truth_file);
    Task_7();
catch ME
    warning('Task 6 failed: %s\nSkipping Task 7.', ME.message);
end
end

function name = infer_run_name(gnss_file, imu_file)
t = regexp(gnss_file, 'X\d+', 'match');
if ~isempty(t), name = t{1}; return; end
t = regexp(imu_file, 'X\d+', 'match');
if ~isempty(t), name = t{1}; else, name = 'RUN'; end
end
