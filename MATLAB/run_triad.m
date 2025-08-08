function S = run_triad(cfg)
%RUN_TRIAD  Structured runner that executes Tasks 1..7 with timing & checks.

arguments
    cfg struct
end

% --- Paths & deps
addpath(cfg.paths.src_utils);      % src/utils (shared constants, frames, etc.)
addpath(cfg.paths.matlab_utils);   % MATLAB/utils (triad_matrix, etc.)
ensure_dir(cfg.paths.results);

% --- Inputs must exist (zero hidden defaults)
require_files({cfg.imu_path, cfg.gnss_path});

% --- Deterministic plots & RNG
rng(0);
set(0,'DefaultFigureVisible', ternary(cfg.plots.popup_figures,'on','off'));

% --- Run ID + standard filenames
[~,imu_name,~]  = fileparts(cfg.imu_path);
[~,gnss_name,~] = fileparts(cfg.gnss_path);
run_id = sprintf('%s_%s_%s', imu_name, gnss_name, cfg.method);
S = struct('run_id',run_id, 't',struct());

% --- Task list (function handles keep code DRY)
tasks = { ...
    @( ) Task_1(cfg.imu_path, cfg.gnss_path, cfg.method), ...
    @( ) Task_2(cfg.imu_path, cfg.gnss_path, cfg.method), ...
    @( ) Task_3(cfg.imu_path, cfg.gnss_path, cfg.method), ...
    @( ) Task_4(cfg.imu_path, cfg.gnss_path, cfg.method), ...
    @( ) Task_5(cfg.imu_path, cfg.gnss_path, cfg.method)  ...
};

fprintf('\u25b6 %s\n', run_id);
for k = 1:numel(tasks)
    t0 = tic;
    try
        tasks{k}();
    catch ME
        fprintf(2,'[ERROR] Task %d failed: %s\n', k, ME.message);
        rethrow(ME);
    end
    S.t.(sprintf('Task%d',k)) = toc(t0);
end

% --- Tasks 6 & 7 only if we have both: Task 5 results + truth
task5_file = fullfile(cfg.paths.results, sprintf('%s_task5_results.mat', run_id));
has_truth  = ~isempty(cfg.truth_path) && isfile(cfg.truth_path);
if isfile(task5_file) && has_truth
    fprintf('--- Running Task 6: Truth Overlay/Validation ---\n');
    t0 = tic;
    Task_6(task5_file, cfg.imu_path, cfg.gnss_path, cfg.truth_path);
    S.t.Task6 = toc(t0);

    fprintf('--- Running Task 7: Residuals & Summary ---\n');
    t0 = tic;
    Task_7();              % uses saved Task 5 artifacts by design
    S.t.Task7 = toc(t0);

    out_dir = fullfile(cfg.paths.results, run_id);
    fprintf('Task 6/7 plots saved under: %s\n', out_dir);
else
    warning('Task 6/7 skipped: missing Task 5 results or truth file.');
end

% --- Save a compact run summary
S.cfg = cfg; %#ok<STRNU>
save(fullfile(cfg.paths.results, [run_id '_driver_summary.mat']), 'S');

end
