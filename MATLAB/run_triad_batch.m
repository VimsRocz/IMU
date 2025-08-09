function run_triad_batch()
% RUN_TRIAD_BATCH Headless TRIAD run with sane defaults and logging.
%   Designed for: matlab -batch "run('MATLAB/run_triad_batch.m')"

clc; close all; warning('off','all');
addpath('MATLAB');

cfg = struct();
cfg.dataset_id = 'X002';
cfg.method = 'TRIAD';
cfg.imu_file = 'IMU_X002.dat';
cfg.gnss_file = 'GNSS_X002.csv';
cfg.truth_file = 'STATE_IMU_X001.txt';
cfg.plots = struct('popup_figures', false, 'save_pdf', true, 'save_png', true);

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

