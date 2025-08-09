% RUN_TRIAD_ALL Run TRIAD Tasks 1–5 for all datasets sequentially.
%   Usage (MATLAB):
%       addpath('MATLAB'); run_triad_all
%   Usage (headless):
%       matlab -batch "run('MATLAB/run_triad_all.m')"

clc; close all; warning('off','all');
addpath('MATLAB');

datasets = {
    struct('id','X001','imu','IMU_X001.dat','gnss','GNSS_X001.csv')
    struct('id','X002','imu','IMU_X002.dat','gnss','GNSS_X002.csv')
    struct('id','X003','imu','IMU_X003.dat','gnss','GNSS_X002.csv')  % X003 shares GNSS X002
};

truth_file = 'STATE_IMU_X001.txt';  % common truth overlay

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
    cfg.plots = struct('popup_figures', false, 'save_pdf', true, 'save_png', true);

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
        % continue with next dataset
    end
end

fprintf('\n[TRIAD] All datasets finished in %.2f s\n', toc(overallStart));

