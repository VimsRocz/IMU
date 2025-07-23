%% RUN_TRIAD_ONLY  Run all datasets using the TRIAD method (MATLAB pipeline)
% This script mirrors ``run_triad_only.py`` but executes the batch
% processing purely in MATLAB via ``run_all_datasets_matlab``.
% All figures and MAT files are written to ``results/``. When matching
% ``STATE_X*.txt`` logs are present Tasks 6 and 7 (truth overlay and
% residual evaluation) are invoked automatically.
%
% Usage:
%   run_triad_only

% Simply forward to the MATLAB batch runner. This enumerates all datasets,
% runs Tasks 1--5 and triggers Tasks 6 and 7 when truth files exist.
run_all_datasets_matlab('TRIAD');

