%% RUN_TRIAD_ONLY  Run all datasets using the TRIAD method (MATLAB pipeline)
% This script mirrors ``run_triad_only.py`` but executes the batch
% processing purely in MATLAB via ``run_all_datasets_matlab``.
% All figures and MAT files are written to ``results/`` and Task 6 is
% invoked automatically when matching ``STATE_X*.txt`` logs are present.
%
% Usage:
%   run_triad_only

% Simply forward to the MATLAB batch runner. This enumerates all datasets,
% runs Tasks 1--5 and triggers Task 6 when truth files exist.
run_all_datasets_matlab();

