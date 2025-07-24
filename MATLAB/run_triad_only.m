%% RUN_TRIAD_ONLY  Process all datasets using the TRIAD method only
% Mirrors ``run_triad_only.py`` by forwarding to ``run_all_datasets_matlab``.
% All figures and metrics are written under ``results/`` in the repository
% root.
%
% Usage:
%   run_triad_only
%
% See also: run_all_datasets_matlab, run_method_only

function run_triad_only
run_all_datasets_matlab('TRIAD');
end
