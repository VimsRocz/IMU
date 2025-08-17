%% RUN_METHOD_ONLY  Run all datasets with a chosen initialisation method
% Pure MATLAB implementation mirroring ``run_method_only.py``. The helper
% forwards the selected METHOD to ``run_all_datasets_matlab`` so no Python
% interpreter is required.
%
% Usage:
%   run_method_only                % defaults to 'TRIAD'
%   run_method_only('SVD')
%
% All results are written to the folder returned by ``get_results_dir``.

function run_method_only(method)
if nargin < 1 || isempty(method)
    method = 'TRIAD';
end

run_all_datasets_matlab(method);
end
