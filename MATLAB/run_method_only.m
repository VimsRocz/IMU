%% RUN_METHOD_ONLY  Run all datasets with a chosen initialisation method
% This now mirrors ``run_method_only.py`` using the pure MATLAB pipeline.
% The selected METHOD is forwarded to ``run_all_datasets_matlab`` so no
% external Python interpreter is required.
%
% Usage:
%   run_method_only                % defaults to 'TRIAD'
%   run_method_only('SVD')
%
% All results are written to the 'results' folder.

function run_method_only(method)
if nargin < 1 || isempty(method)
    method = 'TRIAD';
end

run_all_datasets_matlab(method);
end
