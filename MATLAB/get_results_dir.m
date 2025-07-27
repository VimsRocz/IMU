function dir = get_results_dir()
%GET_RESULTS_DIR Return path to the results directory.
%   DIR = GET_RESULTS_DIR() returns the path to the top-level 'results'
%   folder within the repository. MATLAB scripts use this helper so that
%   logs and figures are stored next to the Python outputs.
%
%   Example:
%       out = fullfile(get_results_dir(), 'example.mat');
%       save(out, 'data');
%
%   See also RUN_ALL_DATASETS_MATLAB, RUN_ALL_METHODS.

    % Determine the repository root (this file resides in <root>/MATLAB).
    root = fileparts(fileparts(mfilename('fullpath')));
    dir  = fullfile(root, 'results');
end
