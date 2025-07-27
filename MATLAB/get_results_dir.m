function dir = get_results_dir()
%GET_RESULTS_DIR Return path to the MATLAB results directory.
%   DIR = GET_RESULTS_DIR() returns the path to ``MATLAB/results`` within
%   the repository. All MATLAB tasks write logs, MAT-files and figures
%   exclusively to this folder so the pipeline remains independent of the
%   Python implementation.
%
%   Example:
%       out = fullfile(get_results_dir(), 'example.mat');
%       save(out, 'data');
%
%   See also RUN_ALL_DATASETS_MATLAB, RUN_ALL_METHODS.

    % Determine the repository root (this file resides in <root>/MATLAB).
    root = fileparts(fileparts(mfilename('fullpath')));
    dir  = fullfile(root, 'MATLAB', 'results');
end
