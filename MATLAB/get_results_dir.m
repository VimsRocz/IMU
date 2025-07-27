function dir = get_results_dir()
%GET_RESULTS_DIR Return path to the results directory.
%   DIR = GET_RESULTS_DIR() returns the path to the 'results' folder
%   located alongside this function. MATLAB scripts use this helper to
%   store logs and figures consistently next to the Python outputs.
%
%   Example:
%       out = fullfile(get_results_dir(), 'example.mat');
%       save(out, 'data');
%
%   See also RUN_ALL_DATASETS_MATLAB, RUN_ALL_METHODS.

    dir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
