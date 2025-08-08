function results_dir = get_matlab_results_dir()
%GET_MATLAB_RESULTS_DIR Return MATLAB-only results directory (independent of Python).
%   RESULTS_DIR = GET_MATLAB_RESULTS_DIR() returns the path to
%   <repository>/MATLAB/results and ensures the directory exists. This
%   keeps MATLAB outputs separate from the Python pipeline which writes to
%   <repository>/results.
%
%   See also GET_RESULTS_DIR.

    % Determine the MATLAB root folder (this file resides in <repo>/MATLAB).
    matlab_root = fileparts(mfilename('fullpath'));
    results_dir = fullfile(matlab_root, 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
end
