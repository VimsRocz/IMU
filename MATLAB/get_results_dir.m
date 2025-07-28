function dir = get_results_dir()
%GET_RESULTS_DIR Return path to the results directory.
%   DIR = GET_RESULTS_DIR() returns the path to ``results`` within the
%   repository root.  When the environment variable ``MATLAB_RESULTS_DIR``
%   is set, that location is used instead.  All MATLAB tasks write logs,
%   MAT-files and figures exclusively to this folder so the pipeline
%   mirrors the Python implementation.
%
%   Example:
%       out = fullfile(get_results_dir(), 'example.mat');
%       save(out, 'data');
%
%   See also RUN_ALL_DATASETS_MATLAB, RUN_ALL_METHODS.

    override = getenv('MATLAB_RESULTS_DIR');
    if ~isempty(override)
        dir = override;
    else
        % Determine the repository root (this file resides in <root>/MATLAB>).
        root = fileparts(fileparts(mfilename('fullpath')));
        dir  = fullfile(root, 'results');
    end
end
