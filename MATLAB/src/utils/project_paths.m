function paths = project_paths()
%PROJECT_PATHS Return key directories for MATLAB pipeline.
%   PATHS = PROJECT_PATHS() returns paths for root, MATLAB and results.
%   Robust against being called from any subdirectory.

%   This is the canonical implementation used throughout the project.
%
% Usage:
%   p = project_paths();
%
% The function also adds known utils directories to the MATLAB path.
%
% Returns:
%   paths.root           - project root (parent of MATLAB/)
%   paths.matlab         - .../MATLAB
%   paths.matlab_results - .../MATLAB/results (created if missing)

here = fileparts(mfilename('fullpath'));        % .../MATLAB/src/utils
paths.matlab = fileparts(fileparts(here));      % .../MATLAB
paths.root   = fileparts(paths.matlab);         % project root

paths.matlab_results = fullfile(paths.matlab, 'results');
if ~exist(paths.matlab_results, 'dir')
    mkdir(paths.matlab_results);
end

% Candidate utils dirs to add (add only if they exist)
utils_candidates = {
    fullfile(paths.matlab, 'utils')
    fullfile(paths.matlab, 'src', 'utils')
    fullfile(paths.root, 'src', 'utils')
    fullfile(paths.root, 'PYTHON', 'src', 'utils')
};

found_any = false;
for i = 1:numel(utils_candidates)
    p = utils_candidates{i};
    if exist(p, 'dir')
        addpath(p);
        found_any = true;
    end
end
if ~found_any
    warning('utils folder not found near %s', fullfile(paths.matlab, 'src', 'utils'));
end
end
