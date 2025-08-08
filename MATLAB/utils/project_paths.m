function paths = project_paths()
%PROJECT_PATHS Robust paths for the MATLAB pipeline (independent from Python).
%   paths = PROJECT_PATHS() returns a struct with paths.root, paths.matlab,
%   and paths.matlab_results. The function also adds nearby utils directories
%   to the MATLAB path so that helper functions are found reliably.
%
%   Usage:
%       p = project_paths();
%
%   The results directory (MATLAB/results) is created if it does not exist.

here = fileparts(mfilename('fullpath'));  % .../IMU/MATLAB
paths.matlab = here;
paths.root   = fileparts(here);           % .../IMU (project root)

% results dir (MATLAB-only)
paths.matlab_results = fullfile(paths.matlab,'results');
if ~exist(paths.matlab_results,'dir'), mkdir(paths.matlab_results), end

% candidate utils dirs (add only if they exist)
utils_candidates = {
    fullfile(paths.matlab,'utils')
    fullfile(paths.matlab,'src','utils')
    fullfile(paths.root,'MATLAB','utils')
    fullfile(paths.root,'MATLAB','src','utils')
    fullfile(paths.root,'src','utils')
};

found_any = false;
for i = 1:numel(utils_candidates)
    p = utils_candidates{i};
    if exist(p,'dir')
        addpath(p);
        found_any = true;
    end
end
if ~found_any
    warning('utils folder not found near %s', fullfile(paths.matlab,'src','utils'));
end
end

