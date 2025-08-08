function paths = project_paths()
% PROJECT_PATHS: resolve repo root and results folders; add paths
here = fileparts(mfilename('fullpath'));
% <repo_root>/MATLAB/src/utils  -> go up three levels to <repo_root>
root = fileparts(fileparts(fileparts(here)));

paths = struct();
paths.root           = root;
paths.matlab_src     = fullfile(root,'MATLAB','src');
paths.matlab_utils   = fullfile(root,'MATLAB','src','utils');
paths.matlab_results = fullfile(root,'MATLAB','results');  % MATLAB-only
paths.python_results = fullfile(root,'results');           % Python-only

% Ensure MATLAB side results dir exists
if ~exist(paths.matlab_results,'dir'), mkdir(paths.matlab_results); end

% Add MATLAB code to path (idempotent)
addpath(paths.matlab_src);
addpath(paths.matlab_utils);
end
