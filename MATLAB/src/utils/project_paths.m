function P = project_paths()
%PROJECT_PATHS Resolve repository root and results path; add utils.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(fileparts(here)));  % up to repo root

P = struct();
P.root = root;
P.matlab_results = fullfile(root,'MATLAB','results');

addpath(fullfile(root,'MATLAB','src','utils'));
if ~exist(P.matlab_results,'dir'), mkdir(P.matlab_results); end
end
