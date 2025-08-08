function P = project_paths()
%PROJECT_PATHS  Discover project root reliably and standardize folders.

% Find the folder of the caller (works both from editor and command line)
st = dbstack('-completenames');
if isempty(st)
    here = fileparts(mfilename('fullpath'));
else
    here = fileparts(st(1).file);
end

% Assume this file lives in MATLAB/ ; project root is its parent
root = fileparts(here);

% Standard locations
P = struct();
P.root          = root;
P.results       = fullfile(root, 'results');
P.src_utils     = fullfile(root, 'src', 'utils');     % Python-shared math
P.matlab_utils  = fullfile(root, 'MATLAB', 'utils');  % MATLAB helpers

% Add to path once
if exist(P.src_utils,'dir');    addpath(P.src_utils);   end
if exist(P.matlab_utils,'dir'); addpath(P.matlab_utils);end
if exist(P.results,'dir')==0;   mkdir(P.results);       end
end
