function startup()
% Add MATLAB project folders to the path (recursively)
root = fileparts(mfilename('fullpath'));
addpath(root);
addpath(genpath(fullfile(root,'utils')));
addpath(genpath(fullfile(root,'lib')));
addpath(genpath(fullfile(root,'config')));
% Keep task subfolders on path for compatibility during migration
addpath(genpath(fullfile(root,'Task_1')));
addpath(genpath(fullfile(root,'Task_2')));
addpath(genpath(fullfile(root,'Task_3')));
addpath(genpath(fullfile(root,'Task_4')));
addpath(genpath(fullfile(root,'Task_5')));
addpath(genpath(fullfile(root,'Task_6')));
addpath(genpath(fullfile(root,'Task_7')));
fprintf('[startup] MATLAB path initialised for project under %s\n', root);
end

