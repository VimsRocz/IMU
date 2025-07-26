function run_all_datasets_matlab(method)
%RUN_ALL_DATASETS_MATLAB  Loop over all IMU/GNSS pairs.
%   RUN_ALL_DATASETS_MATLAB(METHOD) calls Task_1 for each available dataset
%   pair using the specified initialisation METHOD.  When METHOD is omitted
%   it defaults to 'TRIAD'.
%
% The helper mirrors ``src/run_triad_only.py`` but is implemented entirely in
% MATLAB for cross-language parity.
%
% Example:
%   run_all_datasets_matlab('TRIAD');
%
% See also: Task_1, run_triad_only

    if nargin < 1 || isempty(method)
        method = 'TRIAD';
    end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    data_dir = fullfile(root, 'Data');
    if ~exist(data_dir, 'dir')
        data_dir = root;
    end

    pairs = {
        'IMU_X001.dat', 'GNSS_X001.csv';
        'IMU_X002.dat', 'GNSS_X002.csv';
        'IMU_X003.dat', 'GNSS_X002.csv';
    };

    for k = 1:size(pairs, 1)
        imu_file  = fullfile(data_dir, pairs{k,1});
        gnss_file = fullfile(data_dir, pairs{k,2});
        if ~isfile(imu_file)
            imu_file = fullfile(root, pairs{k,1});
        end
        if ~isfile(gnss_file)
            gnss_file = fullfile(root, pairs{k,2});
        end
        Task_1(imu_file, gnss_file, method);
    end
end
