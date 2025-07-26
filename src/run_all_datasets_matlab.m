function run_all_datasets_matlab(method)
%RUN_ALL_DATASETS_MATLAB Batch process all IMU/GNSS pairs
%   RUN_ALL_DATASETS_MATLAB(METHOD) runs Task_1 for each dataset pair
%   using the specified initialisation METHOD (e.g. 'TRIAD').
%
% Example:
%   run_all_datasets_matlab('TRIAD')

    if nargin < 1 || isempty(method)
        method = 'TRIAD';
    end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(here);
    data_dir = root;
    pairs = {
        'IMU_X001.dat', 'GNSS_X001.csv';
        'IMU_X002.dat', 'GNSS_X002.csv';
        'IMU_X003.dat', 'GNSS_X002.csv';
    };

    for k = 1:size(pairs,1)
        imu_file  = fullfile(data_dir, pairs{k,1});
        gnss_file = fullfile(data_dir, pairs{k,2});
        fprintf('Processing %s + %s using %s...\n', pairs{k,1}, pairs{k,2}, method);
        Task_1(imu_file, gnss_file, method);
    end
end
