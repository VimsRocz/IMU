function run_triad_method(imu_file, gnss_file)
%RUN_TRIAD_METHOD  Run Tasks 1-5 using the TRIAD method on a single dataset.
%   RUN_TRIAD_METHOD(IMU_FILE, GNSS_FILE) loads the specified files from the
%   "Data" directory relative to the repository root.  Pass both filenames
%   explicitly.  An error is raised when either file does not exist.  This
%   helper mirrors the Python ``check_files`` functionality for parity.
%
%   Valid dataset pairs bundled with the repository are:
%       * IMU_X001.dat with GNSS_X001.csv
%       * IMU_X002.dat with GNSS_X002.csv
%       * IMU_X003.dat with GNSS_X002.csv (shared GNSS log)
%
%   Example:
%       run_triad_method('IMU_X002.dat', 'GNSS_X002.csv');

    data_dir = 'Data';
    imu_path = fullfile(data_dir, imu_file);
    gnss_path = fullfile(data_dir, gnss_file);
    if ~exist(imu_path, 'file') || ~exist(gnss_path, 'file')
        error('File not found: %s or %s', imu_path, gnss_path);
    end

    % Load data
    imu_data = readmatrix(imu_path);
    gnss_data = readtable(gnss_path);

    % Proceed with TRIAD processing (Tasks 1--5)
    Task_1(imu_path, gnss_path, 'TRIAD');
    Task_2(imu_path, gnss_path, 'TRIAD');
    Task_3(imu_path, gnss_path, 'TRIAD');
    Task_4(imu_path, gnss_path, 'TRIAD');
    Task_5(imu_path, gnss_path, 'TRIAD');
end
