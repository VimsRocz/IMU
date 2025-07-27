function run_all_methods()
%RUN_ALL_METHODS Execute Tasks 1--7 for the X002 dataset using TRIAD.
%   This helper mirrors ``src/run_triad_only.py`` but runs the MATLAB
%   pipeline.  Dataset files are resolved via ``get_data_file`` so the
%   script can be launched from any directory. All outputs are stored
%   under ``results/`` with the standard naming convention.

    dataset.imu  = 'IMU_X002.dat';
    dataset.gnss = 'GNSS_X002.csv';
    method = 'TRIAD';

    imu_path  = get_data_file(dataset.imu);
    gnss_path = get_data_file(dataset.gnss);

    if ~isfile(imu_path) || ~isfile(gnss_path)
        error('File not found: %s or %s', imu_path, gnss_path);
    end

    Task_1(imu_path, gnss_path, method);

    imu_raw = readmatrix(imu_path);
    dt_imu = mean(diff(imu_raw(1:100,2)));
    imu_data.accel = imu_raw(:,6:8) / dt_imu;
    imu_data.gyro  = imu_raw(:,3:5) / dt_imu;
    Task_2(imu_data, method);
    Task_3(imu_path, gnss_path, method);
    Task_4(imu_path, gnss_path, method);
    Task_5(imu_path, gnss_path, method);

    res_file = fullfile(get_results_dir(), sprintf('IMU_X002_GNSS_X002_%s_task5_results.mat', method));
    if isfile(res_file)
        S = load(res_file);
        if isfield(S,'pos_ned') && isfield(S,'gnss_pos_ned')
            Task_6(res_file, imu_path, gnss_path, 'STATE_X001.txt');
            Task_7(S.pos_ned, S.gnss_pos_ned, method);
        end
    end
end
