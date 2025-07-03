function S = GNSS_IMU_Fusion_single(imu_file, gnss_file)
%GNSS_IMU_FUSION_SINGLE  Run the five-task pipeline for one dataset.
%   S = GNSS_IMU_FUSION_SINGLE(IMU_FILE, GNSS_FILE) resolves the dataset
%   paths using GET_DATA_FILE and sequentially executes Task_1 through
%   Task_5 using the TRIAD method.  The Task 5 result structure is loaded
%   from <IMU>_<GNSS>_TRIAD_task5_results.mat and returned.
%
%   Default files IMU_X001.dat and GNSS_X001.csv are used when the
%   arguments are omitted.

if nargin < 1 || isempty(imu_file)
    imu_file = 'IMU_X001.dat';
end
if nargin < 2 || isempty(gnss_file)
    gnss_file = 'GNSS_X001.csv';
end

imu_path  = get_data_file(imu_file);
gnss_path = get_data_file(gnss_file);
method = 'TRIAD';

Task_1(imu_path, gnss_path, method);
Task_2(imu_path, gnss_path, method);
Task_3(imu_path, gnss_path, method);
Task_4(imu_path, gnss_path, method);
Task_5(imu_path, gnss_path, method);

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
tag = sprintf('%s_%s_%s', imu_name, gnss_name, method);
res_file = fullfile('results', [tag '_task5_results.mat']);
if exist(res_file, 'file')
    S = load(res_file);
else
    warning('Results file %s not found.', res_file);
    S = struct();
end

% Rename key figures to match the Python naming scheme
rename_plot(sprintf('%s_Task3_ErrorComparison.pdf', tag), ...
            sprintf('%s_task3_errors_comparison.pdf', tag));
rename_plot(sprintf('%s_Task3_QuaternionComparison.pdf', tag), ...
            sprintf('%s_task3_quaternions_comparison.pdf', tag));
rename_plot(sprintf('%s_Task4_NEDFrame.pdf', tag), ...
            sprintf('%s_task4_comparison_ned.pdf', tag));
rename_plot(sprintf('%s_Task4_MixedFrame.pdf', tag), ...
            sprintf('%s_task4_mixed_frames.pdf', tag));
rename_plot(sprintf('%s_Task4_ECEFFrame.pdf', tag), ...
            sprintf('%s_task4_all_ecef.pdf', tag));
rename_plot(sprintf('%s_Task4_BodyFrame.pdf', tag), ...
            sprintf('%s_task4_all_body.pdf', tag));
rename_plot(sprintf('%s_Task5_Position.pdf', tag), ...
            sprintf('%s_task5_results_%s.pdf', tag, method));
rename_plot(sprintf('%s_Task5_Velocity.pdf', tag), ...
            sprintf('%s_task5_all_ned.pdf', tag));
rename_plot(sprintf('%s_Task5_Acceleration.pdf', tag), ...
            sprintf('%s_task5_all_ecef.pdf', tag));
rename_plot(sprintf('%s_Task5_Attitude.pdf', tag), ...
            sprintf('%s_task5_all_body.pdf', tag));
rename_plot(sprintf('%s_Task5_ErrorAnalysis.pdf', tag), ...
            sprintf('%s_%s_residuals.pdf', tag, lower(method)));

end

function rename_plot(src, dst)
%RENAME_PLOT Move SRC to DST inside the results folder if it exists.
results_dir = 'results';
src_path = fullfile(results_dir, src);
dst_path = fullfile(results_dir, dst);
if exist(src_path, 'file')
    movefile(src_path, dst_path, 'f');
end
end
