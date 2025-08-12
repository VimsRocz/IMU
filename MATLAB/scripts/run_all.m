function run_all(gnss_file, imu_file, run_name)
%RUN_ALL Execute Tasks 1-4
addpath(genpath('..'));
run_task1(gnss_file, imu_file, run_name);
run_task2(gnss_file, imu_file, run_name);
run_task3(gnss_file, imu_file, run_name);
run_task4(gnss_file, imu_file, run_name);
end
