function rid = run_id(imu_path, gnss_path, method)
%RUN_ID Build a consistent run identifier.
%   rid = RUN_ID(IMU_PATH, GNSS_PATH, METHOD) concatenates the base names of
%   the IMU and GNSS files with the uppercase METHOD.
%
% Usage:
%   rid = run_id('IMU_X002.dat','GNSS_X002.csv','triad');

[~, imu_base]  = fileparts(imu_path);
[~, gnss_base] = fileparts(gnss_path);
rid = sprintf('%s_%s_%s', imu_base, gnss_base, upper(method));
end
