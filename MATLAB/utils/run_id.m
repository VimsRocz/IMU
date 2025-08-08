function rid = run_id(imu_path, gnss_path, method)
%RUN_ID Construct a consistent run identifier.
%   rid = RUN_ID(IMU_PATH, GNSS_PATH, METHOD) returns a string identifier
%   of the form ``IMU_X002_GNSS_X002_TRIAD``. Only the base names of the
%   input files are used and METHOD is upper-cased.
%
%   Usage:
%       rid = run_id('IMU_X002.dat', 'GNSS_X002.csv', 'triad');

[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);

% Normalize names
imu_tag  = regexprep(imu_base,  '^IMU_','IMU_');
gnss_tag = regexprep(gnss_base, '^GNSS_','GNSS_');

rid = sprintf('%s_%s_%s', imu_tag, gnss_tag, upper(method));
end

