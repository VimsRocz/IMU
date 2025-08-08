function rid = run_id(imu_path, gnss_path, method)
%RUN_ID Construct a standardized run identifier.
%   RID = RUN_ID(IMU_PATH, GNSS_PATH, METHOD) returns a string of the form
%   '<IMU>_<GNSS>_<METHOD>' using the base file names without extensions.
%   METHOD is uppercased.

[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
if nargin < 3 || isempty(method)
    method = '';
end
rid = sprintf('%s_%s_%s', imu_name, gnss_name, upper(method));
end
