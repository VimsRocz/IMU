function rid = run_id(imu_path, gnss_path, method)
%RUN_ID Make a consistent run id like "IMU_X002_GNSS_X002_TRIAD"
%   RID = RUN_ID(IMU_PATH, GNSS_PATH, METHOD) returns a run identifier
%   composed of the IMU base name, GNSS base name, and METHOD in uppercase.
%   METHOD must be provided.

    if nargin < 3
        error('run_id needs imu_path, gnss_path, method');
    end
    [~, imu_name, ~]  = fileparts(imu_path);   % e.g., IMU_X002
    [~, gnss_name, ~] = fileparts(gnss_path);  % e.g., GNSS_X002
    rid = sprintf('%s_%s_%s', imu_name, gnss_name, upper(string(method)));
    rid = char(rid);
end

