function rid = run_id(imu_path, gnss_path, method)
%RUN_ID Consistent run label like: IMU_X002_GNSS_X002_TRIAD
%   RID = RUN_ID(IMU_PATH, GNSS_PATH, METHOD) returns a run identifier
%   composed of the IMU base name, GNSS base name and METHOD in uppercase.
%   Extensions ``.DAT`` and ``.CSV`` are stripped from the tags.

    [~, imu_file, imu_ext]   = fileparts(imu_path);
    [~, gnss_file, gnss_ext] = fileparts(gnss_path);
    imu_tag  = strrep(upper([imu_file imu_ext]),  '.DAT','');
    gnss_tag = strrep(upper([gnss_file gnss_ext]),'.CSV','');
    rid = sprintf('%s_%s_%s', imu_tag, gnss_tag, upper(method));
end

