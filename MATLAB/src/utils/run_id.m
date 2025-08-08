function rid = run_id(imu_path, gnss_path, method)
%RUN_ID standardised run identifier IMU_<id>_GNSS_<id>_<METHOD>
%   RID = RUN_ID(IMU_PATH, GNSS_PATH, METHOD) returns a run identifier
%   composed of the IMU base name, GNSS base name and METHOD in uppercase.
%   If the dataset names do not already start with ``IMU_`` or ``GNSS_``,
%   the prefixes are added automatically.

    [~, imu_name, imu_ext]   = fileparts(imu_path);
    [~, gnss_name, gnss_ext] = fileparts(gnss_path);
    imu_name  = erase(imu_name,  imu_ext);
    gnss_name = erase(gnss_name, gnss_ext);

    if startsWith(lower(imu_name), 'imu_')
        imu_name = imu_name;
    else
        imu_name = ['IMU_' imu_name];
    end

    if startsWith(lower(gnss_name), 'gnss_')
        gnss_name = gnss_name;
    else
        gnss_name = ['GNSS_' gnss_name];
    end

    rid = sprintf('%s_%s_%s', imu_name, gnss_name, upper(string(method)));
end

