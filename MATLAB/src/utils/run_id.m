function rid = run_id(imu_path, gnss_path, method)
%RUN_ID standardized run identifier: IMU_<name>_GNSS_<name>_<METHOD>

    [~, imu_name, imu_ext]   = fileparts(imu_path);
    [~, gnss_name, gnss_ext] = fileparts(gnss_path);
    imu_name  = erase(imu_name,  imu_ext);
    gnss_name = erase(gnss_name, gnss_ext);

    % normalize prefixes
    if ~startsWith(lower(imu_name),'imu_');   imu_name  = ['IMU_'  imu_name];  end
    if ~startsWith(lower(gnss_name),'gnss_'); gnss_name = ['GNSS_' gnss_name]; end

    rid = sprintf('%s_%s_%s', imu_name, gnss_name, upper(string(method)));
end
