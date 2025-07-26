function imu = read_imu(filename)
%READ_IMU  Load IMU measurements from a .dat file.
%   IMU = READ_IMU(FILENAME) returns a struct with time, gyroscope and
%   accelerometer data.  The file format is expected to match the sample
%   logs bundled with the repository:
%       [index, time, dtheta_x, dtheta_y, dtheta_z, dv_x, dv_y, dv_z, ...]
%
%   Gyroscope and accelerometer increments are converted to rates using the
%   sampling period derived from the time column or a default of 400 Hz.

    data = readmatrix(filename);
    if size(data,2) < 8
        error('Unexpected IMU format in %s', filename);
    end
    time = data(:,2);
    if numel(time) > 1
        dt = mean(diff(time(1:min(end,100))));
    else
        dt = 1/400; % fall back
    end
    imu.time_s = time;
    imu.gyro_radps = data(:,3:5) / dt;
    imu.accel_mps2 = data(:,6:8) / dt;
end
