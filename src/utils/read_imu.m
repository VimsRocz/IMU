function imu = read_imu(filename)
%READ_IMU  Load IMU increments from a .dat file.
%   IMU = READ_IMU(FILENAME) reads the 10-column whitespace separated IMU
%   file. The returned struct contains ``time_s``, ``dtheta`` and ``dv``
%   fields which mirror the Python loader in ``src/utils.py``.

    data = readmatrix(filename);
    imu.time_s = data(:,2);
    imu.dtheta = data(:,3:5);
    imu.dv     = data(:,6:8);
    if size(data,2) >= 9
        imu.temperature_C = data(:,9);
    else
        imu.temperature_C = [];
    end
    if size(data,2) >= 10
        imu.status = data(:,10);
    else
        imu.status = [];
    end
end
