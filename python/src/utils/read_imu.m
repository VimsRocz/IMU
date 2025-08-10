function data = read_imu(path)
%READ_IMU Load IMU measurements from a text file.
%   DATA = READ_IMU(PATH) returns a struct with fields 'time', 'accel',
%   'gyro' and 'mag'. The file is assumed to contain columns in this order
%   with whitespace separation.

    m = readmatrix(path);
    data.time  = m(:,2);
    data.accel = m(:,3:5);
    data.gyro  = m(:,6:8);
    if size(m,2) >= 11
        data.mag = m(:,9:11);
    else
        data.mag = zeros(size(m,1),3);
    end
end
