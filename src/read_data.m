function data = read_data(imu_file, gnss_file)
%READ_DATA Load IMU and GNSS measurements.
%   DATA = READ_DATA(IMU_FILE, GNSS_FILE) returns a struct with the raw
%   IMU matrix and GNSS table. This mirrors the Python helper function used
%   in run_all_methods.py.

if nargin < 1
    error('IMU file required');
end
if nargin < 2
    error('GNSS file required');
end

data.imu_file = imu_file;
data.gnss_file = gnss_file;

data.imu  = readmatrix(imu_file);
data.gnss = readtable(gnss_file);
end
