function T = read_imu_numeric(path)
%READ_IMU_NUMERIC Read IMU .dat file coercing all columns to numeric.
%   T = READ_IMU_NUMERIC(PATH) reads the whitespace-delimited IMU data
%   at PATH and converts every column to double. Any non-numeric token
%   becomes NaN, mirroring the behaviour of the Python helper
%   ``_read_imu_numeric``.

data = readtable(path, FileType="text", Delimiter=" ");
for k = 1:width(data)
    data.(k) = str2double(string(data.(k)));
end
% Drop rows that are entirely NaN
rowsAllNan = all(ismissing(data),2);
data(rowsAllNan,:) = [];
T = data;
end

