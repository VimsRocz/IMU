function df = read_imu_numeric(path)
% READ_IMU_NUMERIC Read IMU data coercing tokens to numeric.
%   DF = READ_IMU_NUMERIC(PATH) loads the IMU .dat file at PATH using
%   READMATRIX and ensures the output is numeric. Non-numeric tokens are
%   converted to NaN which keeps ISFINITE checks safe.

% Read raw data; READMATRIX already converts non-numerics to NaN.
df = readmatrix(path, 'FileType', 'text');

% Ensure output is double and drop empty rows.
df = double(df);
df(~any(isfinite(df), 2), :) = [];
end

