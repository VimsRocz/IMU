function GNSS_IMU_Fusion(imu_file, gnss_file, method)
%GNSS_IMU_FUSION  MATLAB wrapper for the Python GNSS_IMU_Fusion pipeline
%   GNSS_IMU_FUSION(IMU_FILE, GNSS_FILE, METHOD) runs the five processing
%   tasks using the specified IMU log, GNSS log and attitude-initialisation
%   method ('TRIAD', 'Davenport' or 'SVD').  The generated plots and MAT
%   files match those produced by the Python script GNSS_IMU_Fusion.py.
%
%   When IMU_FILE or GNSS_FILE are omitted the default logs IMU_X001.dat and
%   GNSS_X001.csv are used.  METHOD defaults to 'Davenport'.  Results are
%   written to the directory returned by ``get_results_dir()``.

if nargin < 1 || isempty(imu_file)
    imu_file = 'IMU_X001.dat';
end
if nargin < 2 || isempty(gnss_file)
    gnss_file = 'GNSS_X001.csv';
end
if nargin < 3 || isempty(method)
    method = 'Davenport';
end

fprintf('GNSS\\_IMU\\_Fusion: %s + %s using %s method\n', ...
        imu_file, gnss_file, method);

if ~exist('output_matlab','dir')
    mkdir('output_matlab');
end

Task_1(imu_file, gnss_file, method);
Task_2(imu_file, gnss_file, method);
Task_3(imu_file, gnss_file, method);
Task_4(imu_file, gnss_file, method);
Task_5(imu_file, gnss_file, method);

fprintf('All tasks completed. Results saved to %s\n', fullfile(pwd,'output_matlab'));
end
