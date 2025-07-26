function result = process_data(data, method)
%PROCESS_DATA Run GNSS/IMU fusion for a single dataset.
%   RESULT = PROCESS_DATA(DATA, METHOD) calls GNSS_IMU_Fusion to execute
%   the MATLAB pipeline. The resulting Task 5 MAT file is loaded and key
%   arrays are returned in RESULT.

if nargin < 2
    method = 'TRIAD';
end

GNSS_IMU_Fusion(data.imu_file, data.gnss_file, method);

pair_tag = [erase(data.imu_file,'.dat') '_' erase(data.gnss_file,'.csv')];
mat_file = fullfile('results', sprintf('%s_%s_task5_results.mat', pair_tag, method));
if exist(mat_file,'file')
    S = load(mat_file);
    result.time = (0:size(S.x_log,2)-1).';
    result.fused_pos = S.x_log(1:3,:).';
    result.fused_vel = S.vel_log.';
    result.gnss_pos = S.gnss_pos_ned;
    result.gnss_vel = S.gnss_vel_ned;
else
    warning('Result file not found: %s', mat_file);
    result = struct();
end
end
