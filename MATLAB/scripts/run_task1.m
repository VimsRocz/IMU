function run_task1(gnss_file, imu_file, run_name)
%RUN_TASK1 Plot world map track
addpath(genpath('..'));
if nargin < 3 || isempty(run_name)
    run_name = infer_run_name(gnss_file, imu_file);
end
T = readtable(gnss_file);
if all(T.Latitude_deg == 0) && all(T.Longitude_deg == 0)
    [lat, lon] = ecef2lla_wgs84_fallback(T.X_ECEF_m, T.Y_ECEF_m, T.Z_ECEF_m);
else
    lat = T.Latitude_deg; lon = T.Longitude_deg;
end
world_map_track(lat, lon);
out_dir = fullfile('OUTPUTS', run_name, 'Task_1');
save_plot_png(gcf, out_dir, 'task1_world_track');
close(gcf);
end

function name = infer_run_name(gnss_file, imu_file)
t = regexp(gnss_file,'X\d+','match');
if ~isempty(t), name=t{1}; return; end
t = regexp(imu_file,'X\d+','match');
if ~isempty(t), name=t{1}; else name='RUN'; end
end
