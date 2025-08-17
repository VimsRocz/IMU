function run_task4(gnss_file, imu_file, run_name)
%RUN_TASK4 Multi-frame overlays
addpath(genpath('..'));
if nargin < 3 || isempty(run_name)
    run_name = infer_run_name(gnss_file, imu_file);
end
Tg = readtable(gnss_file);
t = Tg.Posix_Time;
x = Tg.X_ECEF_m; y = Tg.Y_ECEF_m; z = Tg.Z_ECEF_m;
vx = Tg.VX_ECEF_mps; vy = Tg.VY_ECEF_mps; vz = Tg.VZ_ECEF_mps;
ax = gradient(vx,t); ay = gradient(vy,t); az = gradient(vz,t);
[lat0, lon0, ~] = ecef2lla_wgs84_fallback(x(1), y(1), z(1));
[n,e,d] = ecef_to_ned(x,y,z,lat0,lon0);
[vn,ve,vd] = ecef_to_ned(vx,vy,vz,lat0,lon0);
[an,ae,ad] = ecef_to_ned(ax,ay,az,lat0,lon0);
Ti = readmatrix(imu_file);
ti = Ti(:,2); acc = Ti(:,6:8);
fig = figure('Position',[100 100 900 800]);
% Body
subplot(3,3,7); plot(ti, acc); title('Body Accel'); legend('x','y','z');
% ECEF
subplot(3,3,1); plot(t, [x y z]); title('ECEF Pos'); legend('X','Y','Z');
subplot(3,3,2); plot(t, [vx vy vz]); title('ECEF Vel'); legend('VX','VY','VZ');
subplot(3,3,3); plot(t, [ax ay az]); title('ECEF Accel'); legend('AX','AY','AZ');
% NED
subplot(3,3,4); plot(t, [n e d]); title('NED Pos'); legend('N','E','D');
subplot(3,3,5); plot(t, [vn ve vd]); title('NED Vel'); legend('VN','VE','VD');
subplot(3,3,6); plot(t, [an ae ad]); title('NED Accel'); legend('AN','AE','AD');
for i = 1:9
    subplot(3,3,i); xlabel('Time [s]');
end
out_dir = fullfile('OUTPUTS', run_name, 'Task_4');
save_plot_png(fig, out_dir, 'task4_frame_overlays'); close(fig);
end

function name = infer_run_name(gnss_file, imu_file)
t = regexp(gnss_file,'X\d+','match');
if ~isempty(t), name=t{1}; return; end
t = regexp(imu_file,'X\d+','match');
if ~isempty(t), name=t{1}; else name='RUN'; end
end
