function run_task2(gnss_file, imu_file, run_name)
%RUN_TASK2 Sampling statistics and plots
addpath(genpath('..'));
if nargin < 3 || isempty(run_name)
    run_name = infer_run_name(gnss_file, imu_file);
end
Tg = readtable(gnss_file);
Ti = readtable(imu_file);
% assume time columns
wg = Tg.Posix_Time; wi = Ti.Var2;
dt_g = diff(wg); dt_i = diff(wi);
fs_g = 1/mean(dt_g); fs_i = 1/mean(dt_i);
fig = figure('Position',[100 100 900 300]);
subplot(1,3,1); plot(wg(2:end), dt_g); title('GNSS dt'); xlabel('Time [s]'); ylabel('dt [s]');
subplot(1,3,2); plot(wi(2:end), dt_i); title('IMU dt'); xlabel('Time [s]'); ylabel('dt [s]');
subplot(1,3,3); axis off; text(0.1,0.5,{sprintf('fs_{GNSS}=%.2f',fs_g), sprintf('fs_{IMU}=%.2f',fs_i)});
out_dir = fullfile('OUTPUTS', run_name, 'Task_2');
save_plot_png(fig, out_dir, 'task2_dt_fs');
close(fig);
end

function name = infer_run_name(gnss_file, imu_file)
t = regexp(gnss_file,'X\d+','match');
if ~isempty(t), name=t{1}; return; end
t = regexp(imu_file,'X\d+','match');
if ~isempty(t), name=t{1}; else name='RUN'; end
end
