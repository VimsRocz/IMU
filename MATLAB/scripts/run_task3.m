function run_task3(gnss_file, imu_file, run_name)
%RUN_TASK3 Quaternion comparison plots
addpath(genpath('..'));
if nargin < 3 || isempty(run_name)
    run_name = infer_run_name(gnss_file, imu_file);
end
state_path = fullfile('..','DATA','Truth',['STATE_' run_name '.txt']);
state = readmatrix(state_path,'FileType','text','NumHeaderLines',1);
t = state(:,2); q = state(:,9:12);
meas = repmat(q(1,:), size(q,1),1);
fig1 = figure; plot(t, meas); legend('qw','qx','qy','qz'); title('Measured quaternion');
out_dir = fullfile('OUTPUTS', run_name, 'Task_3');
save_plot_png(fig1, out_dir, 'task3_measured_quat'); close(fig1);
fig2 = figure; plot(t, q); legend('qw','qx','qy','qz'); title('STATE quaternion');
save_plot_png(fig2, out_dir, 'task3_state_quat'); close(fig2);
err = meas - q; rmse = sqrt(mean(err.^2));
fig3 = figure; plot(t, meas, '--'); hold on; plot(t, q); legend('meas','state'); title(sprintf('RMSE %.3f %.3f %.3f %.3f', rmse));
save_plot_png(fig3, out_dir, 'task3_quat_comparison'); close(fig3);
end

function name = infer_run_name(gnss_file, imu_file)
t = regexp(gnss_file,'X\d+','match');
if ~isempty(t), name=t{1}; return; end
t = regexp(imu_file,'X\d+','match');
if ~isempty(t), name=t{1}; else name='RUN'; end
end
