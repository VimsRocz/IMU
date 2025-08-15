function task3_results = Task_3(imu_path, gnss_path, method)
% TASK_3 - Solve Wahba (TRIAD / Davenport / SVD) and save task3_results
% Consolidated Task 3 implementation from MATLAB/Task_3/Task_3.m

addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));

try
    cfg = evalin('caller','cfg');
catch
    try, cfg = evalin('base','cfg'); catch, cfg = cfg.default_cfg(); end
end
visibleFlag = 'off';
if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures, visibleFlag = 'on'; end

p = project_paths();
results_dir = p.matlab_results;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);
imu_id  = erase(imu_base, '.dat');
gnss_id = erase(gnss_base, '.csv');

t1 = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method));
t2 = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method));
if ~isfile(t1), Task_1(imu_path, gnss_path, method); end
if ~isfile(t2), Task_2(imu_path, gnss_path, method); end

S1 = load(t1);
S2 = load(t2);
if isfield(S2, 'body_data'), bd = S2.body_data; else, bd = S2; end

g_ned  = S1.g_NED(:);
w_ned  = S1.omega_NED(:);
g_body = bd.g_body(:);
w_body = bd.omega_ie_body(:);

R_tri = triad(g_ned, w_ned, g_body, w_body);
q_tri = rotm2quat(R_tri);
try, R_dav = davenport_q_method([g_ned w_ned], [g_body w_body]); q_dav = rotm2quat(R_dav); catch, R_dav = R_tri; q_dav = q_tri; end
try, R_svd = svd_wahba([g_ned w_ned], [g_body w_body]); q_svd = rotm2quat(R_svd); catch, R_svd = R_tri; q_svd = q_tri; end

eul_tri = rad2deg(rotm2eul(R_tri, 'ZYX'));
eul_dav = rad2deg(rotm2eul(R_dav, 'ZYX'));
eul_svd = rad2deg(rotm2eul(R_svd, 'ZYX'));
diff_dav = eul_dav - eul_tri; diff_svd = eul_svd - eul_tri;
fig_cmp = figure('Name', 'Task 3 Attitude Comparison', 'Visible', visibleFlag);
bar([diff_dav; diff_svd]'); set(gca,'XTickLabel',{'Yaw','Pitch','Roll'});
ylabel('Difference [deg]'); legend({'Davenport - TRIAD','SVD - TRIAD'},'Location','best');
title('Attitude difference vs TRIAD');
base = sprintf('%s_%s_%s_task3_attitude', imu_id, gnss_id, method);
save_plot_fig(fig_cmp, fullfile(results_dir, [base '.fig']));
if isfield(cfg,'plots') && isfield(cfg.plots,'save_pdf') && cfg.plots.save_pdf
    set(fig_cmp, 'PaperPosition', [0 0 8 6]); print(fig_cmp, fullfile(results_dir,[base '.pdf']), '-dpdf', '-bestfit');
end
if isfield(cfg,'plots') && isfield(cfg.plots,'save_png') && cfg.plots.save_png
    exportgraphics(fig_cmp, fullfile(results_dir,[base '.png']), 'Resolution', 300);
end

task3_results = struct();
task3_results.methods = {'TRIAD','Davenport','SVD'};
task3_results.Rbn.TRIAD     = R_tri;
task3_results.Rbn.Davenport = R_dav;
task3_results.Rbn.SVD       = R_svd;
task3_results.q.TRIAD       = q_tri;
task3_results.q.Davenport   = q_dav;
task3_results.q.SVD         = q_svd;
task3_results.meta = struct('imu_id',imu_id,'gnss_id',gnss_id,'method',method);

out_generic = fullfile(results_dir, sprintf('Task3_results_%s_%s.mat', imu_id, gnss_id));
out_method  = fullfile(results_dir, sprintf('%s_%s_%s_task3_results.mat', imu_id, gnss_id, method));
save(out_generic, 'task3_results', '-v7'); save(out_method,  'task3_results', '-v7');
try, assignin('base','task3_results', task3_results); catch, end
end

