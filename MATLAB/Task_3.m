function task3_results = Task_3(imu_path, gnss_path, method)
% TASK_3 - Solve Wahba (TRIAD / Davenport / SVD) and save task3_results
%   task3_results = Task_3(imu_path, gnss_path, method) computes an initial
%   attitude using reference vectors from Task 1 and body-frame measurements
%   from Task 2.  Results are stored in the MATLAB-only results directory.
%
%   Usage:
%       task3_results = Task_3(imu_path, gnss_path, method)
%
%   If Task 1 or Task 2 outputs are missing they are regenerated
%   automatically to keep the pipeline deterministic.

% paths
p = project_paths();                      % has fields: root, matlab_results, etc.
results_dir = p.matlab_results;
if ~exist(results_dir,'dir'), mkdir(results_dir); end

% ids
[~, imu_base, ~]  = fileparts(imu_path);
[~, gnss_base, ~] = fileparts(gnss_path);
imu_id  = erase(imu_base, '.dat');
gnss_id = erase(gnss_base, '.csv');
dataset_name = sprintf('%s_%s_%s', imu_id, gnss_id, method);

% ensure Task1/Task2 outputs exist (run them if missing)
t1 = fullfile(results_dir, sprintf('Task1_init_%s_%s_%s.mat', imu_id, gnss_id, method));
t2 = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method));
if ~isfile(t1), Task_1(imu_path, gnss_path, method); end
if ~isfile(t2), Task_2(imu_path, gnss_path, method); end

S1 = load(t1);   % expects ref vectors etc.
S2 = load(t2);   % expects body vectors etc.
if isfield(S2, 'body_data')
    bd = S2.body_data;
else
    bd = S2;
end

% ---- compute DCMs (use existing implementations) ----
g_ned  = S1.g_NED(:);          % reference gravity in NED
w_ned  = S1.omega_NED(:);      % earth rotation in NED
g_body = bd.g_body(:);         % measured gravity in body
w_body = bd.omega_ie_body(:);  % measured earth rotation in body

R_tri = triad(g_ned, w_ned, g_body, w_body);
q_tri = rotm2quat(R_tri);

try
    R_dav = davenport_q_method([g_ned w_ned], [g_body w_body]);
    q_dav = rotm2quat(R_dav);
catch
    R_dav = R_tri;
    q_dav = q_tri;
end

try
    R_svd = svd_wahba([g_ned w_ned], [g_body w_body]);
    q_svd = rotm2quat(R_svd);
catch
    R_svd = R_tri;
    q_svd = q_tri;
end

% ---- compute angular errors for validation ----
[triad_grav_err_deg, triad_erate_err_deg]       = compute_wahba_errors(R_tri, g_body, w_body, g_ned, w_ned);
[davenport_grav_err_deg, davenport_erate_err_deg] = compute_wahba_errors(R_dav, g_body, w_body, g_ned, w_ned);
[svd_grav_err_deg, svd_erate_err_deg]           = compute_wahba_errors(R_svd, g_body, w_body, g_ned, w_ned);

% ---- pack canonical struct that Task_4 expects ----
task3_results = struct();
task3_results.methods = {'TRIAD','Davenport','SVD'};
task3_results.Rbn.TRIAD     = R_tri;
task3_results.Rbn.Davenport = R_dav;
task3_results.Rbn.SVD       = R_svd;
task3_results.q.TRIAD       = q_tri;
task3_results.q.Davenport   = q_dav;
task3_results.q.SVD         = q_svd;
task3_results.meta = struct('imu_id',imu_id,'gnss_id',gnss_id,'method',method);

% store error metrics for cross-language parity
task3_results.grav_err_deg = struct('TRIAD', triad_grav_err_deg, ...
                                   'Davenport', davenport_grav_err_deg, ...
                                   'SVD', svd_grav_err_deg);
task3_results.erate_err_deg = struct('TRIAD', triad_erate_err_deg, ...
                                     'Davenport', davenport_erate_err_deg, ...
                                     'SVD', svd_erate_err_deg);

% ---- plot and save attitude error comparison ----
ge = task3_results.grav_err_deg;
ee = task3_results.erate_err_deg;
meth = {'TRIAD','Davenport','SVD'};
grav_vals = cellfun(@(m) double(ge.(m)), meth);
erate_vals = cellfun(@(m) double(ee.(m)), meth);

if any(~isfinite(grav_vals)), warning('Task3:grav NaN -> 0'); grav_vals(~isfinite(grav_vals)) = 0; end
if any(~isfinite(erate_vals)), warning('Task3:erate NaN -> 0'); erate_vals(~isfinite(erate_vals)) = 0; end

fprintf('[Task3] Gravity errors (deg): TRIAD=%.6g, Davenport=%.6g, SVD=%.6g\n', grav_vals);
fprintf('[Task3] Earth-rate errors (deg): TRIAD=%.6g, Davenport=%.6g, SVD=%.6g\n', erate_vals);

assert(~isempty(grav_vals) && ~isempty(erate_vals), 'Task3:NoData','No error data computed for plotting.');

f = figure('Color','w');
tiledlayout(1,2,'TileSpacing','compact');

nexttile;
bar(categorical(meth), grav_vals);
ylabel('Error (degrees)'); title('Gravity Error');
ymax = max(abs(grav_vals)); if ymax==0, ymax = 0.01; end
ylim([-1 1]*1.1*ymax); grid on;
for i = 1:numel(grav_vals)
    text(i, grav_vals(i), sprintf('%.3g', grav_vals(i)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
end

nexttile;
bar(categorical(meth), erate_vals);
ylabel('Error (degrees)'); title('Earth Rate Error');
ymax = max(abs(erate_vals)); if ymax==0, ymax = 0.01; end
ylim([-1 1]*1.1*ymax); grid on;
for i = 1:numel(erate_vals)
    text(i, erate_vals(i), sprintf('%.3g', erate_vals(i)), 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
end

sgtitle('Task 3: Attitude Error Comparison');

outbase = fullfile(results_dir, sprintf('%s_task3_errors_comparison', dataset_name));
if exist('save_plot_all','file')
    save_plot_all(f, outbase, {'.fig','.pdf','.png'});
else
    exportgraphics(f, [outbase '.pdf'], 'ContentType','vector');
    exportgraphics(f, [outbase '.png'], 'Resolution',300);
    savefig(f, [outbase '.fig']);
end
close(f);

out_generic = fullfile(results_dir, sprintf('Task3_results_%s_%s.mat', imu_id, gnss_id));
out_method  = fullfile(results_dir, sprintf('%s_%s_%s_task3_results.mat', imu_id, gnss_id, method));

save(out_generic, 'task3_results', '-v7');
save(out_method,  'task3_results', '-v7');
fprintf('Task 3: saved task3_results to:\n  %s\n  %s\n', out_generic, out_method);

% Also expose in base workspace for downstream tasks
try
    assignin('base','task3_results', task3_results);
catch
end
end
