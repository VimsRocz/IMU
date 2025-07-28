function Task_7(tag)
%TASK_7  Plot truth minus fused residuals using Task 6 overlay data.
%   TASK_7(TAG) loads the overlay arrays saved by Task_6 and produces
%   6-subplot residual figures for the NED, ECEF and Body frames.
%   Summary statistics (mean, std, max per axis) are written to
%   results/<tag>_task7_summary.mat.
%
%   Example:
%       Task_7('IMU_X002_GNSS_X002_TRIAD')
%
%   This mirrors the Python post-processing step.

if nargin < 1 || isempty(tag)
    error('Task_7:TagRequired','Tag identifying the dataset/method required');
end

results_dir = 'results';
mat_file = fullfile(results_dir, sprintf('%s_task6_overlay.mat', tag));
if ~isfile(mat_file)
    error('Task_7:OverlayMissing','%s not found', mat_file);
end

D = load(mat_file);

frames = {'NED','ECEF','Body'};
summary = struct();

for k = 1:numel(frames)
    frame = frames{k};
    fn_pos_t = sprintf('pos_truth_%s', lower(frame));
    fn_vel_t = sprintf('vel_truth_%s', lower(frame));
    fn_pos_e = sprintf('pos_est_%s',  lower(frame));
    fn_vel_e = sprintf('vel_est_%s',  lower(frame));

    pos_err = D.(fn_pos_t) - D.(fn_pos_e);
    vel_err = D.(fn_vel_t) - D.(fn_vel_e);

    plot_residual_overlay(frame, D.t, pos_err, vel_err, tag);

    stats.pos_mean = mean(pos_err); %#ok<*NASGU>
    stats.pos_std  = std(pos_err);
    stats.pos_max  = max(abs(pos_err));
    stats.vel_mean = mean(vel_err);
    stats.vel_std  = std(vel_err);
    stats.vel_max  = max(abs(vel_err));
    summary.(frame) = stats;
end

save(fullfile(results_dir, sprintf('%s_task7_summary.mat', tag)), 'summary');
end

%% -------------------------------------------------------------------------
function plot_residual_overlay(frameName, t, posErr, velErr, tag)
colors = {'b','g','r'}; % N/X, E/Y, D/Z
labels = {'X','Y','Z'};
if strcmpi(frameName,'NED')
    labels = {'N','E','D'};
end
f = figure('Visible','off','Position',[100 100 1200 800]);
tl = tiledlayout(3,2,'TileSpacing','compact');
for i = 1:3
    nexttile(tl,i); hold on;
    plot(t, posErr(:,i), 'Color', colors{i});
    yline(0,'k:');
    ylabel('Position Error (m)');
    legend(sprintf('%s-%s',labels{i},'err'),'Location','best');
    title(labels{i}); grid on; axis tight;
end
for i = 1:3
    nexttile(tl,i+3); hold on;
    plot(t, velErr(:,i), 'Color', colors{i});
    yline(0,'k:');
    ylabel('Velocity Error (m/s)');
    legend(sprintf('%s-%s',labels{i},'err'),'Location','best');
    title(labels{i}); grid on; axis tight;
end
xlabel(tl,'Time (s)');
sgtitle(sprintf('Task 7.5 \x2013 %s Truth-Fused Residuals (%s)', tag, frameName));
base = fullfile('results', sprintf('%s_task7_5_diff_truth_fused_over_time_%s', tag, upper(frameName)));
print(f,[base '.pdf'],'-dpdf','-bestfit');
print(f,[base '.png'],'-dpng');
close(f);
end
