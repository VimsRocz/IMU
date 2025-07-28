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

outDir = 'results';
ovFile = fullfile(outDir,sprintf('%s_task6_overlay.mat',tag));
if ~isfile(ovFile)
    error('Task_7:OverlayMissing','%s not found', ovFile);
end
L = load(ovFile);          % brings tt,pos_*,vel_* etc.

frames = {'NED','ECEF','Body'};
summary = struct();

for k = 1:numel(frames)
    frame = frames{k};
    fn_pos_t = sprintf('pos_truth_%s', lower(frame));
    fn_vel_t = sprintf('vel_truth_%s', lower(frame));
    fn_pos_e = sprintf('pos_est_%s',  lower(frame));
    fn_vel_e = sprintf('vel_est_%s',  lower(frame));

    pos_err = L.(fn_pos_t) - L.(fn_pos_e);
    vel_err = L.(fn_vel_t) - L.(fn_vel_e);

    plot_res72(frame, L.tt, pos_err, vel_err, tag, outDir);

    stats.pos_mean = mean(pos_err); %#ok<*NASGU>
    stats.pos_std  = std(pos_err);
    stats.pos_max  = max(abs(pos_err));
    stats.vel_mean = mean(vel_err);
    stats.vel_std  = std(vel_err);
    stats.vel_max  = max(abs(vel_err));
    summary.(frame) = stats;
end

save(fullfile(outDir, sprintf('%s_task7_summary.mat', tag)), 'summary');
end

%% ---------------------------------------------------------------------
function plot_res72(frame, tt, posErr, velErr, tag, folder)
    f  = figure('Color','w','Position',[50 50 1400 600]);
    tl = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
    hdr  = {'X','Y','Z'}; if strcmp(frame,'NED'), hdr={'\x0394N','\x0394E','\x0394D'}; end

    for k = 1:3
        nexttile(k);   plot(tt,posErr(k,:),'-'); grid on;
        title(hdr{k}); if k==1, ylabel('Position Error [m]'); end

        nexttile(3+k); plot(tt,velErr(k,:),'-'); grid on;
        if k==1, ylabel('Velocity Error [m/s]'); end
    end
    xlabel(tl,'Time [s]');
    sgtitle(sprintf('Task 7.5 \x2013 Truth \x2212 Fused Residuals (%s Frame)',frame));
    base = fullfile(folder,sprintf('%s_task7_5_diff_truth_fused_over_time_%s', tag, lower(frame)));
    exportgraphics(f,[base '.png'],'Resolution',300);
    exportgraphics(f,[base '.pdf'],'ContentType','vector');
    close(f);
end
