function stats = task7_5_error_plots(data_file, tag)
%TASK7_5_ERROR_PLOTS  Plot Truth minus Fused errors for Task 7.5.
%   STATS = TASK7_5_ERROR_PLOTS(DATA_FILE, TAG) loads the fused and truth
%   trajectories saved by Task 6 and plots the signed error
%   ``truth - fused`` for position and velocity in the NED, ECEF and body
%   frames.  DATA_FILE must contain ``t`` along with variables named
%   ``pos_fused_<frame>``, ``vel_fused_<frame>``, ``pos_truth_<frame>`` and
%   ``vel_truth_<frame>`` where <frame> is ``ned``/``ecef``/``body``.
%   Figures are written to ``results/`` using TAG as filename prefix. The
%   returned STATS structure holds mean, RMS and max errors for each frame.
%
%   Example:
%       stats = task7_5_error_plots('results/X001_task6_truth_and_fused.mat', ...
%                                   'X001_TRIAD');
%
%   This MATLAB implementation mirrors the Python Task 7.5 plotting helper.

if nargin < 2 || isempty(tag)
    [~, tag] = fileparts(data_file);
end

S = load(data_file);
if ~isfield(S,'t')
    error('task7_5_error_plots:BadFile','Missing time vector ''t'' in %s',data_file);
end

frames = {'NED','ECEF','Body'};
stats = struct();
results_dir = fullfile('results');
if ~exist(results_dir,'dir'); mkdir(results_dir); end

for i = 1:numel(frames)
    frame = frames{i};
    f = lower(frame);
    pos_fused = S.(sprintf('pos_fused_%s',f));
    vel_fused = S.(sprintf('vel_fused_%s',f));
    pos_truth = S.(sprintf('pos_truth_%s',f));
    vel_truth = S.(sprintf('vel_truth_%s',f));

    pos_err = pos_truth - pos_fused;
    vel_err = vel_truth - vel_fused;

    stats.(frame).mean_pos = mean(pos_err,1);
    stats.(frame).rms_pos = sqrt(mean(pos_err.^2,1));
    stats.(frame).max_pos = max(abs(pos_err),[],1);
    stats.(frame).mean_vel = mean(vel_err,1);
    stats.(frame).rms_vel = sqrt(mean(vel_err.^2,1));
    stats.(frame).max_vel = max(abs(vel_err),[],1);

    [~,idx] = max(vecnorm(pos_err,2,2));
    fprintf('Task 7.5 (%s):  max |\x0394Pos|=%.3f m  at t=%.1f\n', ...
        frame, max(vecnorm(pos_err,2,2)), S.t(idx));

    base = fullfile(results_dir, sprintf('%s_task7_5_diff_truth_fused_over_time_%s', tag, frame));
    plot_error_over_time(frame, pos_err, vel_err, S.t, tag, base);

    stats_file = fullfile(results_dir, sprintf('%s_task7_error_stats_%s.mat', tag, frame));
    save(stats_file, '-struct', 'stats', frame);
end

end

function plot_error_over_time(frame, pos_err, vel_err, t, tag, out_base)
    warn_state = warning('off','MATLAB:axes:NegativeDataInLog');
    f = figure('Visible','off','Position',[100 100 900 400]);
    tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
    switch upper(frame)
        case 'NED'; labels = {'N','E','D'};
        otherwise; labels = {'X','Y','Z'};
    end
    ax1 = gobjects(1,3); ax2 = gobjects(1,3);
    for k=1:3
        ax1(k) = nexttile; %#ok<LNTNS>
        plot(t, pos_err(:,k),'k-','LineWidth',1.0);
        yline(0,'Color',[.5 .5 .5],'LineStyle','--');
        title(sprintf('\x0394Pos %s', labels{k}));
        ylabel('Position Error (m)');
        grid on; axis tight;
    end
    for k=1:3
        ax2(k) = nexttile; %#ok<LNTNS>
        plot(t, vel_err(:,k),'k-','LineWidth',1.0);
        yline(0,'Color',[.5 .5 .5],'LineStyle','--');
        title(sprintf('\x0394Vel %s', labels{k}));
        ylabel('Velocity Error (m/s)');
        xlabel('Epoch Time (s)');
        grid on; axis tight;
    end
    linkaxes(ax1,'y'); linkaxes(ax2,'y');
    lim1 = max(abs(pos_err),[],'all') * 1.1;
    lim2 = max(abs(vel_err),[],'all') * 1.1;
    for k=1:3
        ylim(ax1(k),[-lim1 lim1]);
        ylim(ax2(k),[-lim2 lim2]);
    end
    sgtitle(sprintf('Task 7.5 \x2013 Error (Truth \x2212 Fused)  \x2013  %s Frame  \x2013  %s', frame, tag));
    pdf = [out_base '.pdf'];
    png = [out_base '.png'];
    exportgraphics(f, png,'Resolution',300);
    saveas(f, pdf);
    close(f);
    warning(warn_state);
end
