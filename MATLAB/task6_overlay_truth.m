function task6_overlay_truth(fused_data, truth_data, time_vec)
%TASK6_OVERLAY_TRUTH Plot fused minus truth differences for Task 6.
%   task6_overlay_truth(fused_data, truth_data, time_vec) compares the
%   fused navigation solution with the truth data.  For each reference
%   frame (NED, ECEF and Body) a 3x3 figure is generated where rows
%   correspond to position, velocity and acceleration differences and the
%   columns correspond to axes 1/2/3.  Plots are saved as interactive
%   ``.fig`` files under ``MATLAB/results``.

resDir = fullfile('MATLAB','results');
if ~exist(resDir,'dir'); mkdir(resDir); end

frames = {'ned','ecef','body'};
fileNames = { ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_ned.fig', ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_ecef.fig', ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_body.fig'};
rowLabels = {'Position Diff [m]','Velocity Diff [m/s]','Acceleration Diff [m/s^2]'};

t  = time_vec(:);
N  = numel(t);
fs = 12;

for k = 1:numel(frames)
    frame = frames{k};
    f = fused_data.(frame);
    tr = truth_data.(frame);

    fused_pos = f.pos;
    fused_vel = f.vel;
    fused_acc = f.acc;

    truth_pos = tr.pos;
    truth_vel = tr.vel;
    truth_acc = tr.acc;

    if size(truth_pos,1) ~= N
        t_truth = linspace(t(1), t(end), size(truth_pos,1));
        truth_pos = interp1(t_truth, truth_pos, t, 'linear', 'extrap');
    end
    if size(truth_vel,1) ~= N
        t_truth = linspace(t(1), t(end), size(truth_vel,1));
        truth_vel = interp1(t_truth, truth_vel, t, 'linear', 'extrap');
    end
    if size(truth_acc,1) ~= N
        t_truth = linspace(t(1), t(end), size(truth_acc,1));
        truth_acc = interp1(t_truth, truth_acc, t, 'linear', 'extrap');
    end

    diff_pos = fused_pos - truth_pos;
    diff_vel = fused_vel - truth_vel;
    diff_acc = fused_acc - truth_acc;

    hFig = figure('Visible','off','Color','w', ...
        'Position',[100 100 1800 1200]);
    sgtitle(sprintf('Task 6: Fused - Truth Diffs in %s Frame | IMU_X002_GNSS_X002_TRIAD', ...
        upper(frame)), 'FontSize', fs);

    for r = 1:3
        for c = 1:3
            ax = subplot(3,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on');
            set(ax,'FontSize',fs);
            switch r
                case 1, data = diff_pos(:,c); ylabelTxt = rowLabels{1};
                case 2, data = diff_vel(:,c); ylabelTxt = rowLabels{2};
                case 3, data = diff_acc(:,c); ylabelTxt = rowLabels{3};
            end
            plot(ax, t, data, 'b-');
            yline(ax, 1, 'r--');
            yline(ax,-1, 'r--');
            nExc = sum(abs(data) > 1);
            text(ax,0.02,0.9,sprintf('Exceed >1: %d', nExc), ...
                'Units','normalized','Color','r','FontSize',fs,'FontWeight','bold');
            axis(ax,'tight');
            if c == 1
                ylabel(ax, ylabelTxt);
            end
            if r == 3
                xlabel(ax, 'Time [s]');
            end
        end
    end

    savefig(hFig, fullfile(resDir, fileNames{k}));
    fprintf('[SAVE] %s saved as interactive .fig\n', fileNames{k});
    close(hFig);
end

end

