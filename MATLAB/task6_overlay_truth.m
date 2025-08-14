function task6_overlay_truth(fused_data, truth_data, time_vec)
%TASK6_OVERLAY_TRUTH Plot fused minus truth differences for Task 6.
%   task6_overlay_truth(fused_data, truth_data, time_vec) creates three
%   figures (NED, ECEF, Body).  Each figure is a 3x3 grid with rows for
%   Position/Velocity/Acceleration differences and columns for axes 1/2/3.
%   The differences (fused - truth) are plotted in blue with horizontal red
%   lines at ±1.  The number of samples exceeding the threshold is shown as
%   text.  Missing inputs produce grey dashed zero lines and a red warning
%   symbol.  Figures are saved as 1800x1200 PNGs at 200 DPI in
%   MATLAB/results.

outDir = fullfile('MATLAB','results');
if ~exist(outDir,'dir'); mkdir(outDir); end

colDiff    = [0.000, 0.447, 0.741]; % blue
colThresh  = [0.85,  0.10,  0.10];  % red
colMissing = [0.5   0.5   0.5];     % grey

fs = 12;
t  = time_vec(:);

frames = {'ned','ecef','body'};
axisLabels = {
    {'North','East','Down'}, ...
    {'X','Y','Z'}, ...
    {'BX','BY','BZ'}
};
fileNames = {
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_ned.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_ecef.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_body.png'
};

quantities = {'pos','vel','acc'};
rowLabels  = {'Position Diff [m]','Velocity Diff [m/s]','Acceleration Diff [m/s^2]'};

for k = 1:numel(frames)
    frame = frames{k};
    try
        hFig = figure('Units','pixels','Position',[100 100 1800 1200], ...
            'Color','w','Visible','off');
        sgtitle(sprintf('Task 6: Fused - Truth Diffs in %s | IMU_X002_GNSS_X002_TRIAD', ...
            upper(frame)), 'FontSize',fs,'FontWeight','bold');

        fFrame = getframefield(fused_data, frame);
        tFrame = getframefield(truth_data, frame);

        for r = 1:3
            qty = quantities{r};
            for c = 1:3
                ax = subplot(3,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on');
                set(ax,'FontSize',fs);
                axisName = axisLabels{k}{c};

                warn = false;
                thr  = 1; % threshold for exceedance
                nExc = 0;

                [fCol, hasF] = getqtycol(fFrame, qty, c);
                [tCol, hasT] = getqtycol(tFrame, qty, c);
                if hasF && hasT
                    fCol = interp_to_time(fCol, t);
                    tCol = interp_to_time(tCol, t);
                    diffCol = fCol - tCol;
                    plot(ax, t, diffCol, '-', 'Color', colDiff, 'LineWidth',1.2, ...
                        'DisplayName','Diff');
                    nExc = sum(abs(diffCol) > thr);
                else
                    plot(ax, t, zeros(size(t)), '--', 'Color', colMissing, ...
                        'DisplayName','Diff (missing)');
                    warn = true;
                end

                yline(ax, thr, '-', 'Color', colThresh, 'HandleVisibility','off');
                yline(ax,-thr, '-', 'Color', colThresh, 'HandleVisibility','off');
                text(ax,0.02,0.85,sprintf('Exceed >1: %d', nExc), ...
                    'Units','normalized','Color',colThresh,'FontSize',fs,'FontWeight','bold');

                legend(ax,'Location','northwest');
                ylabel(ax, rowLabels{r});
                if r == 3, xlabel(ax,'Time [s]'); end
                axis(ax,'tight');

                if warn
                    add_warning(ax, fs);
                end
            end
        end

        % Save figure
        set(hFig,'Units','pixels','Position',[100 100 1800 1200]);
        outPath = fullfile(outDir,fileNames{k});
        exportgraphics(hFig, outPath, 'Resolution',200);
        info = dir(outPath);
        if isempty(info) || info.bytes < 5000
            error('Save failed: %s', fileNames{k});
        end
        fprintf('[SAVE] %s (%d bytes)\n', info.name, info.bytes);
        close(hFig);
    catch ME
        if exist('hFig','var') && isvalid(hFig), close(hFig); end
        warning('Failed to generate Task 6 figure for %s: %s', upper(frame), ME.message);
    end
end

close all;

end

% ---------------------- Helper Functions ----------------------
function frameStruct = getframefield(S, frame)
frameStruct = struct();
try
    if isstruct(S) && isfield(S,frame) && isstruct(S.(frame))
        frameStruct = S.(frame);
    end
catch
end
end

function [col, has] = getqtycol(frameStruct, qty, idx)
col = zeros(0,1); has = false;
try
    if isstruct(frameStruct) && isfield(frameStruct, qty)
        arr = frameStruct.(qty);
        if isnumeric(arr) && ndims(arr)==2 && size(arr,2)>=idx
            col = arr(:,idx); has = true;
        end
    end
catch
end
end

function out = interp_to_time(data, t)
n = numel(data);
if n == numel(t)
    out = data(:);
elseif n > 1
    tOrig = linspace(t(1), t(end), n);
    out = interp1(tOrig(:), data(:), t, 'linear', 'extrap');
else
    out = zeros(size(t));
end
end

function add_warning(ax, fs)
try
    text(ax,0.02,0.9,char(9888),'Units','normalized','Color',[0.85 0.1 0.1], ...
        'FontSize',fs,'FontWeight','bold');
catch
    text(ax,0.02,0.9,'!', 'Units','normalized','Color',[0.85 0.1 0.1], ...
        'FontSize',fs,'FontWeight','bold');
end
end

