function task5_plot_fusion_results(fused_data, raw_data, time_vec, blowup_times)
%TASK5_PLOT_FUSION_RESULTS Plot fused vs raw IMU signals with blow-up markers.
%   task5_plot_fusion_results(fused_data, raw_data, time_vec, blowup_times)
%   creates one figure per frame (NED, ECEF, Body).  Each figure is a 3x3
%   grid (rows: Position/Velocity/Acceleration; columns: axis 1/2/3) showing
%   fused data (blue solid) against raw IMU integration (red dashed).  Red
%   dashed vertical lines indicate blow-up times.  Missing data are drawn as
%   grey dashed zero lines and annotated with a red warning symbol.  Figures
%   are saved to MATLAB/results as 1800x1200 PNGs at 200 DPI.

if nargin < 4, blowup_times = []; end

outDir = fullfile('MATLAB','results');
if ~exist(outDir,'dir'); mkdir(outDir); end

% Colours
colFused   = [0.000, 0.447, 0.741]; % blue
colRaw     = [0.850, 0.325, 0.098]; % red
colBlow    = [0.85,  0.10,  0.10];  % red for markers
colMissing = [0.5   0.5   0.5];     % grey

fs = 12;               % font size
t  = time_vec(:);      % x-axis

frames = {'ned','ecef','body'};
axisLabels = {
    {'North','East','Down'}, ...
    {'X','Y','Z'}, ...
    {'BX','BY','BZ'}
};
fileNames = {
    'IMU_X002_GNSS_X002_TRIAD_task5_ned.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task5_ecef.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task5_body.png'
};

quantities = {'pos','vel','acc'};
rowLabels  = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};

for k = 1:numel(frames)
    frame = frames{k};
    try
        hFig = figure('Units','pixels','Position',[100 100 1800 1200], ...
            'Color','w','Visible','off');
        sgtitle(sprintf('Task 5: Fused vs Raw in %s Frame | IMU_X002_GNSS_X002_TRIAD | Blow-ups=%d', ...
            upper(frame), numel(blowup_times)), 'FontSize',fs,'FontWeight','bold');

        fFrame = getframefield(fused_data, frame);
        rFrame = getframefield(raw_data,   frame);

        for r = 1:3
            qty = quantities{r};
            for c = 1:3
                ax = subplot(3,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on');
                set(ax,'FontSize',fs);
                axisName = axisLabels{k}{c};

                warn = false;

                % Raw IMU
                [col, has] = getqtycol(rFrame, qty, c);
                if has
                    col = interp_to_time(col, t);
                    plot(ax, t, col, '--', 'Color', colRaw, 'LineWidth',1.0, ...
                        'DisplayName','IMU raw');
                else
                    plot(ax, t, zeros(size(t)), '--', 'Color', colMissing, ...
                        'DisplayName','IMU raw (missing)');
                    warn = true;
                end

                % Fused
                [col, has] = getqtycol(fFrame, qty, c);
                if has
                    col = interp_to_time(col, t);
                    plot(ax, t, col, '-', 'Color', colFused, 'LineWidth',1.2, ...
                        'DisplayName','Fused');
                else
                    plot(ax, t, zeros(size(t)), '--', 'Color', colMissing, ...
                        'DisplayName','Fused (missing)');
                    warn = true;
                end

                legend(ax,'Location','northwest');
                ylabel(ax, rowLabels{r});
                if r == 3, xlabel(ax,'Time [s]'); end
                axis(ax,'tight');
                draw_blowups(ax, blowup_times, colBlow);

                if warn
                    add_warning(ax, fs);
                end
            end
        end

        % Save
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
        warning('Failed to generate Task 5 figure for %s: %s', upper(frame), ME.message);
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

function draw_blowups(ax, times, col)
if isempty(times), return; end
times = times(:)';
for tt = times
    try
        xline(ax, tt, '--', 'Color', col, 'HandleVisibility','off');
    catch
        yl = ylim(ax);
        plot(ax, [tt tt], yl, '--', 'Color', col, 'HandleVisibility','off');
    end
end
end

