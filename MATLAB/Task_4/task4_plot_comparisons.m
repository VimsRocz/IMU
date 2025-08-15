function task4_plot_comparisons(gnss_data, imu_data, fused_data, time_vec)
%TASK4_PLOT_COMPARISONS Plot GNSS and IMU-only data in 3x3 grids.
%   task4_plot_comparisons(gnss_data, imu_data, fused_data, time_vec)
%   creates comparison figures for NED, ECEF and Body frames.  Each figure
%   contains a 3x3 subplot grid with rows corresponding to Position,
%   Velocity and Acceleration and columns for the first, second and third
%   axes.  Missing data are replaced by grey dashed zero lines and
%   annotated with a red warning symbol.  Figures are written to
%   MATLAB/results as 1800x1200 PNGs at 200 DPI.
% Removed redundant individual NED pos/vel/acc plots as they do not make
% sense in Task 4.
% Task 4: Label results as "Derived GNSS" or "Derived IMU".

disp('Task 4: Removed nonsensical individual NED plots.');

% Ensure output directory exists
outDir = fullfile('MATLAB','results');
if ~exist(outDir,'dir'); mkdir(outDir); end

% Colours
colGnss   = [0.000, 0.447, 0.741]; % blue
colImu    = [0.929, 0.694, 0.125]; % orange
colMissing= [0.5   0.5   0.5  ];   % grey

fs = 12;               % font size
t  = time_vec(:);      % ensure column vector for x-axis

frames = {'ned','ecef','body'};
fileNames = {
    'IMU_X002_GNSS_X002_TRIAD_task4_ned.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task4_ecef.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task4_body.png'
};

quantities = {'pos','vel','acc'};
rowLabels  = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};

for k = 1:numel(frames)
    frame = frames{k};
    try
        hFig = figure('Units','pixels','Position',[100 100 1800 1200], ...
            'Color','w','Visible','off');
        sgtitle(sprintf('Task 4: Comparisons in %s Frame | IMU_X002_GNSS_X002_TRIAD', ...
            upper(frame)), 'FontSize',fs,'FontWeight','bold');

        gFrame = getframefield(gnss_data, frame);
        iFrame = getframefield(imu_data,  frame);

        for r = 1:3
            qty = quantities{r};
            for c = 1:3
                ax = subplot(3,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on');
                set(ax,'FontSize',fs);

                warn = false;

                % Determine labels based on frame and quantity
                [gnssLabel, imuLabel] = get_labels(frame, qty);

                % GNSS
                [col, has] = getqtycol(gFrame, qty, c);
                if has
                    col = interp_to_time(col, t);
                    plot(ax, t, col, '-', 'Color', colGnss, 'LineWidth',1.0, ...
                        'DisplayName', gnssLabel);
                else
                    plot(ax, t, zeros(size(t)), '--', 'Color', colMissing, ...
                        'DisplayName',[gnssLabel ' (missing)']);
                    warn = true;
                end

                % IMU only
                [col, has] = getqtycol(iFrame, qty, c);
                if has
                    col = interp_to_time(col, t);
                    plot(ax, t, col, '--', 'Color', colImu, 'LineWidth',1.0, ...
                        'DisplayName', imuLabel);
                else
                    plot(ax, t, zeros(size(t)), '--', 'Color', colMissing, ...
                        'DisplayName',[imuLabel ' (missing)']);
                    warn = true;
                end

                legend(ax,'Location','northeast');
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
        warning('Failed to generate %s frame plot: %s', upper(frame), ME.message);
    end
end

close all;

disp('Task 4: Updated all_ned plot with Derived GNSS/IMU labels; no fused data.');

end

% ---------------------- Helper Functions ----------------------
function [gnssLabel, imuLabel] = get_labels(frame, qty)
switch frame
    case 'ecef'
        if strcmp(qty,'acc')
            gnssLabel = 'Derived GNSS';
        else
            gnssLabel = 'Measured GNSS';
        end
        imuLabel = 'Derived IMU';
    case 'body'
        gnssLabel = 'Derived GNSS';
        if strcmp(qty,'acc')
            imuLabel = 'Measured IMU';
        else
            imuLabel = 'Derived IMU';
        end
    otherwise % 'ned'
        gnssLabel = 'Derived GNSS';
        imuLabel  = 'Derived IMU';
end
end

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

