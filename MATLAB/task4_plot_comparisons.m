function task4_plot_comparisons(gnss_data, imu_data, fused_data, time_vec)
%TASK4_PLOT_COMPARISONS Create Task 4 comparison plots for NED, ECEF, Body.
%
% function task4_plot_comparisons(gnss_data, imu_data, fused_data, time_vec)
% Inputs:
%  - gnss_data, imu_data, fused_data: structs with fields .ned/.ecef/.body
%    Each frame contains .pos, .vel, .acc arrays of size [N x 3].
%  - time_vec: time information; can be:
%    * numeric vector (e.g., 0:dt:T) for one dataset; other datasets are
%      mapped by linspace(min,max,N)
%    * scalar duration T (seconds)
%    * struct with fields 'gnss','imu','fused' (each numeric vector)
%
% This function creates three figures (NED, ECEF, Body), each with a 3x3
% grid (rows: Position/Velocity/Acceleration; columns: axis 1/2/3), plots
% GNSS (blue), IMU only (dashed orange), and Fused (thick green). Missing
% signals render as flat zero lines and are annotated.
%
% Figures are saved to MATLAB/results as PNGs with 1800x1200 px at 200 DPI
% with the exact filenames specified in the task.

% Ensure results directory exists
outDir = fullfile('MATLAB','results');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

% Shared style
fs = 12; % font size
colorGnss  = [0.000, 0.447, 0.741]; % blue
colorImu   = [0.929, 0.694, 0.125]; % orange
colorFused = [0.466, 0.674, 0.188]; % green

% Suptitle line 2 (fixed per spec)
subtitle_line2 = 'IMU_X002_GNSS_X002_TRIAD | IMU n=500000 | GNSS n=1250 | Truth n=20087';

% Plot each frame separately
frames = {'ned','ecef','body'};
axisLabels = {
    {'North','East','Down'}, ... % NED
    {'X','Y','Z'}, ...           % ECEF
    {'BX','BY','BZ'}             % Body
};
fileNames = {
    'IMU_X002_GNSS_X002_TRIAD_task4_all_ned.png',
    'IMU_X002_GNSS_X002_TRIAD_task4_all_ecef.png',
    'IMU_X002_GNSS_X002_TRIAD_task4_all_body.png'
};

for k = 1:numel(frames)
    frame = frames{k};
    try
        % Create figure with required size
        hFig = figure('Units','pixels','Position',[100 100 1800 1200],'Color','w','Visible','off');

        % Suptitle with two lines
        frameUpper = upper(frame);
        line1 = sprintf('Task 4: All data in %s frame', frameUpper);
        sgtitle({line1; subtitle_line2}, 'FontSize', fs, 'FontWeight','bold');

        % Quantities and units per row
        quantities = {'pos','vel','acc'};
        yUnits = {'[m]','[m/s]','[m/s²]'};
        rowTitles = {'Position','Velocity','Acceleration'};

        % Prepare data references for this frame
        gnssFrame = getframefield(gnss_data, frame);
        imuFrame  = getframefield(imu_data,  frame);
        fusedFrame= getframefield(fused_data,frame);

        % Loop rows (quantities) and columns (axes)
        for r = 1:3
            qty = quantities{r};
            unitStr = yUnits{r};
            for c = 1:3
                ax = subplot(3,3,(r-1)*3 + c);
                hold(ax,'on'); grid(ax,'on');

                % Extract column signals and plot
                axisName = axisLabels{k}{c};

                % GNSS
                [gnssCol, hasGnss]   = getqtycol(gnssFrame, qty, c);
                tGnss = resolvetime(time_vec, numel(gnssCol), 'gnss');
                if hasGnss
                    p1 = plot(ax, tGnss, gnssCol, '-', 'Color', colorGnss, 'LineWidth', 1.0, 'DisplayName', 'GNSS');
                else
                    p1 = plot(ax, tGnss, zeros(size(tGnss)), '-', 'Color', colorGnss, 'LineWidth', 1.0, 'DisplayName', 'GNSS (missing)');
                end

                % IMU only
                [imuCol, hasImu]     = getqtycol(imuFrame,  qty, c);
                tImu = resolvetime(time_vec, numel(imuCol), 'imu');
                if hasImu
                    p2 = plot(ax, tImu, imuCol, '--', 'Color', colorImu, 'LineWidth', 1.0, 'DisplayName', 'IMU only');
                else
                    p2 = plot(ax, tImu, zeros(size(tImu)), '--', 'Color', colorImu, 'LineWidth', 1.0, 'DisplayName', 'IMU only (missing)');
                end

                % Fused
                [fusedCol, hasFused] = getqtycol(fusedFrame,qty, c);
                tFused = resolvetime(time_vec, numel(fusedCol), 'fused');
                if hasFused
                    p3 = plot(ax, tFused, fusedCol, '-', 'Color', colorFused, 'LineWidth', 2.0, 'DisplayName', 'Fused');
                else
                    p3 = plot(ax, tFused, zeros(size(tFused)), '-', 'Color', colorFused, 'LineWidth', 2.0, 'DisplayName', 'Fused (missing)');
                end

                % Legend, axes formatting
                legend(ax,'Location','north');
                axis(ax,'tight');
                set(ax,'FontSize',fs);

                % Labels
                yLabelText = sprintf('%s %s %s', axisName, rowTitles{r}, unitStr);
                ylabel(ax, yLabelText, 'FontSize', fs);
                if r == 3
                    xlabel(ax, 'Time [s]', 'FontSize', fs);
                end

                % Missing annotation if any missing
                if ~(hasGnss && hasImu && hasFused)
                    add_missing_annotation(ax, fs);
                end
            end
        end

        % Save figure
        drawnow;
        fname = fileNames{k};
        outPath = fullfile(outDir, fname);
        % Ensure figure size in pixels is as required before export
        set(hFig,'Units','pixels','Position',[100 100 1800 1200]);
        exportgraphics(hFig, outPath, 'Resolution', 200);

        info = dir(outPath);
        if isempty(info) || info.bytes < 5000
            error('Save failed: %s', fname);
        end
        fprintf('[SAVE] %s (%d bytes)\n', fname, info.bytes);

        close(hFig);
    catch ME
        if exist('hFig','var') && isvalid(hFig)
            close(hFig);
        end
        warning('Failed to generate %s frame figure: %s', upper(frame), ME.message);
    end
end

% Close any remaining open figures
close all;

end

% --------- Helpers ---------

function frameStruct = getframefield(dataStruct, frame)
% Safely get frame sub-struct (.ned/.ecef/.body), or empty struct
frameStruct = struct();
try
    if isstruct(dataStruct) && isfield(dataStruct, frame) && isstruct(dataStruct.(frame))
        frameStruct = dataStruct.(frame);
    end
catch
    frameStruct = struct();
end
end

function [col, has] = getqtycol(frameStruct, qty, idx)
% Get a single column from frameStruct.(qty) if present and valid
has = false;
col = [];
try
    if isstruct(frameStruct) && isfield(frameStruct, qty)
        arr = frameStruct.(qty);
        if isnumeric(arr) && ndims(arr) == 2 && size(arr,2) >= idx
            col = arr(:,idx);
            has = true;
        end
    end
catch
    has = false;
    col = [];
end
if ~has
    % return empty; caller will build time from length zero -> handle as zero vector
    col = zeros(0,1);
end
end

function t = resolvetime(tv, n, role)
% Resolve a time vector of length n for a given role: 'gnss'|'imu'|'fused'
% Supports:
%  - tv as struct with fields 'gnss','imu','fused' (or 't_gnss', etc.)
%  - tv as numeric vector (will map via linspace if lengths differ)
%  - tv as scalar duration (seconds)

% defaults
t0 = 0; T = 1400;

if nargin < 3 || isempty(role)
    role = 'fused';
end

try
    if isstruct(tv)
        % Prefer exact field
        if isfield(tv, role) && isnumeric(tv.(role)) && isvector(tv.(role))
            tRaw = tv.(role)(:);
        elseif isfield(tv, ['t_' role]) && isnumeric(tv.(['t_' role])) && isvector(tv.(['t_' role]))
            tRaw = tv.(['t_' role])(:);
        else
            tRaw = [];
        end
        if ~isempty(tRaw)
            t0 = tRaw(1);
            T  = tRaw(end);
            if numel(tRaw) == n
                t = tRaw;
                return;
            end
        end
        % fallthrough to build by linspace
    elseif isnumeric(tv) && isvector(tv)
        tv = tv(:);
        t0 = tv(1);
        T  = tv(end);
        if numel(tv) == n
            t = tv;
            return;
        end
    elseif isnumeric(tv) && isscalar(tv)
        T = tv;
    end
catch
    % ignore and use defaults
end

% Build by linspace
if n <= 1
    t = linspace(t0, T, max(n,2)).';
    t = t(1:n); % keep n elements (possibly 0)
else
    t = linspace(t0, T, n).';
end
end

function add_missing_annotation(ax, fs)
% Add a red warning text inside the axes
txt = '\x26A0 Missing data'; % fallback if Unicode warning triangle unsupported
try
    % Try rendering with the given symbol
    text(ax, 0.02, 0.90, '\x26A0 Missing data', 'Units','normalized', ...
        'Color',[0.85 0.1 0.1], 'FontWeight','bold', 'FontSize', fs);
catch
    % Fallback
    text(ax, 0.02, 0.90, txt, 'Units','normalized', ...
        'Color',[0.85 0.1 0.1], 'FontWeight','bold', 'FontSize', fs);
end
end

