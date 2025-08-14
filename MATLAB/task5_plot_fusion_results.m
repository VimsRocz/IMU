function task5_plot_fusion_results(fused_data, raw_data, time_vec)
%TASK5_PLOT_FUSION_RESULTS Standardized plots for Kalman fused results (Task 5).
%
% function task5_plot_fusion_results(fused_data, raw_data, time_vec)
% Inputs:
%  - fused_data: struct with frames .ned/.ecef/.body/.mixed containing
%                .pos, .vel, .acc arrays [N x 3]
%  - raw_data:   struct with frames .ned/.ecef/.body/.mixed containing
%                .pos, .vel, .acc arrays [N x 3] (raw IMU/integrated)
%  - time_vec:   numeric vector, scalar duration, or struct with fields
%                'fused','raw' (or t_fused/t_raw). Used for x-axes.
%
% Outputs: Saves PNGs in MATLAB/results with required names and sizes.

% Ensure output directory exists
outDir = fullfile('MATLAB','results');
if ~exist(outDir,'dir'); mkdir(outDir); end

fs = 12; % font size

% Colors
colBlue   = [0.000, 0.447, 0.741];
colOrange = [0.850, 0.325, 0.098];
colYellow = [0.929, 0.694, 0.125];
colRed    = [0.85,  0.10,  0.10];

% Task 5 subtitle line 2 for comparison figures
subtitle_line2_cmp = 'IMU_X002_GNSS_X002_TRIAD | ZUPT=479587 | Blow-ups=9662';

% Extract potential blow-up times (seconds) if provided
blowup_times = extract_blowups(time_vec, fused_data);

% ---------------------- Main NED fused-only figure ----------------------
try
    hFig = figure('Units','pixels','Position',[100 100 1800 1200], 'Color','w', 'Visible','off');
    sgtitle('Task 5: Fused Results in NED Frame | IMU_X002_GNSS_X002_TRIAD', ...
        'FontSize', fs, 'FontWeight','bold');

    % Get fused NED data
    fNED = getframefield(fused_data, 'ned');
    [posN, hasPos] = getqty(fNED, 'pos');
    [velN, hasVel] = getqty(fNED, 'vel');
    [accN, hasAcc] = getqty(fNED, 'acc');

    % Resolve times (use fused timeline)
    tPos = resolvetime(time_vec, size(posN,1), 'fused');
    tVel = resolvetime(time_vec, size(velN,1), 'fused');
    tAcc = resolvetime(time_vec, size(accN,1), 'fused');

    % Row 1: Position
    ax1 = subplot(3,1,1); hold(ax1,'on'); grid(ax1,'on'); set(ax1,'FontSize',fs);
    if hasPos
        plot(ax1, tPos, posN(:,1), '-', 'Color', colBlue,   'DisplayName','North');
        plot(ax1, tPos, posN(:,2), '-', 'Color', colOrange, 'DisplayName','East');
        plot(ax1, tPos, posN(:,3), '-', 'Color', colYellow, 'DisplayName','Down');
    else
        plot(ax1, tPos, zeros(size(tPos)), '-', 'Color', colBlue,   'DisplayName','North (missing)');
        plot(ax1, tPos, zeros(size(tPos)), '-', 'Color', colOrange, 'DisplayName','East (missing)');
        plot(ax1, tPos, zeros(size(tPos)), '-', 'Color', colYellow, 'DisplayName','Down (missing)');
        add_missing_annotation(ax1, fs);
    end
    ylabel(ax1,'Position [m]','FontSize',fs);
    legend(ax1,'Location','north'); axis(ax1,'tight');
    draw_blowups(ax1, blowup_times, colRed);

    % Row 2: Velocity
    ax2 = subplot(3,1,2); hold(ax2,'on'); grid(ax2,'on'); set(ax2,'FontSize',fs);
    if hasVel
        plot(ax2, tVel, velN(:,1), '-', 'Color', colBlue,   'DisplayName','North');
        plot(ax2, tVel, velN(:,2), '-', 'Color', colOrange, 'DisplayName','East');
        plot(ax2, tVel, velN(:,3), '-', 'Color', colYellow, 'DisplayName','Down');
    else
        plot(ax2, tVel, zeros(size(tVel)), '-', 'Color', colBlue,   'DisplayName','North (missing)');
        plot(ax2, tVel, zeros(size(tVel)), '-', 'Color', colOrange, 'DisplayName','East (missing)');
        plot(ax2, tVel, zeros(size(tVel)), '-', 'Color', colYellow, 'DisplayName','Down (missing)');
        add_missing_annotation(ax2, fs);
    end
    ylabel(ax2,'Velocity [m/s]','FontSize',fs);
    legend(ax2,'Location','north'); axis(ax2,'tight');
    draw_blowups(ax2, blowup_times, colRed);

    % Row 3: Acceleration
    ax3 = subplot(3,1,3); hold(ax3,'on'); grid(ax3,'on'); set(ax3,'FontSize',fs);
    if hasAcc
        plot(ax3, tAcc, accN(:,1), '-', 'Color', colBlue,   'DisplayName','North');
        plot(ax3, tAcc, accN(:,2), '-', 'Color', colOrange, 'DisplayName','East');
        plot(ax3, tAcc, accN(:,3), '-', 'Color', colYellow, 'DisplayName','Down');
    else
        plot(ax3, tAcc, zeros(size(tAcc)), '-', 'Color', colBlue,   'DisplayName','North (missing)');
        plot(ax3, tAcc, zeros(size(tAcc)), '-', 'Color', colOrange, 'DisplayName','East (missing)');
        plot(ax3, tAcc, zeros(size(tAcc)), '-', 'Color', colYellow, 'DisplayName','Down (missing)');
        add_missing_annotation(ax3, fs);
    end
    ylabel(ax3,'Acceleration [m/s²]','FontSize',fs);
    xlabel(ax3,'Time [s]','FontSize',fs);
    legend(ax3,'Location','north'); axis(ax3,'tight');
    draw_blowups(ax3, blowup_times, colRed);

    % Save
    set(hFig,'Units','pixels','Position',[100 100 1800 1200]);
    outPath = fullfile(outDir,'IMU_X002_GNSS_X002_TRIAD_task5_fused_ned.png');
    exportgraphics(hFig, outPath, 'Resolution', 200);
    info = dir(outPath);
    if isempty(info) || info.bytes < 5000
        error('Save failed: %s', 'IMU_X002_GNSS_X002_TRIAD_task5_fused_ned.png');
    end
    fprintf('[SAVE] %s (%d bytes)\n', info.name, info.bytes);
    close(hFig);
catch ME
    if exist('hFig','var') && isvalid(hFig), close(hFig); end
    warning('Failed to generate main NED fused figure: %s', ME.message);
end

% ---------------------- Comparison figures (per frame) ------------------
frames = {'mixed','ned','ecef','body'};
axisLabels = {
    {'Axis 1','Axis 2','Axis 3'}, ... % mixed
    {'North','East','Down'}, ...      % NED
    {'X','Y','Z'}, ...                % ECEF
    {'BX','BY','BZ'}                  % Body
};
fileNames = {
    'IMU_X002_GNSS_X002_TRIAD_task5_comparison_mixed.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task5_comparison_ned.png',   ...
    'IMU_X002_GNSS_X002_TRIAD_task5_comparison_ecef.png',  ...
    'IMU_X002_GNSS_X002_TRIAD_task5_comparison_body.png'   ...
};

for k = 1:numel(frames)
    frame = frames{k};
    try
        hFig = figure('Units','pixels','Position',[100 100 1800 1200], 'Color','w', 'Visible','off');
        line1 = sprintf('Task 5: Fused vs Raw in %s frame', upper(frame));
        sgtitle({line1; subtitle_line2_cmp}, 'FontSize', fs, 'FontWeight','bold');

        % Frame data
        fFrame = getframefield(fused_data, frame);
        rFrame = getframefield(raw_data,   frame);

        quantities = {'pos','vel','acc'};
        rowNames   = {'Position','Velocity','Acceleration'};
        units      = {'[m]','[m/s]','[m/s²]'};

        for r = 1:3
            qty = quantities{r};
            for c = 1:3
                ax = subplot(3,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',fs);
                axisName = axisLabels{k}{c};

                % Extract columns
                [fCol, hasF] = getqtycol(fFrame, qty, c);
                [rCol, hasR] = getqtycol(rFrame, qty, c);

                tF = resolvetime(time_vec, numel(fCol), 'fused');
                tR = resolvetime(time_vec, numel(rCol), 'raw');

                % Plot Raw IMU (red dashed) and Fused (blue solid)
                if hasR
                    plot(ax, tR, rCol, '--', 'Color', colRed, 'LineWidth', 1.0, 'DisplayName', 'IMU raw');
                else
                    plot(ax, tR, zeros(size(tR)), '--', 'Color', colRed, 'LineWidth', 1.0, 'DisplayName', 'IMU raw (missing)');
                    add_missing_annotation(ax, fs);
                end
                if hasF
                    plot(ax, tF, fCol, '-', 'Color', colBlue, 'LineWidth', 1.5, 'DisplayName', 'Fused');
                else
                    plot(ax, tF, zeros(size(tF)), '-', 'Color', colBlue, 'LineWidth', 1.5, 'DisplayName', 'Fused (missing)');
                    add_missing_annotation(ax, fs);
                end

                legend(ax,'Location','north'); axis(ax,'tight');
                ylabel(ax, sprintf('%s %s %s', axisName, rowNames{r}, units{r}), 'FontSize', fs);
                if r == 3, xlabel(ax,'Time [s]','FontSize',fs); end

                draw_blowups(ax, blowup_times, colRed);
            end
        end

        % Save
        set(hFig,'Units','pixels','Position',[100 100 1800 1200]);
        outPath = fullfile(outDir, fileNames{k});
        exportgraphics(hFig, outPath, 'Resolution', 200);
        info = dir(outPath);
        if isempty(info) || info.bytes < 5000
            error('Save failed: %s', fileNames{k});
        end
        fprintf('[SAVE] %s (%d bytes)\n', info.name, info.bytes);
        close(hFig);
    catch ME
        if exist('hFig','var') && isvalid(hFig), close(hFig); end
        warning('Failed to generate comparison figure for %s: %s', upper(frame), ME.message);
    end
end

% Close any remaining open figures
close all;

end

% ------------------------------ Helpers -------------------------------

function frameStruct = getframefield(dataStruct, frame)
% Safely fetch frame sub-struct or return empty struct
frameStruct = struct();
try
    if isstruct(dataStruct) && isfield(dataStruct, frame) && isstruct(dataStruct.(frame))
        frameStruct = dataStruct.(frame);
    end
catch
    frameStruct = struct();
end
end

function [arr, has] = getqty(frameStruct, qty)
% Get full array for a quantity if valid
has = false; arr = zeros(0,3);
try
    if isstruct(frameStruct) && isfield(frameStruct, qty)
        cand = frameStruct.(qty);
        if isnumeric(cand) && ndims(cand)==2 && size(cand,2)>=3
            arr = cand;
            has = true;
        end
    end
catch
    has = false; arr = zeros(0,3);
end
end

function [col, has] = getqtycol(frameStruct, qty, idx)
% Get a single column if present
has = false; col = zeros(0,1);
try
    if isstruct(frameStruct) && isfield(frameStruct, qty)
        arr = frameStruct.(qty);
        if isnumeric(arr) && ndims(arr)==2 && size(arr,2)>=idx
            col = arr(:,idx);
            has = true;
        end
    end
catch
    has = false; col = zeros(0,1);
end
end

function t = resolvetime(tv, n, role)
% Resolve a time vector of length n for role 'fused' or 'raw'
t0 = 0; T = 1400;
if nargin < 3 || isempty(role), role = 'fused'; end
try
    if isstruct(tv)
        if isfield(tv, role) && isnumeric(tv.(role)) && isvector(tv.(role))
            tr = tv.(role)(:);
        elseif isfield(tv, ['t_' role]) && isnumeric(tv.(['t_' role])) && isvector(tv.(['t_' role]))
            tr = tv.(['t_' role])(:);
        else
            tr = [];
        end
        if ~isempty(tr)
            if numel(tr) == n
                t = tr; return;
            end
            t0 = tr(1); T = tr(end);
        end
    elseif isnumeric(tv) && isvector(tv)
        tv = tv(:);
        if numel(tv) == n
            t = tv; return;
        end
        t0 = tv(1); T = tv(end);
    elseif isnumeric(tv) && isscalar(tv)
        T = tv;
    end
catch
    % use defaults
end
if n <= 1
    t = linspace(t0, T, max(n,2)).';
    t = t(1:n);
else
    t = linspace(t0, T, n).';
end
end

function add_missing_annotation(ax, fs)
% Add a red missing data label
try
    text(ax, 0.02, 0.88, '\x26A0 Missing data', 'Units','normalized', ...
        'Color',[0.85 0.1 0.1], 'FontWeight','bold', 'FontSize', fs);
catch
    text(ax, 0.02, 0.88, 'Missing data', 'Units','normalized', ...
        'Color',[0.85 0.1 0.1], 'FontWeight','bold', 'FontSize', fs);
end
end

function draw_blowups(ax, blowup_times, col)
% Draw vertical lines at provided times
if nargin < 3 || isempty(col)
    col = [0.85 0.1 0.1];
end
if isempty(blowup_times) || ~isnumeric(blowup_times)
    return;
end
bt = blowup_times(:)';
for t = bt
    try
        xline(ax, t, '-', 'Color', col, 'LineWidth', 0.75, 'HandleVisibility','off');
    catch
        % older MATLAB without xline; fallback
        yl = ylim(ax);
        plot(ax, [t t], yl, '-', 'Color', col, 'LineWidth', 0.75, 'HandleVisibility','off');
    end
end
end

function bt = extract_blowups(tv, fused)
% Attempt to find blow-up times (seconds) from inputs
bt = [];
try
    if isstruct(fused)
        if isfield(fused,'blowups') && isnumeric(fused.blowups)
            bt = fused.blowups(:)';
            return;
        elseif isfield(fused,'blowup_times') && isnumeric(fused.blowup_times)
            bt = fused.blowup_times(:)';
            return;
        end
    end
    if isstruct(tv)
        if isfield(tv,'blowups') && isnumeric(tv.blowups)
            bt = tv.blowups(:)';
            return;
        elseif isfield(tv,'t_blowups') && isnumeric(tv.t_blowups)
            bt = tv.t_blowups(:)';
            return;
        end
    end
catch
    bt = [];
end
end

