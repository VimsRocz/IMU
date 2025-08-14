function task6_overlay_truth(fused_data, truth_data, time_vec)
%TASK6_OVERLAY_TRUTH Plot fused vs truth differences and error norms.
%
% function task6_overlay_truth(fused_data, truth_data, time_vec)
% Inputs
%  - fused_data: struct with frames .ned/.ecef/.body having .pos/.vel/.acc [N x 3]
%  - truth_data: same frame structure with ground truth arrays [N x 3]
%  - time_vec: numeric vector, scalar duration, or struct with fields
%               'fused'/'truth' (or 't_fused'/'t_truth').
%
% Saves 4 PNGs into MATLAB/results per the spec, with size checks.

outDir = fullfile('MATLAB','results');
if ~exist(outDir,'dir'); mkdir(outDir); end

fs = 12;
colDiff = [0.000, 0.447, 0.741]; % blue
colThr  = [0.85,  0.10,  0.10];  % red

frames = {'ned','ecef','body'};
axisLabels = {
    {'North','East','Down'}, ...
    {'X','Y','Z'}, ...
    {'BX','BY','BZ'}
};
fileNames = {
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_ned.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_ecef.png', ...
    'IMU_X002_GNSS_X002_TRIAD_task6_diff_body.png'  ...
};

for k = 1:numel(frames)
    frame = frames{k};
    try
        fFrame = getframefield(fused_data, frame);
        tFrame = getframefield(truth_data, frame);

        % Differences (align lengths by truncation)
        [posDiff, hasPos] = diff_aligned(fFrame, tFrame, 'pos');
        [velDiff, hasVel] = diff_aligned(fFrame, tFrame, 'vel');

        % Time vectors (use fused timeline length where available)
        nPos = size(posDiff,1); nVel = size(velDiff,1);
        tPos = resolvetime(time_vec, nPos, 'fused');
        tVel = resolvetime(time_vec, nVel, 'fused');

        hFig = figure('Units','pixels','Position',[100 100 1800 1200], 'Color','w', 'Visible','off');
        sgtitle(sprintf('Task 6: Fused vs Truth Diffs in %s | IMU_X002_GNSS_X002_TRIAD', upper(frame)), ...
            'FontSize', fs, 'FontWeight','bold');

        rowNames = {'Position Diff [m]','Velocity Diff [m/s]'};
        for r = 1:2
            for c = 1:3
                ax = subplot(2,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on'); set(ax,'FontSize',fs);
                axisName = axisLabels{k}{c};
                if r == 1
                    if hasPos
                        plot(ax, tPos, posDiff(:,c), '-', 'Color', colDiff, 'LineWidth', 1.25, 'DisplayName','Diff');
                        nExc = sum(abs(posDiff(:,c)) > 1);
                    else
                        plot(ax, tPos, zeros(size(tPos)), '-', 'Color', colDiff, 'LineWidth', 1.25, 'DisplayName','Diff (missing)');
                        nExc = 0; add_missing_annotation(ax, fs);
                    end
                    thr = 1; yline(ax, thr, '-', 'Color', colThr, 'HandleVisibility','off'); yline(ax, -thr, '-', 'Color', colThr, 'HandleVisibility','off');
                    title(ax, sprintf('%s %s', axisName, rowNames{r}), 'FontSize', fs);
                else
                    if hasVel
                        plot(ax, tVel, velDiff(:,c), '-', 'Color', colDiff, 'LineWidth', 1.25, 'DisplayName','Diff');
                        nExc = sum(abs(velDiff(:,c)) > 1);
                    else
                        plot(ax, tVel, zeros(size(tVel)), '-', 'Color', colDiff, 'LineWidth', 1.25, 'DisplayName','Diff (missing)');
                        nExc = 0; add_missing_annotation(ax, fs);
                    end
                    thr = 1; yline(ax, thr, '-', 'Color', colThr, 'HandleVisibility','off'); yline(ax, -thr, '-', 'Color', colThr, 'HandleVisibility','off');
                    title(ax, sprintf('%s %s', axisName, rowNames{r}), 'FontSize', fs);
                end

                if r == 2
                    xlabel(ax,'Time [s]','FontSize',fs);
                end
                % Exceedance annotation
                text(ax, 0.02, 0.90, sprintf('Samples exceeding 1m/1m/s: %d', nExc), ...
                    'Units','normalized','Color',colThr,'FontSize',fs,'FontWeight','bold');

                axis(ax,'tight');
            end
        end

        % Save figure
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
        warning('Failed to generate Task 6 diffs for %s: %s', upper(frame), ME.message);
    end
end

% ---------------------- Error Norms Figure (NED priority) ---------------
% Choose frame for norms (NED -> ECEF -> BODY)
frameOrder = {'ned','ecef','body'};
chosen = '';
for i = 1:numel(frameOrder)
    if isfield(fused_data, frameOrder{i}) && isfield(truth_data, frameOrder{i})
        chosen = frameOrder{i}; break;
    end
end
if isempty(chosen); chosen = 'ned'; end % fall back to ned naming

try
    fFrame = getframefield(fused_data, chosen);
    tFrame = getframefield(truth_data, chosen);
    [posDiff, hasPos] = diff_aligned(fFrame, tFrame, 'pos');
    [velDiff, hasVel] = diff_aligned(fFrame, tFrame, 'vel');
    [accDiff, hasAcc] = diff_aligned(fFrame, tFrame, 'acc');
    tPos = resolvetime(time_vec, size(posDiff,1), 'fused');
    tVel = resolvetime(time_vec, size(velDiff,1), 'fused');
    tAcc = resolvetime(time_vec, size(accDiff,1), 'fused');

    posNorm = nrm(posDiff);
    velNorm = nrm(velDiff);
    accNorm = nrm(accDiff);

    hFig = figure('Units','pixels','Position',[100 100 1800 1200], 'Color','w', 'Visible','off');
    sgtitle('Task 6: Error Norms | IMU_X002_GNSS_X002_TRIAD', 'FontSize', fs, 'FontWeight','bold');

    ax1 = subplot(3,1,1); hold(ax1,'on'); grid(ax1,'on'); set(ax1,'FontSize',fs);
    if hasPos, plot(ax1, tPos, posNorm, '-', 'Color', colDiff, 'DisplayName','Error Norm');
    else, plot(ax1, tPos, zeros(size(tPos)), '-', 'Color', colDiff, 'DisplayName','Error Norm (missing)'); add_missing_annotation(ax1, fs); end
    ylabel(ax1,'Position Norm [m]','FontSize',fs); legend(ax1,'Location','north'); axis(ax1,'tight');

    ax2 = subplot(3,1,2); hold(ax2,'on'); grid(ax2,'on'); set(ax2,'FontSize',fs);
    if hasVel, plot(ax2, tVel, velNorm, '-', 'Color', colDiff, 'DisplayName','Error Norm');
    else, plot(ax2, tVel, zeros(size(tVel)), '-', 'Color', colDiff, 'DisplayName','Error Norm (missing)'); add_missing_annotation(ax2, fs); end
    ylabel(ax2,'Velocity Norm [m/s]','FontSize',fs); legend(ax2,'Location','north'); axis(ax2,'tight');

    ax3 = subplot(3,1,3); hold(ax3,'on'); grid(ax3,'on'); set(ax3,'FontSize',fs);
    if hasAcc, plot(ax3, tAcc, accNorm, '-', 'Color', colDiff, 'DisplayName','Error Norm');
    else, plot(ax3, tAcc, zeros(size(tAcc)), '-', 'Color', colDiff, 'DisplayName','Error Norm (missing)'); add_missing_annotation(ax3, fs); end
    ylabel(ax3,'Acceleration Norm [m/s²]','FontSize',fs); xlabel(ax3,'Time [s]','FontSize',fs); legend(ax3,'Location','north'); axis(ax3,'tight');

    % Save
    set(hFig,'Units','pixels','Position',[100 100 1800 1200]);
    outPath = fullfile(outDir, 'IMU_X002_GNSS_X002_TRIAD_task6_error_norms.png');
    exportgraphics(hFig, outPath, 'Resolution', 200);
    info = dir(outPath);
    if isempty(info) || info.bytes < 5000
        error('Save failed: %s', 'IMU_X002_GNSS_X002_TRIAD_task6_error_norms.png');
    end
    fprintf('[SAVE] %s (%d bytes)\n', info.name, info.bytes);
    close(hFig);
catch ME
    if exist('hFig','var') && isvalid(hFig), close(hFig); end
    warning('Failed to generate Task 6 error norms: %s', ME.message);
end

% Close any remaining open figures
close all;

end

% ----------------------------- Helpers ---------------------------------

function frameStruct = getframefield(dataStruct, frame)
frameStruct = struct();
try
    if isstruct(dataStruct) && isfield(dataStruct, frame) && isstruct(dataStruct.(frame))
        frameStruct = dataStruct.(frame);
    end
catch
    frameStruct = struct();
end
end

function [diffs, has] = diff_aligned(fFrame, tFrame, qty)
diffs = zeros(0,3); has = false;
try
    if isstruct(fFrame) && isstruct(tFrame) && isfield(fFrame, qty) && isfield(tFrame, qty)
        A = fFrame.(qty); B = tFrame.(qty);
        if isnumeric(A) && isnumeric(B) && size(A,2)>=3 && size(B,2)>=3
            n = min(size(A,1), size(B,1));
            diffs = A(1:n,1:3) - B(1:n,1:3);
            has = true;
        end
    end
catch
    diffs = zeros(0,3); has = false;
end
end

function t = resolvetime(tv, n, role)
t0 = 0; T = 1400; if nargin<3||isempty(role), role='fused'; end
try
    if isstruct(tv)
        if isfield(tv, role) && isnumeric(tv.(role)) && isvector(tv.(role))
            tr = tv.(role)(:); if numel(tr)==n, t=tr; return; else, t0=tr(1); T=tr(end); end
        elseif isfield(tv, ['t_' role]) && isnumeric(tv.(['t_' role])) && isvector(tv.(['t_' role]))
            tr = tv.(['t_' role])(:); if numel(tr)==n, t=tr; return; else, t0=tr(1); T=tr(end); end
        end
    elseif isnumeric(tv) && isvector(tv)
        tv=tv(:); if numel(tv)==n, t=tv; return; else, t0=tv(1); T=tv(end); end
    elseif isnumeric(tv) && isscalar(tv)
        T = tv;
    end
catch
end
if n<=1, t = linspace(t0,T,max(n,2)).'; t=t(1:n); else, t = linspace(t0,T,n).'; end
end

function add_missing_annotation(ax, fs)
try
    text(ax, 0.02, 0.90, '\x26A0 Missing data', 'Units','normalized', 'Color',[0.85 0.1 0.1], 'FontWeight','bold', 'FontSize', fs);
catch
    text(ax, 0.02, 0.90, 'Missing data', 'Units','normalized', 'Color',[0.85 0.1 0.1], 'FontWeight','bold', 'FontSize', fs);
end
end

function v = nrm(M)
if isempty(M)
    v = zeros(0,1);
else
    v = sqrt(sum(M.^2, 2));
end
end

