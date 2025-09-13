function plot_any(input)
% plot_any - Generic MATLAB plotter for Task 1–7 .mat outputs
%
% Usage:
%   plot_any()                 % choose a .mat interactively
%   plot_any('path/to/file.mat')
%
% This helper inspects the loaded variables and renders an appropriate plot
% for common Task 1–7 artifacts. It is intentionally defensive: if an
% expected key is missing it will skip that subplot gracefully.

% Allow flexible launch:
% - No arg: auto-pick a single .mat in cwd or prompt user
% - Drag-and-drop path string: treat as filename
if nargin < 1 || isempty(input)
    mats = dir('*.mat');
    if numel(mats) == 1
        file = fullfile(mats(1).folder, mats(1).name);
    else
        [f,p] = uigetfile('*.mat','Select a .mat results file');
        if isequal(f,0); return; end
        file = fullfile(p,f);
    end
else
    % If user dragged a file path into the command window, it comes as char/string
    if isstring(input) || ischar(input)
        file = char(input);
    else
        error('plot_any:badarg','Unsupported input type');
    end
end

S = load(file);
fn = fieldnames(S);
fprintf('[plot_any] Loaded %s with fields: %s\n', file, strjoin(fn', ', '));

% --- Task 6/7: overlay bundles (2x3 Position/Velocity, Fused vs Truth) ----
% Detect a struct named 'data' with fields time, pos_fused/vel_fused and
% optional pos_truth/vel_truth. These are produced by Task 6/7 overlay tools.
if ismember('data', fn) && isstruct(S.data)
    D = S.data;
    has_time = isfield(D,'time');
    has_pf = isfield(D,'pos_fused') || isfield(D,'vel_fused');
    if has_time && has_pf
        t = D.time(:);
        pos_fused  = getf(D,'pos_fused');
        vel_fused  = getf(D,'vel_fused');
        pos_truth  = getf(D,'pos_truth');
        vel_truth  = getf(D,'vel_truth');
        frame = infer_frame_from_name(file);
        comps = frame_labels(frame);
        figure('Name','Task 6/7 Overlay','Color','w');
        plot_frame_2x3(t, pos_fused, vel_fused, comps, frame, pos_truth, vel_truth);
        [fp, fnm] = fileparts(file); %#ok<ASGLU>
        sgtitle(sprintf('Truth vs Fused (%s) — %s', frame, fnm),'Interpreter','none');
        % Save PNG alongside
        try
            outPng = fullfile(fp, [fnm '.png']);
            exportgraphics(gcf, outPng, 'Resolution', 200);
            fprintf('[plot_any] Saved PNG -> %s\n', outPng);
        catch
            try, saveas(gcf, outPng); fprintf('[plot_any] Saved PNG -> %s\n', outPng); end
        end
        return;
    end
end

% --- Task 2: static interval (acc/gyro norms) -----------------------------
if all(ismember({'t','acc_norm','gyro_norm'}, fn))
    t = S.t(:);
    accn = S.acc_norm(:);
    gyron = S.gyro_norm(:);
    figure('Name','Task 2: Static interval','Color','w');
    tiledlayout(2,1);
    nexttile; plot(t, accn,'-'); grid on; ylabel('|acc| [m/s^2]');
    title('Task 2: Accel norm');
    hold on;
    if isfield(S,'static_start') && isfield(S,'static_end')
        xline(S.static_start,'r--','start'); xline(S.static_end,'r--','end');
    end
    nexttile; plot(t, gyron,'-'); grid on; ylabel('|gyro| [rad/s]'); xlabel('Time [s]');
    title('Task 2: Gyro norm');
    return;
end

% --- Task 3: attitude init error bars -------------------------------------
if all(ismember({'methods','grav_err_deg','earth_err_deg'}, fn))
    m = string(S.methods(:));
    ge = S.grav_err_deg(:); oe = S.earth_err_deg(:);
    figure('Name','Task 3: Init errors','Color','w');
    tiledlayout(1,2);
    nexttile; bar(categorical(m), ge); grid on; ylabel('deg'); title('Gravity alignment error');
    nexttile; bar(categorical(m), oe); grid on; ylabel('deg'); title('Earth-rate alignment error');
    return;
end

% --- Task 3: quaternion values (case snapshot) ----------------------------
caseKeys = {'TRIAD_Case1','TRIAD_Case2','Davenport_Case1','Davenport_Case2','SVD_Case1','SVD_Case2'};
if any(ismember(caseKeys, fn))
    figure('Name','Task 3: Initial quaternions','Color','w');
    rows = 3; cols = 2; k = 1;
    for i = 1:numel(caseKeys)
        ck = caseKeys{i};
        if isfield(S, ck)
            q = S.(ck)(:).'; % expect [w x y z]
            subplot(rows,cols,k); bar(q); grid on;
            set(gca,'XTickLabel',{'w','x','y','z'});
            title(ck,'Interpreter','none');
            k = k + 1;
        end
    end
    return;
end

% --- Task 5: fused results in NED/ECEF/Body (time series) -----------------
% Expected keys (subset): time_s, pos_ned_m, vel_ned_ms, pos_ecef_m, vel_ecef_ms,
% pos_body_m, vel_body_ms
if isfield(S,'time_s')
    t = S.time_s(:);
    hasNED = isfield(S,'pos_ned_m') || isfield(S,'vel_ned_ms');
    hasECEF = isfield(S,'pos_ecef_m') || isfield(S,'vel_ecef_ms');
    hasBODY = isfield(S,'pos_body_m') || isfield(S,'vel_body_ms');

    % Prefer plotting only the matching frame for this file (one figure, 3x3)
    [fp, fn, fe] = fileparts(file); %#ok<ASGLU>
    lowerfn = lower(fn);
    if contains(lowerfn,'task5_all_ned') && hasNED
        figure('Name','Task 5: NED (Fused)','Color','w');
        plot_frame_2x3(t, getf(S,'pos_ned_m'), getf(S,'vel_ned_ms'), {'N','E','D'}, 'NED');
        sgtitle(sprintf('Task 5 – NED (Fused) — %s', fn),'Interpreter','none');
    elseif contains(lowerfn,'task5_all_ecef') && hasECEF
        figure('Name','Task 5: ECEF (Fused)','Color','w');
        plot_frame_2x3(t, getf(S,'pos_ecef_m'), getf(S,'vel_ecef_ms'), {'X','Y','Z'}, 'ECEF');
        sgtitle(sprintf('Task 5 – ECEF (Fused) — %s', fn),'Interpreter','none');
    elseif contains(lowerfn,'task5_all_body') && hasBODY
        figure('Name','Task 5: BODY (Fused)','Color','w');
        plot_frame_2x3(t, getf(S,'pos_body_m'), getf(S,'vel_body_ms'), {'X','Y','Z'}, 'BODY');
        sgtitle(sprintf('Task 5 – BODY (Fused) — %s', fn),'Interpreter','none');
    else
        % Fallback: pick the first available frame in priority order
        if hasNED
            figure('Name','Task 5: NED (Fused)','Color','w');
            plot_frame_2x3(t, getf(S,'pos_ned_m'), getf(S,'vel_ned_ms'), {'N','E','D'}, 'NED');
            sgtitle(sprintf('Task 5 – NED (Fused) — %s', fn),'Interpreter','none');
        elseif hasECEF
            figure('Name','Task 5: ECEF (Fused)','Color','w');
            plot_frame_2x3(t, getf(S,'pos_ecef_m'), getf(S,'vel_ecef_ms'), {'X','Y','Z'}, 'ECEF');
            sgtitle(sprintf('Task 5 – ECEF (Fused) — %s', fn),'Interpreter','none');
        elseif hasBODY
            figure('Name','Task 5: BODY (Fused)','Color','w');
            plot_frame_2x3(t, getf(S,'pos_body_m'), getf(S,'vel_body_ms'), {'X','Y','Z'}, 'BODY');
            sgtitle(sprintf('Task 5 – BODY (Fused) — %s', fn),'Interpreter','none');
        else
            warning('plot_any:task5missing','Task 5 fused fields not found in %s', file);
        end
    end
    % Save PNG alongside the .mat with same base name
    try
        outPng = fullfile(fp, [fn '.png']);
        exportgraphics(gcf, outPng, 'Resolution', 200);
        fprintf('[plot_any] Saved PNG -> %s\n', outPng);
    catch
        try, saveas(gcf, outPng); fprintf('[plot_any] Saved PNG -> %s\n', outPng); end
    end
    return;
end

warning('plot_any:unhandled','No Task 1–5 handler matched for %s', file);

end

% ---- helpers --------------------------------------------------------------
function v = getf(S, k)
    if isfield(S,k)
        v = S.(k);
    else
        v = [];
    end
end

function frame = infer_frame_from_name(file)
    [~, fn] = fileparts(file);
    lowerfn = lower(fn);
    if contains(lowerfn,'ecef')
        frame = 'ECEF'; return;
    elseif contains(lowerfn,'body')
        frame = 'BODY'; return;
    else
        frame = 'NED'; return;
    end
end

function comps = frame_labels(frame)
    switch upper(string(frame))
        case 'ECEF', comps = {'X','Y','Z'};
        case 'BODY', comps = {'X','Y','Z'};
        otherwise,  comps = {'N','E','D'};
    end
end

function plot_frame_2x3(t, pos, vel, comps, frame, pos_truth, vel_truth)
    % Create a 2x3 grid: rows = Position/Velocity, cols = components
    % If optional truth arrays are provided, overlay them with dashed lines.
    if nargin < 6, pos_truth = []; end
    if nargin < 7, vel_truth = []; end

    clf; tl = tiledlayout(2,3,'Padding','compact','TileSpacing','compact'); %#ok<NASGU>
    havePos = ~isempty(pos) && size(pos,2) == 3;
    haveVel = ~isempty(vel) && size(vel,2) == 3;
    havePosT = ~isempty(pos_truth) && size(pos_truth,2) == 3;
    haveVelT = ~isempty(vel_truth) && size(vel_truth,2) == 3;

    % Decimate if extremely long to keep rendering responsive
    n = numel(t);
    stride = max(1, ceil(n/200000));
    ti = t(1:stride:end);
    take = @(A) (isempty(A) * [] + ~isempty(A) * (A(1:stride:end, :))); %#ok<NASGU>
    pf = pos; vf = vel; pt = pos_truth; vt = vel_truth;
    if stride > 1
        if ~isempty(pf), pf = pf(1:stride:end, :); end
        if ~isempty(vf), vf = vf(1:stride:end, :); end
        if ~isempty(pt), pt = pt(1:stride:end, :); end
        if ~isempty(vt), vt = vt(1:stride:end, :); end
    end

    hfused = []; htruth = [];
    for j = 1:3
        % Position row
        nexttile(j);
        hold on; grid on;
        if havePos
            h = plot(ti, pf(:,j), 'r-', 'LineWidth',1.2, 'Marker','x','MarkerSize',3, 'DisplayName','Fused');
            if j==1, hfused = h; end
        end
        if havePosT
            h = plot(ti, pt(:,j), 'k:', 'Marker','*','MarkerSize',3, 'DisplayName','Truth');
            if j==1, htruth = h; end
        end
        title(sprintf('%s %s', frame, comps{j})); ylabel(sprintf('Position %s [m]', frame));
        % Velocity row
        nexttile(3+j);
        hold on; grid on;
        if haveVel, plot(ti, vf(:,j), 'r-', 'LineWidth',1.2, 'Marker','x','MarkerSize',3, 'DisplayName','Fused'); end
        if haveVelT, plot(ti, vt(:,j), 'k:', 'Marker','*','MarkerSize',3, 'DisplayName','Truth'); end
        ylabel(sprintf('Velocity %s [m/s]', frame)); xlabel('Time [s]');
    end
    % Shared legend for the layout using first column handles
    try
        legs = []; names = {};
        if ~isempty(hfused), legs = [legs hfused]; names{end+1} = 'Fused'; end %#ok<AGROW>
        if ~isempty(htruth), legs = [legs htruth]; names{end+1} = 'Truth'; end %#ok<AGROW>
        if ~isempty(legs)
            lgd = legend(legs, names, 'Orientation','horizontal');
            try, lgd.Layout.Tile = 'north'; catch, set(lgd,'Location','northoutside'); end
        end
    catch
    end

    % Harmonize symmetric y-limits using robust percentile when truth present
    try
        if havePos || havePosT
            lim_p = prctile(abs([pf(:); pt(:)]), 99.5);
            if isfinite(lim_p) && lim_p>0
                for i=1:3, ylim(subplot(2,3,i), [-lim_p lim_p]); end
            end
        end
        if haveVel || haveVelT
            v_all = [vf(:); vt(:)]; v_all = v_all(isfinite(v_all));
            if ~isempty(v_all)
                lim_v = prctile(abs(v_all), 99.5);
                if isfinite(lim_v) && lim_v>0
                    for i=1:3, ylim(subplot(2,3,3+i), [-lim_v lim_v]); end
                end
            end
        end
    catch, end
end

function plot_frame_3x3(t, pos, vel, acc, comps, frame)
    % Create a 3x3 grid: rows = Position/Velocity/Acceleration, cols = components
    % Each subplot shows a single component over time.
    clf; tl = tiledlayout(3,3,'Padding','compact','TileSpacing','compact'); %#ok<NASGU>
    havePos = ~isempty(pos) && size(pos,2) == 3;
    haveVel = ~isempty(vel) && size(vel,2) == 3;
    haveAcc = ~isempty(acc) && size(acc,2) == 3;
    labels = {sprintf('Position %s [m]', frame), sprintf('Velocity %s [m/s]', frame), sprintf('Acceleration %s [m/s^2]', frame)};
    for j = 1:3
        % Position row
        nexttile((1-1)*3 + j);
        if havePos, plot(t, pos(:,j), '-'); grid on; else, text(0.5,0.5,'N/A','HorizontalAlignment','center'); axis off; end
        title(sprintf('%s %s', frame, comps{j})); if j==1, ylabel(labels{1}); end
        % Velocity row
        nexttile((2-1)*3 + j);
        if haveVel, plot(t, vel(:,j), '-'); grid on; else, text(0.5,0.5,'N/A','HorizontalAlignment','center'); axis off; end
        if j==1, ylabel(labels{2}); end
        % Acceleration row
        nexttile((3-1)*3 + j);
        if haveAcc, plot(t, acc(:,j), '-'); grid on; else, text(0.5,0.5,'N/A','HorizontalAlignment','center'); axis off; end
        if j==1, ylabel(labels{3}); end; xlabel('Time [s]');
    end
end
% (old plot_vec3 helper removed; replaced by plot_frame_3x3)
