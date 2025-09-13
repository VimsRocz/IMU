function plot_any(input)
% plot_any - Generic MATLAB plotter for Task 1–5 .mat outputs from the Python pipeline
%
% Usage:
%   plot_any()                 % choose a .mat interactively
%   plot_any('path/to/file.mat')
%
% This helper inspects the loaded variables and renders an appropriate plot
% for common Task 1–5 artifacts. It is intentionally defensive: if an
% expected key is missing it will skip that subplot gracefully.

if nargin < 1 || isempty(input)
    [f,p] = uigetfile('*.mat','Select a .mat results file');
    if isequal(f,0); return; end
    file = fullfile(p,f);
else
    file = input;
end

S = load(file);
fn = fieldnames(S);
fprintf('[plot_any] Loaded %s with fields: %s\n', file, strjoin(fn', ', '));

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

    if hasNED
        figure('Name','Task 5: NED (Fused)','Color','w');
        plot_frame_3x3(t, getf(S,'pos_ned_m'), getf(S,'vel_ned_ms'), getf(S,'acc_ned'), {'N','E','D'}, 'NED');
    end
    if hasECEF
        figure('Name','Task 5: ECEF (Fused)','Color','w');
        plot_frame_3x3(t, getf(S,'pos_ecef_m'), getf(S,'vel_ecef_ms'), getf(S,'acc_ecef'), {'X','Y','Z'}, 'ECEF');
    end
    if hasBODY
        figure('Name','Task 5: BODY (Fused)','Color','w');
        plot_frame_3x3(t, getf(S,'pos_body_m'), getf(S,'vel_body_ms'), getf(S,'acc_body'), {'X','Y','Z'}, 'BODY');
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
