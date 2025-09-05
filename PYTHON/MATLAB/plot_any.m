function plot_any(file_path)
% plot_any Render or open a plot from a single input file.
%   plot_any()                      -> prompts for a file
%   plot_any('path/to/file.ext')    -> renders/opens based on extension
%
% Supported:
%   - .mat  :
%       * Overlay bundles with struct 'data' (time,pos/vel fused/truth)
%       * Saved axis bundles with keys like ax1_line1_x/ax1_line1_y
%       * Generic numeric variables (vectors/matrices)
%   - .fig  : open MATLAB figure
%   - .png/.jpg/.jpeg : show image
%   - .pdf  : open via system/desktop (cannot render in axes)
%
% The overlay plot style matches Task 7.6 PNGs: 2x3 grid, Fused solid,
% Truth dashed, harmonised symmetric y-limits (99.5th percentile), decimation.

if nargin < 1 || isempty(file_path)
    [f,p] = uigetfile({'*.mat;*.fig;*.png;*.jpg;*.jpeg;*.pdf','All supported files'}, ...
                      'Select a plot file');
    if isequal(f,0), return; end
    file_path = fullfile(p,f);
end

if exist(file_path,'file') ~= 2
    error('File not found: %s', file_path);
end

[~, base, ext] = fileparts(file_path);
ext = lower(ext);

switch ext
    case '.fig'
        openfig(file_path, 'new', 'visible');
        return
    case {'.png','.jpg','.jpeg'}
        img = imread(file_path);
        figure('Name', base, 'Color','w');
        imshow(img, 'Border','tight');
        title(base, 'Interpreter','none');
        return
    case '.pdf'
        try
            open(file_path);  % defer to desktop PDF viewer
        catch
            fprintf('Cannot render PDF in MATLAB axes. Open manually: %s\n', file_path);
        end
        return
    case '.mat'
        % proceed below
    otherwise
        error('Unsupported file extension: %s', ext);
end

% Load .mat and try to detect known bundle formats
S = load(file_path);
fn = fieldnames(S);

% Case A: Overlay bundle with a single struct named 'data'
if ismember('data', fn) && isstruct(S.data)
    D = S.data;
    if isfield(D,'time') && any(isfield(D, {'pos_fused','pos_truth'}))
        local_plot_overlay(D, base);
        return
    end
end

% Case B: Quaternion/Euler bundles saved by Python helpers
has_t = ismember('t', fn);
if has_t
    % Quaternion truth vs estimate (wxyz)
    if (ismember('q_truth', fn) && (ismember('q_kf', fn) || ismember('q_est', fn)))
        qt = S.q_truth; qe = []; if ismember('q_kf', fn), qe = S.q_kf; else, qe = S.q_est; end
        local_plot_quat_components(S.t, qt, qe, base);
        return
    end
    % Euler truth vs estimate (ZYX degrees)
    if (ismember('e_truth_zyx_deg', fn) && (ismember('e_kf_zyx_deg', fn) || ismember('e_est_zyx_deg', fn)))
        et = S.e_truth_zyx_deg; ee = []; if ismember('e_kf_zyx_deg', fn), ee = S.e_kf_zyx_deg; else, ee = S.e_est_zyx_deg; end
        local_plot_euler_zyx_deg(S.t, et, ee, base);
        return
    end
    % Quaternion component residuals dq_wxyz
    if ismember('dq_wxyz', fn)
        local_plot_quat_residuals(S.t, S.dq_wxyz, base);
        return
    end
    % Attitude angle error over time att_err_deg
    if ismember('att_err_deg', fn)
        local_plot_angle_error(S.t, S.att_err_deg, base);
        return
    end
end

% Case C: Axis bundle saved with ax#_line#_x/ax#_line#_y keys
if any(startsWith(fn, 'ax'))
    local_plot_ax_bundle(S, base);
    return
end

% Case D: Generic numeric variables
vars = fn;
plotted = false;
for i = 1:numel(vars)
    vname = vars{i};
    try
        v = S.(vname);
    catch
        continue
    end
    if ~isnumeric(v) || isempty(v)
        continue
    end
    plotted = true;
    if isvector(v)
        y = v(:); x = (0:numel(y)-1)';
        figure('Name', sprintf('%s:%s', base, vname), 'Color','w');
        plot(x, y, '-'); grid on; xlabel('Index'); ylabel(vname, 'Interpreter','none');
        title(sprintf('%s — %s', base, vname), 'Interpreter','none');
    else
        n = size(v,2);
        % Draw up to 6 subplots per figure, continue in additional figures if needed
        cols = min(3, n);
        rows = min(ceil(n/cols), 6);
        idx = 1;
        x = (1:size(v,1))';
        while idx <= n
            take = min(6, n - idx + 1);
            figure('Name', sprintf('%s:%s [%d-%d]', base, vname, idx, idx+take-1), 'Color','w');
            for k = 1:take
                p = subplot(ceil(take/cols), cols, k); %#ok<LAXES>
                plot(p, x, v(:, idx+k-1), '-'); grid(p,'on');
                title(p, sprintf('%s(%d)', vname, idx+k-1), 'Interpreter','none');
            end
            idx = idx + take;
        end
    end
end

if ~plotted
    warning('No plottable numeric variables found in %s', file_path);
end

end

%% --- Helpers ---
function local_plot_quat_components(t, q_truth, q_est, namebase)
t = t(:);
q_truth = reshape(q_truth, [], 4);
q_est = reshape(q_est, [], 4);
n = min([numel(t), size(q_truth,1), size(q_est,1)]);
t = t(1:n); q_truth = q_truth(1:n,:); q_est = q_est(1:n,:);
figure('Name', namebase, 'Color','w');
labs = {'w','x','y','z'};
for i=1:4
    ax = subplot(4,1,i); hold(ax,'on'); grid(ax,'on');
    plot(ax, t, q_truth(:,i), '-', 'DisplayName','Truth');
    plot(ax, t, q_est(:,i),  '--', 'DisplayName','Estimate');
    ylabel(ax, ['q_' labs{i}]);
    if i==1, legend(ax,'show','Location','best'); end
    if i==4, xlabel(ax,'Time [s]'); end
end
sgtitle(strrep(namebase,'_','\_'));
end

function local_plot_euler_zyx_deg(t, e_truth, e_est, namebase)
t = t(:);
e_truth = reshape(e_truth, [], 3);
e_est = reshape(e_est, [], 3);
n = min([numel(t), size(e_truth,1), size(e_est,1)]);
t = t(1:n); e_truth = e_truth(1:n,:); e_est = e_est(1:n,:);
figure('Name', namebase, 'Color','w');
labs = {'Yaw [deg]','Pitch [deg]','Roll [deg]'};
for i=1:3
    ax = subplot(3,1,i); hold(ax,'on'); grid(ax,'on');
    plot(ax, t, e_truth(:,i), '-',  'DisplayName','Truth');
    plot(ax, t, e_est(:,i),   '--', 'DisplayName','Estimate');
    ylabel(ax, labs{i});
    if i==1, legend(ax,'show','Location','best'); end
    if i==3, xlabel(ax,'Time [s]'); end
end
sgtitle(strrep(namebase,'_','\_'));
end

function local_plot_quat_residuals(t, dq, namebase)
t = t(:); dq = reshape(dq, [], 4);
n = min(numel(t), size(dq,1)); t = t(1:n); dq = dq(1:n,:);
figure('Name', namebase, 'Color','w');
labs = {'w','x','y','z'};
for i=1:4
    ax = subplot(2,2,i); hold(ax,'on'); grid(ax,'on');
    plot(ax, t, dq(:,i), '-', 'DisplayName','Δq = est − truth');
    ylabel(ax, ['Δq_' labs{i}]);
    if i==1, legend(ax,'show','Location','best'); end
    if i>=3, xlabel(ax,'Time [s]'); end
end
sgtitle(strrep(namebase,'_','\_'));
end

function local_plot_angle_error(t, ang_deg, namebase)
t = t(:); ang_deg = ang_deg(:);
n = min(numel(t), numel(ang_deg)); t = t(1:n); ang_deg = ang_deg(1:n);
figure('Name', namebase, 'Color','w');
plot(t, ang_deg, '-'); grid on;
xlabel('Time [s]'); ylabel('Angle Error [deg]');
title(strrep(namebase,'_','\_'));
end

%% --- Existing helpers ---
function local_plot_overlay(D, namebase)
% Expect fields: time, pos_fused, pos_truth, vel_fused?, vel_truth?
t = D.time(:);
pf = local_getf(D,'pos_fused',[]); pt = local_getf(D,'pos_truth',[]);
vf = local_getf(D,'vel_fused',[]); vt = local_getf(D,'vel_truth',[]);
labels = {'North','East','Down'};  % default; rename for ECEF/Body if desired by caller name
% Heuristic: rename axis labels if name contains ECEF or Body
if contains(namebase, 'ECEF', 'IgnoreCase',true), labels = {'X','Y','Z'}; end
if contains(namebase, 'Body', 'IgnoreCase',true), labels = {'X','Y','Z'}; end

% Decimate if needed
n = numel(t); stride = max(1, ceil(n/200000));
ti = t(1:stride:end);
pf = local_take_stride(pf, stride);
pt = local_take_stride(pt, stride);
vf = local_take_stride(vf, stride);
vt = local_take_stride(vt, stride);

fig = figure('Name', namebase, 'Color','w');
for i=1:3
    ax = subplot(2,3,i); hold(ax,'on'); grid(ax,'on'); title(ax, labels{i});
    if ~isempty(pf), plot(ax, ti, pf(:,i), '-',  'DisplayName','Fused'); end
    if ~isempty(pt), plot(ax, ti, pt(:,i), '--', 'DisplayName','Truth'); end
    ylabel(ax, 'Position [m]');
end
for i=1:3
    ax = subplot(2,3,3+i); hold(ax,'on'); grid(ax,'on');
    if ~isempty(vf), plot(ax, ti, vf(:,i), '-',  'DisplayName','Fused'); end
    if ~isempty(vt), plot(ax, ti, vt(:,i), '--', 'DisplayName','Truth'); end
    xlabel(ax,'Time [s]'); ylabel(ax,'Velocity [m/s]');
end
% Harmonised symmetric limits
try
    lim_p = prctile(abs([pf(:); pt(:)]), 99.5);
    if isfinite(lim_p) && lim_p>0
        for i=1:3, ylim(subplot(2,3,i), [-lim_p lim_p]); end
    end
    v_all = [vf(:); vt(:)];
    v_all = v_all(isfinite(v_all));
    if ~isempty(v_all)
        lim_v = prctile(abs(v_all), 99.5);
        if isfinite(lim_v) && lim_v>0
            for i=1:3, ylim(subplot(2,3,3+i), [-lim_v lim_v]); end
        end
    end
catch, end
try, legend(subplot(2,3,1),'show','Location','northoutside','Orientation','horizontal'); catch, end
try, sgtitle(sprintf('Task 7.6 – Truth vs Fused (%s)', local_frame_from_name(namebase))); catch, end
end

function local_plot_ax_bundle(D, namebase)
% Rebuild figure(s) from keys ax#_line#_x / ax#_line#_y
fn = fieldnames(D);
ax_ids = [];
for i=1:numel(fn)
    tok = regexp(fn{i}, '^ax(\d+)_', 'tokens', 'once');
    if ~isempty(tok), ax_ids(end+1) = str2double(tok{1}); end %#ok<AGROW>
end
ax_ids = unique(ax_ids);
if isempty(ax_ids), return; end
figure('Name', namebase, 'Color','w');
n = numel(ax_ids);
for j=1:n
    ai = ax_ids(j);
    ax = subplot(n,1,j); hold(ax,'on'); grid(ax,'on');
    li = 1;
    while true
        xk = sprintf('ax%d_line%d_x', ai, li);
        yk = sprintf('ax%d_line%d_y', ai, li);
        if ~isfield(D, xk) || ~isfield(D, yk), break; end
        x = D.(xk); y = D.(yk);
        if isvector(x) && isvector(y)
            plot(ax, x(:), y(:));
        end
        li = li + 1;
    end
    tk = sprintf('ax%d_title', ai);
    xlk = sprintf('ax%d_xlabel', ai);
    ylk = sprintf('ax%d_ylabel', ai);
    if isfield(D, tk), title(ax, string(D.(tk))); end
    if isfield(D, xlk), xlabel(ax, string(D.(xlk))); end
    if isfield(D, ylk), ylabel(ax, string(D.(ylk))); end
end
end

function v = local_getf(S, k, def)
if isfield(S,k), v = S.(k); else, v = def; end
end

function A = local_take_stride(A, s)
if isempty(A), return; end
if isvector(A), A = A(1:s:end); else, A = A(1:s:end, :); end
end

function f = local_frame_from_name(name)
if contains(name,'ECEF','IgnoreCase',true), f='ECEF Frame'; return; end
if contains(name,'Body','IgnoreCase',true), f='Body Frame'; return; end
if contains(name,'NED','IgnoreCase',true), f='NED Frame'; return; end
f = 'Frame';
end
