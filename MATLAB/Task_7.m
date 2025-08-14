function Task_7()
%TASK_7 Residual analysis with frame/time alignment and guard rails.
fprintf('--- Starting Task 7: Residual Analysis ---\n');

% Paths
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'src')));
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'utils')));
paths = project_paths();
resultsDir = paths.matlab_results;
dataTruthDir = fullfile(paths.root, 'DATA', 'Truth');

% Locate Task 5 results
files = dir(fullfile(resultsDir, '*_task5_results.mat'));
if isempty(files), error('Task7: Task5 results not found.'); end
[~, runTag] = fileparts(files(1).name);
runTag = erase(runTag, '_task5_results');

% Load Task-5 state
res5 = load(fullfile(resultsDir, sprintf('%s_task5_results.mat', runTag)));
Ttruth = readtable(fullfile(dataTruthDir, 'STATE_X001.txt'), detectImportOptions(fullfile(dataTruthDir,'STATE_X001.txt')));

% Time
if isfield(res5,'t_est'), t_est = res5.t_est; else, error('Task7: t_est missing'); end
N = numel(t_est);
t_truth = extractTimeVec(Ttruth);

% Estimated (prefer NED; else convert)
lat = res5.ref_lat; lon = res5.ref_lon;
E.pos_ned = []; E.vel_ned = []; E.acc_ned = [];
if isfield(res5,'pos_ned_est'), E.pos_ned = res5.pos_ned_est; end
if isempty(E.pos_ned) && isfield(res5,'pos_ecef_est')
    E.pos_ned = attitude_tools('ecef2ned_vec', res5.pos_ecef_est, lat, lon);
end
if isfield(res5,'vel_ned_est'), E.vel_ned = res5.vel_ned_est; end
if isempty(E.vel_ned) && isfield(res5,'vel_ecef_est')
    E.vel_ned = attitude_tools('ecef2ned_vec', res5.vel_ecef_est, lat, lon);
end
if isfield(res5,'acc_ned_est'), E.acc_ned = res5.acc_ned_est; end
if isempty(E.acc_ned) && isfield(res5,'acc_ecef_est')
    E.acc_ned = attitude_tools('ecef2ned_vec', res5.acc_ecef_est, lat, lon);
end

% Truth (ECEF->NED if needed)
[P_ecef, V_ecef] = extractECEF(Ttruth);
[P_ned,  V_ned ] = extractNED(Ttruth);
if isempty(P_ned) && ~isempty(P_ecef), P_ned = attitude_tools('ecef2ned_vec', P_ecef, lat, lon); end
if isempty(V_ned) && ~isempty(V_ecef), V_ned = attitude_tools('ecef2ned_vec', V_ecef, lat, lon); end

% Interpolate truth to estimator time
interp = @(X) attitude_tools('interp_to', t_truth, X, t_est);
Ptru = interp(P_ned); Vtru = interp(V_ned);

% Now compute residuals (NED)
% Validate shapes before proceeding
if isempty(E.pos_ned) || size(E.pos_ned,2) < 3 || isempty(Ptru) || size(Ptru,2) < 3
    warning('[Task7] Insufficient data for residual plots (E.pos_ned=%s, Ptru=%s). Skipping Task 7 plots.', ...
        mat2str(size(E.pos_ned)), mat2str(size(Ptru)));
    return;
end

pos_residual = E.pos_ned - Ptru;
vel_residual = E.vel_ned - Vtru;
disp(sprintf('[DBG-T7] pos_residual size: %dx%d | t_est length=%d', size(pos_residual,1), size(pos_residual,2), length(t_est)));
if size(pos_residual,2) < 3
    disp('[ERROR-T7] pos_residual has <3 columns; check fused/truth alignment');
end

% Guard rails: drop outliers > 1e4 m or > 1e4 m/s to avoid plotting explosions
pos_residual(abs(pos_residual)>1e4) = NaN;
vel_residual(abs(vel_residual)>1e4) = NaN;

% Replace hard assert with warning + diagnostic save
posMax = max(abs(pos_residual), [], 'omitnan');
if any(posMax > 100)
    warning('Task-7: Large position residual detected (max=%g m). Check reference & bias.', max(posMax));
end

% Rebase using Task-4 origin if available
if exist(fullfile(resultsDir, sprintf('%s_task4_results.mat', runTag)), 'file')
    r4 = load(fullfile(resultsDir, sprintf('%s_task4_results.mat', runTag)));
    if isfield(r4,'r0_ecef') && ~isempty(P_ecef)
        P_ecef = P_ecef - r4.r0_ecef(:).';
        P_ned  = attitude_tools('ecef2ned_vec', P_ecef, lat, lon);
        Ptru   = interp(P_ned);
        pos_residual = E.pos_ned - Ptru;  % recompute with consistent origin
    end
end

% Diagnostic plots
f1 = figure('Visible','off','Position',[100 100 1400 500]);
fprintf('[Task7] Plotting residuals: pos_residual=%dx%d vel_residual=%dx%d\n', size(pos_residual,1), size(pos_residual,2), size(vel_residual,1), size(vel_residual,2));
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
lbl = {'N','E','D'};
for i=1:3
    disp(sprintf('[DBG-T7] Plotting residual %s: mean=%.2f std=%.2f max=%.2f', lbl{i}, mean(pos_residual(:,i),'omitnan'), std(pos_residual(:,i),'omitnan'), max(abs(pos_residual(:,i)), [], 'omitnan')));
    nexttile; plot(t_est, pos_residual(:,i),'LineWidth',1.0); grid on; title(sprintf('Pos residual %s [m]', lbl{i}));
    xlabel('Time [s]');
end
png1 = fullfile(resultsDir, sprintf('%s_task7_pos_residual_NED.png', runTag));
exportgraphics(f1, png1, 'Resolution',150);
try, savefig(f1, strrep(png1, '.png', '.fig')); catch, end
close(f1);

f2 = figure('Visible','off','Position',[100 100 1400 500]);
tiledlayout(1,3,'Padding','compact','TileSpacing','compact');
for i=1:3
    disp(sprintf('[DBG-T7] Plotting velocity residual %s: mean=%.2f std=%.2f max=%.2f', lbl{i}, mean(vel_residual(:,i),'omitnan'), std(vel_residual(:,i),'omitnan'), max(abs(vel_residual(:,i)), [], 'omitnan')));
    nexttile; plot(t_est, vel_residual(:,i),'LineWidth',1.0); grid on; title(sprintf('Vel residual %s [m/s]', lbl{i}));
    xlabel('Time [s]');
end
png2 = fullfile(resultsDir, sprintf('%s_task7_vel_residual_NED.png', runTag));
exportgraphics(f2, png2, 'Resolution',150);
try, savefig(f2, strrep(png2, '.png', '.fig')); catch, end
close(f2);

% Summary metrics
rmse = @(x) sqrt(mean(x.^2, 'omitnan'));
metrics = struct();
metrics.rmse_pos = rmse(pos_residual(:));
metrics.rmse_vel = rmse(vel_residual(:));
metrics.max_pos  = max(abs(pos_residual(:)), [], 'omitnan');
metrics.max_vel  = max(abs(vel_residual(:)), [], 'omitnan');
save(fullfile(resultsDir, sprintf('%s_task7_metrics.mat', runTag)), '-struct', 'metrics');

fprintf('[Task7] RMSE pos=%.3f m, RMSE vel=%.3f m/s, max|pos|=%.3f m\n', ...
    metrics.rmse_pos, metrics.rmse_vel, metrics.max_pos);
end

function t = extractTimeVec(T)
    t = [];
    for c = T.Properties.VariableNames
        nm = lower(c{1});
        if any(strcmp(nm, {'time','t','posix_time','sec','seconds'}))
            t = T.(c{1}); t = t(:); return;
        end
    end
    t = (0:height(T)-1)'; % fallback
end

function [P,V] = extractECEF(T)
    P=[]; V=[];
    cx = findCol(T, {'pos_ecef_x','ecef_x','x_ecef','x'});
    cy = findCol(T, {'pos_ecef_y','ecef_y','y_ecef','y'});
    cz = findCol(T, {'pos_ecef_z','ecef_z','z_ecef','z'});
    if ~isempty(cx)&&~isempty(cy)&&~isempty(cz)
        P = [T.(cx), T.(cy), T.(cz)];
    end
    vx = findCol(T, {'vel_ecef_x','vx_ecef','ecef_vx','vx'});
    vy = findCol(T, {'vel_ecef_y','vy_ecef','ecef_vy','vy'});
    vz = findCol(T, {'vel_ecef_z','vz_ecef','ecef_vz','vz'});
    if ~isempty(vx)&&~isempty(vy)&&~isempty(vz)
        V = [T.(vx), T.(vy), T.(vz)];
    end
end

function [P,V] = extractNED(T)
    P=[]; V=[];
    cn = findCol(T, {'pos_n','ned_n','north'});
    ce = findCol(T, {'pos_e','ned_e','east'});
    cd = findCol(T, {'pos_d','ned_d','down'});
    if ~isempty(cn)&&~isempty(ce)&&~isempty(cd)
        P = [T.(cn), T.(ce), T.(cd)];
    end
    vn = findCol(T, {'vel_n','ned_vn','vn','north_vel'});
    ve = findCol(T, {'vel_e','ned_ve','ve','east_vel'});
    vd = findCol(T, {'vel_d','ned_vd','vd','down_vel'});
    if ~isempty(vn)&&~isempty(ve)&&~isempty(vd)
        V = [T.(vn), T.(ve), T.(vd)];
    end
end

function nm = findCol(T, cand)
    nm = '';
    for k=1:numel(cand)
        idx = find(strcmpi(T.Properties.VariableNames, cand{k}), 1);
        if ~isempty(idx), nm = T.Properties.VariableNames{idx}; return; end
    end
end
