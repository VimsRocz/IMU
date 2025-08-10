function compare_matlab_vs_python()
% COMAPRE_MATLAB_VS_PYTHON
% Compares MATLAB vs Python outputs for a dataset/method across Tasks 1–7.
% - Reads MATLAB .mat results from MATLAB/results/
% - Reads Python runmeta.json (+ time .mat if present) from results/
% - Builds summary tables with PASS/FAIL based on tolerances
%
% Usage:
%   - Adjust cfg.* paths below if your layout differs.
%   - Run:  compare_matlab_vs_python
%
% Notes:
%   * Handles missing fields gracefully (prints N/A).
%   * Flags common mismatch causes (timebase, ZUPT usage, ECEF velocity).
%   * Writes CSVs in MATLAB/results/ for easy sharing.

%% -------------------- CONFIG --------------------
cfg.dataset  = 'X002';       % 'X001' or 'X002' etc
cfg.method   = 'TRIAD';      % 'TRIAD' | 'Davenport' | 'SVD'
% Root folder (repo root). If this script is inside the repo, pwd usually works:
cfg.root     = pwd;

cfg.dir_py   = fullfile(cfg.root, 'results');
cfg.dir_ml   = fullfile(cfg.root, 'MATLAB', 'results');

% Filenames (by convention used in your logs)
tag = sprintf('IMU_%s_GNSS_%s_%s', cfg.dataset, cfg.dataset, cfg.method);

fn_py_runmeta = fullfile(cfg.dir_py, [tag '_runmeta.json']);
fn_py_t5_time = fullfile(cfg.dir_py, [tag '_task5_time.mat']);

fn_ml_t1 = fullfile(cfg.dir_ml, sprintf('Task1_init_%s.mat', tag));
fn_ml_t2 = fullfile(cfg.dir_ml, sprintf('Task2_body_%s.mat', strrep(tag,'_TRIAD','_IMU_X002_GNSS_X002_TRIAD'))); % tolerate legacy name
if ~isfile(fn_ml_t2)
    fn_ml_t2 = fullfile(cfg.dir_ml, sprintf('Task2_body_%s.mat', tag));
end
fn_ml_t4 = fullfile(cfg.dir_ml, [tag '_task4_results.mat']);
fn_ml_t5 = fullfile(cfg.dir_ml, [tag '_task5_results.mat']);
fn_ml_t5_time = fullfile(cfg.dir_ml, [tag '_task5_time.mat']);
fn_ml_t6 = fullfile(cfg.dir_ml, [tag '_task6_results.mat']);
fn_ml_t6_dt = fullfile(cfg.dir_ml, 'Task6_time_shift.mat');   % optional (if you saved it)
fn_ml_t7 = fullfile(cfg.dir_ml, [tag '_task7_results.mat']);

% Tolerances for "pass"
tol = struct();
tol.latlon_deg    = 1e-3;          % deg
tol.g_mag         = 1e-4;          % m/s^2
tol.bias_acc_abs  = 5e-3;          % m/s^2   (static bias should match within a few mg)
tol.bias_gyro_abs = 5e-6;          % rad/s
tol.rmse_pos_m    = 1.0;           % m
tol.rmse_vel_mps  = 5.0;           % m/s
tol.zupt_ratio    = 0.80;          % MATLAB zuptcnt should be >= 80% of Python (since both use same static mask)
tol.dt_sec        = 0.25;          % s

%% -------------------- LOAD PYTHON ARTIFACTS --------------------
P = struct(); P.ok = false;
if isfile(fn_py_runmeta)
    try
        P.meta = jsondecode(fileread(fn_py_runmeta));
        P.ok = true;
    catch ME
        warn('Failed reading Python runmeta: %s', ME.message);
    end
else
    warn('Python runmeta not found: %s', fn_py_runmeta);
end

% Optional Python Task-5 time (for timeline comparison)
if isfile(fn_py_t5_time)
    try
        S = load(fn_py_t5_time);
        P.t5_time = S;
    catch ME
        warn('Failed loading Python task5_time: %s', ME.message);
    end
end

%% -------------------- LOAD MATLAB ARTIFACTS --------------------
M = struct();

M.t1 = load_opt(fn_ml_t1);
M.t2 = load_opt(fn_ml_t2);
M.t4 = load_opt(fn_ml_t4);
M.t5 = load_opt(fn_ml_t5);
M.t5_time = load_opt(fn_ml_t5_time);
M.t6 = load_opt(fn_ml_t6);
M.t6_dt = load_opt(fn_ml_t6_dt);
M.t7 = load_opt(fn_ml_t7);

%% -------------------- GATHER METRICS --------------------
% Task-1: lat/lon, gravity magnitude (NED +Z down)
lat_ml = getoptT(M.t1,'latitude_deg', NaN);
lon_ml = getoptT(M.t1,'longitude_deg',NaN);
g_ml   = vecmag(getoptT(M.t1,'g_ned',[0 0 NaN]));

lat_py = getjson(P,'latitude_deg', NaN);
lon_py = getjson(P,'longitude_deg',NaN);
g_py   = getjson(P,'gravity_mps2', NaN);

% Task-2: body-frame vectors & biases
g_body_ml    = getoptT(M.t2,'g_body', [NaN NaN NaN]); %#ok<NASGU>
omega_body_ml= getoptT(M.t2,'omega_ie_body',[NaN NaN NaN]); %#ok<NASGU>
acc_bias_ml  = getoptT(M.t2,'accel_bias',[NaN NaN NaN]);
gyro_bias_ml = getoptT(M.t2,'gyro_bias',[NaN NaN NaN]);
acc_scale_ml = getoptT(M.t2,'accel_scale',NaN); %#ok<NASGU>

acc_bias_py  = getjson(P,'accel_bias_task2', [NaN NaN NaN]);
gyro_bias_py = getjson(P,'gyro_bias_task2',  [NaN NaN NaN]);

% Task-5: KF metrics (RMSE etc.) & ZUPT count
% MATLAB (often stored in t5.results or printed; try common names)
rmse_pos_ml = pickfirst(M.t5, {'rmse_pos','RMSE_pos','rmse_pos_m'}, NaN);
rmse_vel_ml = pickfirst(M.t5, {'rmse_vel','RMSE_vel','rmse_vel_mps'}, NaN);
final_pos_ml= pickfirst(M.t5, {'final_pos','EndError_pos','final_pos_m'}, NaN);
final_vel_ml= pickfirst(M.t5, {'final_vel','final_vel_mps'}, NaN);
zupt_ml     = pickfirst(M.t5, {'ZUPTcnt','zupt_count'}, NaN);

% Python summary (from runmeta.json dataset block if present)
rmse_pos_py = getjson(P,'rmse_pos_m', NaN);
rmse_vel_py = getjson(P,'rmse_vel_mps', NaN);
final_pos_py= getjson(P,'final_pos_m', NaN);
final_vel_py= getjson(P,'final_vel_mps', NaN);
zupt_py     = getjson(P,'ZUPT_count',  NaN);

% Task-6: time alignment & sanity correlations
dt_ml = pickfirst(M.t6_dt, {'dt_hat','dt_shift','time_offset_s'}, NaN);
dt_py = getjson(P,'time_offset_s', NaN);

corrN_ml = pickfirst(M.t6, {'corrN','corr_north'}, NaN);
corrE_ml = pickfirst(M.t6, {'corrE','corr_east'}, NaN);
corrN_py = getjson(P,'corrN', NaN);
corrE_py = getjson(P,'corrE', NaN);

% Task-7: ECEF residuals
t7_pos_mean_ml = pickfirst(M.t7, {'pos_error_mean','pos_err_mean'}, [NaN NaN NaN]);
t7_pos_std_ml  = pickfirst(M.t7, {'pos_error_std','pos_err_std'},   [NaN NaN NaN]);
t7_vel_mean_ml = pickfirst(M.t7, {'vel_error_mean','vel_err_mean'}, [NaN NaN NaN]);
t7_vel_std_ml  = pickfirst(M.t7, {'vel_error_std','vel_err_std'},   [NaN NaN NaN]);

t7_pos_mean_py = getjson(P,'t7_pos_error_mean_ecef', [NaN NaN NaN]);
t7_pos_std_py  = getjson(P,'t7_pos_error_std_ecef',  [NaN NaN NaN]);
t7_vel_mean_py = getjson(P,'t7_vel_error_mean_ecef', [NaN NaN NaN]);
t7_vel_std_py  = getjson(P,'t7_vel_error_std_ecef',  [NaN NaN NaN]);

%% -------------------- BUILD TABLES --------------------
rows = {};

% Task-1
rows = add_row(rows,'Task1: Latitude [deg]', lat_ml, lat_py, tol.latlon_deg);
rows = add_row(rows,'Task1: Longitude [deg]',lon_ml, lon_py, tol.latlon_deg);
rows = add_row(rows,'Task1: Gravity |g| [m/s^2]', g_ml, g_py, tol.g_mag);

% Task-2
rows = add_row(rows,'Task2: Accel bias X [m/s^2]', acc_bias_ml(1), acc_bias_py(1), tol.bias_acc_abs);
rows = add_row(rows,'Task2: Accel bias Y [m/s^2]', acc_bias_ml(2), acc_bias_py(2), tol.bias_acc_abs);
rows = add_row(rows,'Task2: Accel bias Z [m/s^2]', acc_bias_ml(3), acc_bias_py(3), tol.bias_acc_abs);
rows = add_row(rows,'Task2: Gyro bias X [rad/s]',  gyro_bias_ml(1), gyro_bias_py(1), tol.bias_gyro_abs);
rows = add_row(rows,'Task2: Gyro bias Y [rad/s]',  gyro_bias_ml(2), gyro_bias_py(2), tol.bias_gyro_abs);
rows = add_row(rows,'Task2: Gyro bias Z [rad/s]',  gyro_bias_ml(3), gyro_bias_py(3), tol.bias_gyro_abs);

% Task-5
rows = add_row(rows,'Task5: RMSE pos [m]',     rmse_pos_ml, rmse_pos_py, tol.rmse_pos_m);
rows = add_row(rows,'Task5: Final pos [m]',    final_pos_ml,final_pos_py,tol.rmse_pos_m);
rows = add_row(rows,'Task5: RMSE vel [m/s]',   rmse_vel_ml, rmse_vel_py, tol.rmse_vel_mps);
rows = add_row(rows,'Task5: Final vel [m/s]',  final_vel_ml,final_vel_py,tol.rmse_vel_mps);

% Task-6
rows = add_row(rows,'Task6: Δt align [s]',     dt_ml, dt_py, tol.dt_sec);
rows = add_row(rows,'Task6: corr(N) [–]',      corrN_ml, corrN_py, 0.05);
rows = add_row(rows,'Task6: corr(E) [–]',      corrE_ml, corrE_py, 0.05);

% Task-7 (ECEF residuals; compare norms)
rows = add_row(rows,'Task7: |μ_pos_err| [m]',  nannorm(t7_pos_mean_ml), nannorm(t7_pos_mean_py), 1.0);
rows = add_row(rows,'Task7: |σ_pos_err| [m]',  nannorm(t7_pos_std_ml),  nannorm(t7_pos_std_py),  1.0);
rows = add_row(rows,'Task7: |μ_vel_err| [m/s]',nannorm(t7_vel_mean_ml), nannorm(t7_vel_mean_py), 5.0);
rows = add_row(rows,'Task7: |σ_vel_err| [m/s]',nannorm(t7_vel_std_ml),  nannorm(t7_vel_std_py),  5.0);

T = cell2table(rows, 'VariableNames', {'Metric','MATLAB','Python','AbsDiff','RelDiffPct','Pass'});
disp('===== MATLAB vs Python: Summary =====');
disp(T);

% Write CSV for sharing
out_csv = fullfile(cfg.dir_ml, sprintf('%s_matlab_vs_python_compare.csv', tag));
try
    if ~exist(cfg.dir_ml,'dir'), mkdir(cfg.dir_ml); end
    writetable(T, out_csv);
    fprintf('Saved compare table: %s\n', out_csv);
catch ME
    warn('Failed to save CSV: %s', ME.message);
end

%% -------------------- DIAGNOSTIC HINTS --------------------
fprintf('\n===== Diagnostics =====\n');
% Timebase check (Python vs MATLAB IMU t vectors)
if isfield(P,'t5_time') && isfield(M,'t5_time') && isfield(M.t5_time,'t')
    tpy = P.t5_time.t(:); tml = M.t5_time.t(:);
    fprintf('IMU time: Python t0=%.6f t1=%.6f  |  MATLAB t0=%.6f t1=%.6f\n', ...
        tpy(1), tpy(end), tml(1), tml(end));
    if abs(tpy(1) - 0) < 1e-9 && abs(tml(1) - 0) < 1e-9
        fprintf('OK: Both IMU timelines are normalized to t0=0.\n');
    else
        warn('IMU time bases differ (t0 mismatch). Normalize to t0=0 for both.');
    end
else
    fprintf('IMU time vectors not available in both runs (skip IMU timeline check).\n');
end

% ZUPT check
if ~isnan(zupt_ml) && ~isnan(zupt_py) && zupt_py > 0
    ratio = zupt_ml / zupt_py;
    fprintf('ZUPT count: MATLAB=%d  Python=%d  (%.1f%%)\n', zupt_ml, zupt_py, 100*ratio);
    if ratio < tol.zupt_ratio
        warn('ZUPT usage is too low in MATLAB vs Python. Wire Task-2 static mask into KF ZUPT.');
    end
else
    fprintf('ZUPT counts not available in both runs (skip ZUPT check).\n');
end

% Task-7 ECEF velocity sanity (final vel ≈ zero in MATLAB indicates conversion bug)
if isfield(M.t7,'final_truth_vel_ecef') && isfield(M.t7,'final_fused_vel_ecef')
    vT = M.t7.final_truth_vel_ecef(:);
    vF = M.t7.final_fused_vel_ecef(:);
    if norm(vT) > 50 && norm(vF) < 1e-3
        warn('Task-7: Fused ECEF velocity ~0 while truth is large. Fix NED->ECEF vel: v_e = C_en*v_n + \omega_{ie}\times r.');
    end
end

% Δt consistency across tasks
if ~isnan(dt_ml) && ~isnan(dt_py)
    if abs(dt_ml - dt_py) > tol.dt_sec
        warn('Time shift mismatch: Task-6 Δt differs between MATLAB and Python. Estimate once and reuse.');
    else
        fprintf('OK: Time shift matches (Δt MATLAB=%.3fs, Python=%.3fs).\n', dt_ml, dt_py);
    end
else
    fprintf('Time shift not available in both runs (skip Δt check).\n');
end

fprintf('Diagnostics complete.\n');

end

%% -------------------- HELPERS --------------------
function S = load_opt(fname)
    if isfile(fname)
        try, S = load(fname);
        catch, S = struct(); end
    else
        S = struct();
    end
end

function v = getoptT(S, name, default)
    if isstruct(S) && isfield(S, name)
        v = S.(name);
    else
        v = default;
    end
end

function v = getjson(P, name, default)
    % Flattens common runmeta shapes
    v = default;
    if ~isstruct(P) || ~isfield(P,'meta'), return; end
    M = P.meta;
    if isfield(M, name), v = M.(name); return; end
    % Often nested under .summary or .dataset or .metrics
    keys = {'summary','dataset','metrics','task7','task6','task5'};
    for k = 1:numel(keys)
        if isfield(M, keys{k})
            S = M.(keys{k});
            if isfield(S, name), v = S.(name); return; end
        end
    end
    % dataset array case (find matching tag)
    if isfield(M,'datasets') && iscell(M.datasets)
        try
            for i=1:numel(M.datasets)
                D = M.datasets{i};
                if isfield(D,name), v = D.(name); return; end
            end
        catch
        end
    end
end

function m = vecmag(v)
    v = v(:);
    if any(isnan(v)), m = NaN; else, m = norm(v); end
end

function n = nannorm(v)
    v = v(:);
    if any(isnan(v)), n = NaN; else, n = norm(v); end
end

function rows = add_row(rows,label,ml,py,abs_tol)
    [ad,rd,pass] = diffs(ml,py,abs_tol);
    rows(end+1,1:6) = {label, num2nan(ml), num2nan(py), ad, rd*100, passstr(pass)}; %#ok<AGROW>
end

function [ad,rd,pass] = diffs(ml,py,abs_tol)
    if any(isnan([ml py]))
        ad = NaN; rd = NaN; pass = false; return;
    end
    ad = abs(ml - py);
    denom = max(abs(py), 1e-12);
    rd = ad / denom;
    pass = (ad <= abs_tol) || rd <= 0.05; % also pass if within 5% rel error
end

function s = passstr(tf)
    if tf, s = 'PASS'; else, s = 'FAIL'; end
end

function x = num2nan(x)
    if isempty(x), x = NaN; end
    if islogical(x), x = double(x); end
end

function warn(msg, varargin)
    fprintf(2, ['[WARN] ' msg '\n'], varargin{:});
end

function v = pickfirst(S, names, default)
    % Return first available field from names in S or S.results
    v = default;
    if ~isstruct(S), return; end
    for i=1:numel(names)
        if isfield(S, names{i})
            v = S.(names{i}); return;
        end
    end
    if isfield(S,'results') && isstruct(S.results)
        for i=1:numel(names)
            if isfield(S.results, names{i})
                v = S.results.(names{i}); return;
            end
        end
    end
end

