function [txt, meta] = print_timeline_matlab(rid, imu_path, gnss_path, truth_path, out_dir)
%PRINT_TIMELINE_MATLAB Write timeline summary for datasets.
%   TXT = PRINT_TIMELINE_MATLAB(RID, IMU_PATH, GNSS_PATH, TRUTH_PATH, OUT_DIR)
%   writes a summary to console and file. Metadata is returned in META with
%   fields ``IMU``, ``GNSS`` and ``TRUTH`` mirroring the Python implementation.
%
% Usage:
%   print_timeline_matlab(''runid'', imu_path, gnss_path, truth_path, out_dir);
%
% Writes a timeline summary to console and to <rid>_timeline.txt.
% - Detects IMU time or synthesizes @400 Hz
% - GNSS uses Posix_Time or builds from UTC_* columns, else @1Hz fallback
% - TRUTH ignores '#' comments and uses column 2 as time

lines = strings_compat(0,1);
notes = strings_compat(0,1);
meta = struct();

% ---------- IMU ----------
imu = readmatrix_compat(imu_path,'FileType','text');
nI = size(imu,1);
col = [];
for c = 1:min(3,size(imu,2)) % try first 3 columns
    t = imu(:,c);
    if all(isfinite(t)) && all(diff(t) > 0)
        col = c; break;
    end
end
if isempty(col)
    tI = (0:nI-1)'/400; notes(end+1) = "IMU: constructed time @400Hz";
else
    tI = imu(:,col);     notes = [notes; sprintf("IMU: used time-like column %d (dt_med=%.6f)", col, median(diff(tI)))];
end
[line, meta.IMU] = format_line('IMU', tI, nI);
lines = [lines; line];

% ---------- GNSS ----------
opts = detectImportOptions(gnss_path,'Delimiter',',');
Tg = readtable(gnss_path, opts);
nG = height(Tg);
if any(strcmpi(Tg.Properties.VariableNames,'Posix_Time'))
    tg = ensure_time_vec(Tg.Posix_Time);  notes = [notes; "GNSS: used Posix_Time"];
else
    need = {'UTC_yyyy','UTC_MM','UTC_dd','UTC_HH','UTC_mm','UTC_ss'};
    if all(ismember(need, Tg.Properties.VariableNames))
        utc = datetime(Tg.UTC_yyyy, Tg.UTC_MM, Tg.UTC_dd, Tg.UTC_HH, Tg.UTC_mm, Tg.UTC_ss, 'TimeZone','UTC');
        t0 = utc(1); tg = seconds(utc - t0); notes = [notes; "GNSS: built from UTC_* columns"];
    else
        tg = (0:nG-1)';   notes = [notes; "GNSS: fallback uniform @1Hz"];
    end
end
[line, meta.GNSS] = format_line('GNSS', tg, nG);
lines = [lines; line];

% ---------- TRUTH ----------
if ~isempty(truth_path) && isfile(truth_path)
    to = detectImportOptions(truth_path, 'Delimiter',' ', 'CommentStyle','#', ...
        'ConsecutiveDelimitersRule','join', 'ReadVariableNames',false);
    Ts = readtable(truth_path, to);
    if size(Ts,2) >= 2
        ts = ensure_time_vec(Ts{:,2});
        nS = numel(ts);
        [line, meta.TRUTH] = format_line('TRUTH', ts, nS);
        lines = [lines; line];
    else
        lines = [lines; "TRUTH | present but unreadable (see Notes)."];
        meta.TRUTH = struct('n',0,'hz',NaN,'dt_med',NaN,'dt_min',NaN,'dt_max',NaN, ...
            'dur_s',NaN,'t0',NaN,'t1',NaN,'monotonic',false);
    end
else
    lines = [lines; "TRUTH | present but unreadable (see Notes)."];
    meta.TRUTH = struct('n',0,'hz',NaN,'dt_med',NaN,'dt_min',NaN,'dt_max',NaN, ...
        'dur_s',NaN,'t0',NaN,'t1',NaN,'monotonic',false);
end

% ---------- Print + Save ----------
hdr = "== Timeline summary: " + string(rid) + " ==";
txt = strjoin([hdr; ""; lines(:); ""; "Notes:"; "- " + notes(:)], newline);
disp(txt);

if ~exist(out_dir,'dir'), mkdir(out_dir); end
out_path = fullfile(out_dir, sprintf('%s_timeline.txt', rid));
fid = fopen(out_path,'w'); fprintf(fid,'%s\n',txt); fclose(fid);
fprintf('[DATA TIMELINE] Saved %s\n', out_path);
end

function [s, m] = format_line(tag, t, n)
t = t(:);
% Guard: ensure numeric time for diff
if isdatetime(t), t = seconds(t - t(1)); end
if isduration(t), t = seconds(t); end
if iscell(t) || isstring(t), t = str2double(t); end
dt = diff(t);
m = struct('n', n, 'hz', NaN, 'dt_med', NaN, 'dt_min', NaN, 'dt_max', NaN, ...
    'dur_s', NaN, 't0', NaN, 't1', NaN, 'monotonic', false);
if ~isempty(t)
    m.t0 = t(1); m.t1 = t(end); m.dur_s = t(end) - t(1);
end
if ~isempty(dt) && all(isfinite(dt))
    m.dt_med = median(dt); m.dt_min = min(dt); m.dt_max = max(dt);
    if m.dt_med > 0, m.hz = 1/m.dt_med; end
    m.monotonic = all(dt > 0);
end
s = sprintf('%-5s | n=%-7d hz=%0.6f  dt_med=%0.6f  min/max dt=(%0.6f,%0.6f)  dur=%0.3fs  t0=%0.6f  t1=%0.6f  monotonic=%s',...
    tag, m.n, m.hz, m.dt_med, m.dt_min, m.dt_max, m.dur_s, m.t0, m.t1, lower(string(m.monotonic)));
end

function t = ensure_time_vec(x)
% Ensure a numeric time vector from various input types
    if istable(x)
        x = x{:,1};
    end
    if isdatetime(x)
        t = seconds(x - x(1));
        return
    end
    if isduration(x)
        t = seconds(x);
        return
    end
    if iscell(x) || isstring(x)
        t = str2double(x);
        return
    end
    if ~isnumeric(x)
        try
            x = double(x);
        catch
            x = [];
        end
    end
    t = x(:);
end
