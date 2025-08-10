function print_timeline_matlab(rid, imu_path, gnss_path, truth_path, out_dir)
%PRINT_TIMELINE_MATLAB Write timeline summary for datasets.
%   PRINT_TIMELINE_MATLAB(RID, IMU_PATH, GNSS_PATH, TRUTH_PATH, OUT_DIR) writes a summary to console and file.
%
% Usage:
%   print_timeline_matlab(''runid'', imu_path, gnss_path, truth_path, out_dir);
%
% Writes a timeline summary to console and to <rid>_timeline.txt.
% - Detects IMU time or synthesizes @400 Hz
% - GNSS uses Posix_Time or builds from UTC_* columns, else @1Hz fallback
% - TRUTH ignores '#' comments and uses col 1 as time

lines = strings(0,1);
notes = strings(0,1);

% ---------- IMU ----------
imu = readmatrix(imu_path,'FileType','text');
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
lines = [lines; format_line('IMU', tI, nI)];

% ---------- GNSS ----------
opts = detectImportOptions(gnss_path,'Delimiter',',');
Tg = readtable(gnss_path, opts);
nG = height(Tg);
if any(strcmpi(Tg.Properties.VariableNames,'Posix_Time'))
    tg = ensure_time_vec(Tg.Posix_Time);
    % Normalise numeric POSIX time to start at zero like Python
    if ~isduration(tg) && ~isdatetime(tg)
        tg = tg - tg(1);
    end
    notes = [notes; "GNSS: used Posix_Time (t0=0)"];
else
    need = {'UTC_yyyy','UTC_MM','UTC_dd','UTC_HH','UTC_mm','UTC_ss'};
    if all(ismember(need, Tg.Properties.VariableNames))
        utc = datetime(Tg.UTC_yyyy, Tg.UTC_MM, Tg.UTC_dd, Tg.UTC_HH, Tg.UTC_mm, Tg.UTC_ss, 'TimeZone','UTC');
        t0 = utc(1); tg = seconds(utc - t0); notes = [notes; "GNSS: built from UTC_* columns (t0=0)"];
    else
        tg = (0:nG-1)';   notes = [notes; "GNSS: fallback uniform @1Hz (t0=0)"];
    end
end
lines = [lines; format_line('GNSS', tg, nG)];

% ---------- TRUTH ----------
if ~isempty(truth_path) && isfile(truth_path)
    % Use robust parser for STATE_*.txt via read_state_file to mirror Python
    try
        S = read_state_file(truth_path);
        % Heuristic: if 2nd column is time (common layout), use it; else fallback to 1st
        if size(S,2) >= 2 && all(diff(S(:,2)) >= 0)
            ts = S(:,2);
        else
            ts = S(:,1);
        end
        ts = ts - ts(1); % normalise to t0=0
        nS = numel(ts);
        lines = [lines; format_line('TRUTH', ts, nS)];
        notes = [notes; "TRUTH: parsed via read_state_file (t0=0)"];
    catch
        to = detectImportOptions(truth_path, 'Delimiter',' ', 'CommentStyle','#', 'ConsecutiveDelimitersRule','join');
        Ts = readtable(truth_path, to);
        ts = ensure_time_vec(Ts{:,1});
        if ~isempty(ts)
            ts = ts - ts(1);
        end
        nS = numel(ts);
        lines = [lines; format_line('TRUTH', ts, nS)];
        notes = [notes; "TRUTH: fallback table read (t0=0)"];
    end
else
    lines = [lines; "TRUTH | present but unreadable (see Notes)."];
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

function s = format_line(tag, t, n)
t = t(:);
% Guard: ensure numeric time for diff
if isdatetime(t), t = seconds(t - t(1)); end
if isduration(t), t = seconds(t); end
if iscell(t) || isstring(t), t = str2double(t); end
dt = diff(t);
if isempty(dt) || ~all(isfinite(dt)), hz = NaN; med = NaN; mn = NaN; mx = NaN; mono = false;
else, hz = 1/median(dt); med = median(dt); mn = min(dt); mx = max(dt); mono = all(dt > 0);
end
dur = t(end) - t(1);
s = sprintf('%-5s | n=%-7d hz=%0.6f  dt_med=%0.6f  min/max dt=(%0.6f,%0.6f)  dur=%0.3fs  t0=%0.6f  t1=%0.6f  monotonic=%s',...
    tag, n, hz, med, mn, mx, dur, t(1), t(end), lower(string(mono)));
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
