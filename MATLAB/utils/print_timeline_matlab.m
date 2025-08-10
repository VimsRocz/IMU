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
stats = struct('who',{},'n',{},'hz',{},'dt_med',{},'duration',{},'t0',{},'t1',{});

% ---------- IMU ----------
tI = [];
imu = readmatrix(imu_path,'FileType','text');
nI = size(imu,1);
[tI, st, note] = handle_time('IMU', imu(:,1:min(3,size(imu,2))), nI);
lines = [lines; st.line];
stats(end+1) = st.stats; %#ok<AGROW>
notes(end+1) = note;

% ---------- GNSS ----------
opts = detectImportOptions(gnss_path,'Delimiter',',');
Tg = readtable(gnss_path, opts);
nG = height(Tg);
[tg, st, note] = handle_gnss_time(Tg, nG);
lines = [lines; st.line];
stats(end+1) = st.stats; %#ok<AGROW>
notes(end+1) = note;

% ---------- TRUTH ----------
[ts, st, note] = handle_truth_time(truth_path);
lines = [lines; st.line];
stats(end+1) = st.stats; %#ok<AGROW>
notes(end+1) = note;

% ---------- Print + Save ----------
hdr = "== Timeline summary: " + string(rid) + " ==";
txt = strjoin([hdr; ""; lines(:); ""; "Notes:"; "- " + notes(:)], newline);
disp(txt);

tbl = struct2table(stats);
if ~isempty(tbl)
    csv_path = fullfile(out_dir, sprintf('%s_timeline.csv', rid));
    writetable(tbl, csv_path);
    fprintf('[DATA TIMELINE] Saved %s\n', csv_path);
    tr = tbl(strcmp(tbl.who,'TRUTH'),:);
    if ~isempty(tr) && abs(tr.dt_med - 0.1) > 0.01
        warning('Truth dt_med=%0.3f s differs from 0.100 s', tr.dt_med);
    end
end

if ~exist(out_dir,'dir'), mkdir(out_dir); end
out_path = fullfile(out_dir, sprintf('%s_timeline.txt', rid));
fid = fopen(out_path,'w'); fprintf(fid,'%s\n',txt); fclose(fid);
fprintf('[DATA TIMELINE] Saved %s\n', out_path);
end

function [t, out, note] = handle_time(tag, cols, n)
col = [];
for c = 1:size(cols,2)
    t_candidate = cols(:,c);
    if all(isfinite(t_candidate)) && all(diff(t_candidate) > 0)
        col = c; t = t_candidate; break;
    end
end
if isempty(col)
    t = (0:n-1)'/400; note = sprintf('%s: constructed time @400Hz', tag);
else
    note = sprintf('%s: used time-like column %d (dt_med=%0.6f)', tag, col, median(diff(t)));
end
out = struct();
[out.line, out.stats] = format_line(tag, t, n);
end
 
function [t, out, note] = handle_gnss_time(Tg, nG)
if any(strcmpi(Tg.Properties.VariableNames,'Posix_Time'))
    t = ensure_time_vec(Tg.Posix_Time);
    note = "GNSS: used Posix_Time";
else
    need = {'UTC_yyyy','UTC_MM','UTC_dd','UTC_HH','UTC_mm','UTC_ss'};
    if all(ismember(need, Tg.Properties.VariableNames))
        utc = datetime(Tg.UTC_yyyy, Tg.UTC_MM, Tg.UTC_dd, Tg.UTC_HH, Tg.UTC_mm, Tg.UTC_ss, 'TimeZone','UTC');
        t0 = utc(1); t = seconds(utc - t0);
        note = "GNSS: built from UTC_* columns";
    else
        t = (0:nG-1)';
        note = "GNSS: fallback uniform @1Hz";
    end
end
out = struct();
[out.line, out.stats] = format_line('GNSS', t, nG);
end

function [t, out, note] = handle_truth_time(truth_path)
if ~isempty(truth_path) && isfile(truth_path)
    to = detectImportOptions(truth_path, 'Delimiter',' ', 'CommentStyle','#', 'ConsecutiveDelimitersRule','join');
    Ts = readtable(truth_path, to);
    t = ensure_time_vec(Ts{:,1});
    nS = numel(t);
    out = struct();
    [out.line, out.stats] = format_line('TRUTH', t, nS);
    note = "TRUTH: loaded";
else
    t = [];
    out.line = "TRUTH | present but unreadable (see Notes).";
    out.stats = struct('who','TRUTH','n',0,'hz',NaN,'dt_med',NaN,'duration',NaN,'t0',NaN,'t1',NaN);
    note = "TRUTH: missing";
end
end

function [s, st] = format_line(tag, t, n)
t = t(:);
if isdatetime(t), t = seconds(t - t(1)); end
if isduration(t), t = seconds(t); end
if iscell(t) || isstring(t), t = str2double(t); end
dt = diff(t);
if isempty(dt) || ~all(isfinite(dt))
    hz = NaN; med = NaN; mn = NaN; mx = NaN; mono = false;
else
    med = median(dt); hz = 1/med; mn = min(dt); mx = max(dt); mono = all(dt > 0);
end
dur = t(end) - t(1);
s = sprintf('%-5s | n=%-7d hz=%0.6f  dt_med=%0.6f  min/max dt=(%0.6f,%0.6f)  dur=%0.3fs  t0=%0.6f  t1=%0.6f  monotonic=%s',...
    tag, n, hz, med, mn, mx, dur, t(1), t(end), lower(string(mono)));
st = struct('who',tag,'n',n,'hz',hz,'dt_med',med,'duration',dur,'t0',t(1),'t1',t(end));
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
