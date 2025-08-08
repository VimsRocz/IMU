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
    tI = imu(:,col);     notes(end+1) = sprintf("IMU: used time-like column %d (dt_med=%.6f)", col, median(diff(tI)));
end
lines(end+1) = format_line('IMU', tI, nI);

% ---------- GNSS ----------
opts = detectImportOptions(gnss_path,'Delimiter',',');
Tg = readtable(gnss_path, opts);
nG = height(Tg);
if any(strcmpi(Tg.Properties.VariableNames,'Posix_Time'))
    tg = Tg.Posix_Time;  notes(end+1) = "GNSS: used Posix_Time";
else
    need = {'UTC_yyyy','UTC_MM','UTC_dd','UTC_HH','UTC_mm','UTC_ss'};
    if all(ismember(need, Tg.Properties.VariableNames))
        utc = datetime(Tg.UTC_yyyy, Tg.UTC_MM, Tg.UTC_dd, Tg.UTC_HH, Tg.UTC_mm, Tg.UTC_ss, 'TimeZone','UTC');
        t0 = utc(1); tg = seconds(utc - t0); notes(end+1) = "GNSS: built from UTC_* columns";
    else
        tg = (0:nG-1)';   notes(end+1) = "GNSS: fallback uniform @1Hz";
    end
end
lines(end+1) = format_line('GNSS', tg, nG);

% ---------- TRUTH ----------
if ~isempty(truth_path) && isfile(truth_path)
    to = detectImportOptions(truth_path, 'Delimiter',' ', 'CommentStyle','#', 'ConsecutiveDelimitersRule','join');
    Ts = readtable(truth_path, to);
    ts = Ts{:,1};
    nS = numel(ts);
    lines(end+1) = format_line('TRUTH', ts, nS);
else
    lines(end+1) = "TRUTH | present but unreadable (see Notes).";
end

% ---------- Print + Save ----------
txt = strjoin(["== Timeline summary: " + rid + " =="; ""; lines; ""; "Notes:"; "- " + notes], newline);
disp(txt);

if ~exist(out_dir,'dir'), mkdir(out_dir); end
out_path = fullfile(out_dir, sprintf('%s_timeline.txt', rid));
fid = fopen(out_path,'w'); fprintf(fid,'%s\n',txt); fclose(fid);
fprintf('[DATA TIMELINE] Saved %s\n', out_path);
end

function s = format_line(tag, t, n)
t = t(:); dt = diff(t);
if isempty(dt) || ~all(isfinite(dt)), hz = NaN; med = NaN; mn = NaN; mx = NaN; mono = false;
else, hz = 1/median(dt); med = median(dt); mn = min(dt); mx = max(dt); mono = all(dt > 0);
end
dur = t(end) - t(1);
s = sprintf('%-5s | n=%-7d hz=%0.6f  dt_med=%0.6f  min/max dt=(%0.6f,%0.6f)  dur=%0.3fs  t0=%0.6f  t1=%0.6f  monotonic=%s',...
    tag, n, hz, med, mn, mx, dur, t(1), t(end), lower(string(mono)));
end
