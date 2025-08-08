function print_timeline_matlab(rid, imu_path, gnss_path, truth_path, out_dir)
%PRINT_TIMELINE_MATLAB Summarise time vectors for IMU/GNSS/Truth datasets.
%   PRINT_TIMELINE_MATLAB(RID, IMU_PATH, GNSS_PATH, TRUTH_PATH, OUT_DIR)
%   prints a summary to the console and saves it to <RID>_timeline.txt under
%   OUT_DIR. The implementation is robust to missing columns and ignores
%   comments in the truth file.
%
%   Usage:
%       print_timeline_matlab('run_id', 'IMU.dat', 'GNSS.csv', 'STATE.txt', 'results');

lines = {};

% ---------- IMU ----------
imu = readmatrix(imu_path,'FileType','text');
nI = size(imu,1);
% detect time-like column (prefer strictly increasing, else assume 400 Hz)
col = [];
for c = 1:min(3,size(imu,2))  % try first 3 cols
    t = imu(:,c);
    if all(isfinite(t)) && (numel(unique(diff(t)))==1) && all(diff(t) > 0)
        col = c; break;
    end
end
if isempty(col)
    % assume synthetic time at 400 Hz
    tI = (0:nI-1)'/400;
    noteI = 'IMU: constructed time @400Hz';
else
    tI = imu(:,col);
    noteI = sprintf('IMU: used time-like column %d (median dt=%.6f)', col, median(diff(tI)));
end
lines = [lines; format_line('IMU', tI, nI)];

% ---------- GNSS ----------
opts = detectImportOptions(gnss_path,'Delimiter',',');
Tg = readtable(gnss_path, opts);
nG = height(Tg);

if any(strcmpi(Tg.Properties.VariableNames,'Posix_Time'))
    tg = Tg.Posix_Time;
    noteG = 'GNSS: used Posix_Time';
else
    % fall back: build from UTC parts if present
    need = {'UTC_yyyy','UTC_MM','UTC_dd','UTC_HH','UTC_mm','UTC_ss'};
    if all(ismember(need, Tg.Properties.VariableNames))
        utc = datetime(Tg.UTC_yyyy, Tg.UTC_MM, Tg.UTC_dd, Tg.UTC_HH, Tg.UTC_mm, Tg.UTC_ss, 'TimeZone','UTC');
        t0 = utc(1);
        tg = seconds(utc - t0);
        noteG = 'GNSS: built from UTC_* columns';
    else
        % last fallback: uniform @1Hz
        tg = (0:nG-1)';
        noteG = 'GNSS: fallback uniform @1Hz';
    end
end
lines = [lines; format_line('GNSS', tg, nG)];

% ---------- TRUTH ----------
if ~isempty(truth_path) && isfile(truth_path)
    % ignore comments (#)
    opts = detectImportOptions(truth_path, 'Delimiter',' ', 'CommentStyle','#', 'ConsecutiveDelimitersRule','join');
    Ts = readtable(truth_path, opts);
    ts = Ts{:,1};
    nS = numel(ts);
    lines = [lines; format_line('TRUTH', ts, nS)];
else
    lines = [lines; "TRUTH | present but unreadable (see Notes)."];
end

% ---------- Notes ----------
notes = {noteI; noteG};
if exist('noteS','var'), notes{end+1} = noteS; end

% print + save
txt = strjoin(["== Timeline summary: " + rid + " =="; ""; lines; ""; "Notes:"; "- " + string(notes)], newline);
disp(txt);

if ~exist(out_dir,'dir'), mkdir(out_dir), end
out_path = fullfile(out_dir, sprintf('%s_timeline.txt', rid));
fid = fopen(out_path,'w'); fprintf(fid,'%s\n',txt); fclose(fid);
fprintf('[DATA TIMELINE] Saved %s\n', out_path);

end

function s = format_line(tag, t, n)
t = t(:);
dt  = diff(t);
hz  = 1/median(dt);
dur = t(end)-t(1);
mono = all(dt > 0);
s = sprintf('%-5s | n=%-7d hz=%-10.6f dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s',...
    tag, n, hz, median(dt), min(dt), max(dt), dur, t(1), t(end), lower(string(mono)));
end

