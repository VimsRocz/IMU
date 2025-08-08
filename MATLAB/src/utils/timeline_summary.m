function timeline_summary(run_id, imu_path, gnss_path, truth_path, results_dir)
% TIMELINE_SUMMARY Print dataset timing overview and save artifacts.
%   timeline_summary(run_id, imu_path, gnss_path, truth_path, results_dir)
%   prints a concise table describing the IMU, GNSS and optional truth
%   timelines, writes ``*_timeline.txt`` and ``*_timeline.mat`` in
%   *results_dir*. A Python counterpart is implemented in
%   ``src/utils/timeline.py``.

if ~exist(results_dir,'dir'), mkdir(results_dir); end
notes = strings(0,1);

% --- IMU time -------------------------------------------------------------
try
    M = readmatrix(imu_path);
catch ME
    warning("IMU read failed (%s); fallback to dt=0.0025", ME.message);
    M = nan(500000,1);
end
[t_imu, note] = detect_imu_time(M, 0.0025);
if strlength(note)>0, notes(end+1) = note; end
if any(diff(t_imu)<0), notes(end+1)="IMU: time not strictly increasing after unwrap"; end

% --- GNSS time ------------------------------------------------------------
Tg = readtable(gnss_path);
gnssCols = Tg.Properties.VariableNames;
t_gnss = [];
for name = ["Posix_Time","posix_time","time","Time","TIME","gps_time","GPSTime"]
    if any(strcmp(gnssCols, name))
        t_gnss = Tg.(name)(:);
        notes(end+1) = "GNSS: used '"+name+"' column";
        break
    end
end
if isempty(t_gnss)
    t_gnss = (0:height(Tg)-1)';  % assume 1 Hz
    notes(end+1) = "GNSS: no time column; assume 1 Hz";
end

% --- TRUTH time -----------------------------------------------------------
t_truth = [];
if nargin>=4 && ~isempty(truth_path) && isfile(truth_path)
    try
        Td = readtable(truth_path, "FileType","text");
    catch
        Td = readtable(truth_path, "FileType","text", "Delimiter"," ");
    end
    cols = Td.Properties.VariableNames;
    found = false;
    for name = ["time","Time","t","T","posix","Posix_Time","sec","seconds"]
        if any(strcmp(cols,name))
            t_truth = Td.(name)(:);
            notes(end+1) = "TRUTH: used '"+name+"' column";
            found = true; break
        end
    end
    if ~found
        c0 = Td{:,1};
        if all(isfinite(diff(c0))) && median(abs(diff(c0)))>1e-5
            t_truth = c0(:);
            notes(end+1) = "TRUTH: used column 0 as time";
        else
            t_truth = (0:height(Td)-1)' * 0.1; % 10 Hz synthetic
            notes(end+1) = "TRUTH: no time; assume 10 Hz";
        end
    end
else
    t_truth = [];
end

% --- stats & printing -----------------------------------------------------
sIMU  = stats(t_imu);
sGNSS = stats(t_gnss);
sTRU  = stats(t_truth);

header = sprintf("== Timeline summary: %s ==", run_id);
L = strings(0,1);
L(end+1) = header;
L(end+1) = fmtLine("IMU",  sIMU);
L(end+1) = fmtLine("GNSS", sGNSS);
L(end+1) = fmtLine("TRUTH",sTRU);
if isempty(notes)
    L(end+1) = "Notes: (none)";
else
    L(end+1) = "Notes:";
    for k=1:numel(notes), L(end+1) = "- " + notes(k); end
end

fprintf("%s\n", join(L,newline));

txt = fullfile(results_dir, run_id+"_timeline.txt");
fid = fopen(txt,'w'); fprintf(fid, "%s\n", join(L,newline)); fclose(fid);
save(fullfile(results_dir, run_id+"_timeline.mat"), "t_imu","t_gnss","t_truth","sIMU","sGNSS","sTRU","notes");

end

% -------------------------------------------------------------------------
function [t, note] = detect_imu_time(M, dt_fallback)
%DETECT_IMU_TIME Infer IMU time column or synthesize one.

note = "";
% 1) sub-second column near the end
for c = size(M,2):-1:max(1,size(M,2)-3)
    col = M(:,c);
    if all(isfinite(col)) && min(col)>=0 && max(col)<1
        t = unwrap_subsec(col);
        note = "IMU: used sub-second column " + string(c) + " with unwrap()";
        return
    end
end

% 2) time-like column near the front
for c = 1:min(6,size(M,2))
    col = M(:,c);
    if all(isfinite(col))
        d = diff(col);
        med = median(abs(d));
        if med>1e-4 && med<1
            t = col(:);
            note = "IMU: used time-like column " + string(c) + " (median dt=" + num2str(med,'%.6f') + ")";
            return
        end
    end
end

% 3) fallback uniform rate
n = size(M,1);
t = (0:n-1)' * dt_fallback;
note = "IMU: no time column; fallback uniform dt=" + num2str(dt_fallback);
end

function out = unwrap_subsec(v)
out = zeros(size(v));
wraps = 0;
out(1) = v(1);
for i=2:numel(v)
    dv = v(i) - v(i-1);
    if dv < -0.5, wraps = wraps + 1; end
    out(i) = v(i) + wraps;
end
end

function s = stats(t)
if isempty(t)
    s = struct('n',0,'hz',NaN,'dt_med',NaN,'dt_min',NaN,'dt_max',NaN,'dur',NaN,'t0',NaN,'t1',NaN,'monotonic',false);
    return
end
t = double(t(:));
n = numel(t);
if n<2
    s = struct('n',n,'hz',NaN,'dt_med',NaN,'dt_min',NaN,'dt_max',NaN,'dur',0,'t0',t(1),'t1',t(end),'monotonic',true);
    return
end
dt = diff(t);
dt_med = median(dt);
hz = 1/dt_med;
s = struct('n',n,'hz',hz,'dt_med',dt_med,'dt_min',min(dt),'dt_max',max(dt), ...
           'dur',t(end)-t(1),'t0',t(1),'t1',t(end),'monotonic',all(dt>0));
end

function line = fmtLine(label, s)
if s.n==0
    line = sprintf("%-6s| (missing)", label);
else
    line = sprintf("%-6s| n=%-7d  hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s", ...
        label, s.n, s.hz, s.dt_med, s.dt_min, s.dt_max, s.dur, s.t0, s.t1, string(s.monotonic));
end
end

