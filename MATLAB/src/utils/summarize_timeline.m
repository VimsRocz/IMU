function out = summarize_timeline(imu_path, gnss_path, truth_path, out_dir, run_id)
% SUMMARIZE_TIMELINE  Emit simple timing statistics for dataset files.
% Usage:
%   out = summarize_timeline(imu_path, gnss_path, truth_path, out_dir, run_id)
%
% This helper mirrors the Python ``timeline.py`` utility. It inspects IMU,
% GNSS and optional truth files to compute nominal sample rates and timing
% irregularities. A human-readable text report is saved alongside a
% JSON-like struct in ``out``.
%
% Parameters
% ----------
% imu_path : char
%     Path to IMU .dat file.
% gnss_path : char
%     Path to GNSS .csv file.
% truth_path : char
%     Path to truth file; may be empty.
% out_dir : char
%     Output directory for timeline files.
% run_id : char
%     Identifier used in filenames.
%
% Returns
% -------
% out : struct
%     Structure containing timing summaries.
%
if nargin<5, run_id = 'run'; end
if nargin<4 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')),'..','..','results');
end
if ~exist(out_dir,'dir'), mkdir(out_dir); end

out = struct('run_id',run_id,'notes',{{}});

% -- GNSS (CSV)
Tg = readtable(gnss_path);
if any(strcmp(Tg.Properties.VariableNames,'Posix_Time'))
    t_gnss = Tg.Posix_Time;
else
    t_gnss = (0:height(Tg)-1)';           % assume 1 Hz if missing
    out.notes{end+1} = 'GNSS: Posix_Time not found; assumed 1 Hz via row index.';
end
out.gnss = rate_from_times(t_gnss);

% -- IMU (DAT)
Ti = readmatrix(imu_path);
t_imu = [];
% Try find a sub-second column [0..1)
for c = size(Ti,2):-1:max(1,size(Ti,2)-3)
    v = Ti(:,c);
    if all(isfinite(v)) && all(v>=0 & v<1)
        t_imu = unwrap_subsec(v);
        break;
    end
end
if isempty(t_imu)
    found=false;
    for c=1:min(6,size(Ti,2))
        v = Ti(:,c);
        if all(isfinite(v))
            dv = diff(v);
            if median(abs(dv))>1e-4 && median(abs(dv))<1
                t_imu = v; found=true; break;
            end
        end
    end
    if ~found
        dt = 0.0025;                       % fallback known for X002
        t_imu = (0:size(Ti,1)-1)'*dt;
        out.notes{end+1} = 'IMU: no time column; using 400 Hz fallback (dt=0.0025s).';
    end
end
out.imu = rate_from_times(t_imu);

% -- Truth
if ~isempty(truth_path) && isfile(truth_path)
    Ts = readmatrix(truth_path);
    t_truth = Ts(:,2); % second column = time
    out.truth = rate_from_times(t_truth);
else
    out.truth = struct('n',0);
end

% Save text + json-ish
txt = fullfile(out_dir, sprintf('%s_timeline.txt', run_id));
fid = fopen(txt,'w');
fprintf(fid,'== Timeline summary: %s ==\n', run_id);
write_line(fid,'IMU',out.imu);
write_line(fid,'GNSS',out.gnss);
if isfield(out,'truth') && isfield(out.truth,'n') && out.truth.n>0
    write_line(fid,'TRUTH',out.truth);
end
if ~isempty(out.notes)
    fprintf(fid,'\nNotes:\n');
    for i=1:numel(out.notes), fprintf(fid,'- %s\n', out.notes{i}); end
end
fclose(fid);
end

function s = rate_from_times(t)
t = t(:);
s.n = numel(t);
if numel(t)<2
    s.hz = NaN; s.dt_med=NaN; s.dt_min=NaN; s.dt_max=NaN; s.duration=0; s.t0=NaN; s.t1=NaN; s.monotonic=true; return;
end
dt = diff(t);
s.monotonic = all(dt>=-1e-9);
s.dt_med = median(dt);
s.dt_min = min(dt);
s.dt_max = max(dt);
if s.dt_med>0, s.hz = 1/s.dt_med; else, s.hz = NaN; end
s.duration = t(end)-t(1);
s.t0 = t(1); s.t1 = t(end);
end

function tw = unwrap_subsec(v)
tw = zeros(size(v));
acc=0; tw(1)=v(1);
for i=2:numel(v)
    dv = v(i)-v(i-1);
    if dv < -0.5, acc = acc + 1; end
    tw(i) = v(i) + acc;
end
end

function write_line(fid, tag, s)
fprintf(fid, '%6s | n=%d  hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%d\n', ...
    tag, s.n, s.hz, s.dt_med, s.dt_min, s.dt_max, s.duration, s.t0, s.t1, s.monotonic);
end
