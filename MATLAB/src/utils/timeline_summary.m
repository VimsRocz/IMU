function timeline_summary(run_id, imu_path, gnss_path, truth_path, out_txt)
%TIMELINE_SUMMARY Print and save dataset timelines (IMU/GNSS/TRUTH).
%   TIMELINE_SUMMARY(RUN_ID, IMU_PATH, GNSS_PATH, TRUTH_PATH, OUT_TXT)
%   prints a concise summary of the dataset timebases and writes it to
%   OUT_TXT. The summary mirrors ``print_timeline_summary`` in
%   ``src/utils/timeline.py``.
%
%   Usage:
%       timeline_summary(run_id, imu_path, gnss_path, truth_path, out_txt)
%
%   Inputs:
%       run_id    - identifier used in header
%       imu_path  - path to IMU data file
%       gnss_path - path to GNSS CSV file
%       truth_path- optional path to truth file
%       out_txt   - output text file path

fprintf('%s\n', ['== Timeline summary: ' run_id ' ==']);

% --- IMU ---
imu = readmatrix(imu_path);
% Heuristic: if 2nd column looks like fractional secs, unwrap it
t_imu_raw = imu(:,2);
dt = diff(t_imu_raw);
looks_wrap = any(dt < -0.5) || any(dt > 0.5);
if looks_wrap
    dt_hint = 1/round(1/median(abs(dt(abs(dt)>0 & abs(dt)<0.5)), 'omitnan'));
    if ~isfinite(dt_hint) || dt_hint<=0, dt_hint = 0.0025; end
    t_imu = unwrap_seconds(t_imu_raw, dt_hint);
else
    t_imu = t_imu_raw - t_imu_raw(1);
end
imu_dt = diff(t_imu);
imu_hz = 1/median(imu_dt, 'omitnan');
fprintf('IMU   | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s\n',...
    numel(t_imu), imu_hz, median(imu_dt,'omitnan'), min(imu_dt), max(imu_dt), t_imu(end)-t_imu(1), t_imu(1), t_imu(end), string(all(imu_dt>0)));

% --- GNSS ---
Tg = readtable(gnss_path);
t_gnss = Tg.Posix_Time - Tg.Posix_Time(1);
gnss_dt = diff(t_gnss);
gnss_hz = 1/median(gnss_dt,'omitnan');
fprintf('GNSS  | n=%d     hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s\n',...
    numel(t_gnss), gnss_hz, median(gnss_dt,'omitnan'), min(gnss_dt), max(gnss_dt), t_gnss(end)-t_gnss(1), t_gnss(1), t_gnss(end), string(all(gnss_dt>0)));

% --- TRUTH ---
notes = {};
truth_line = 'TRUTH | (not provided)';
if ~isempty(truth_path) && isfile(truth_path)
    t_truth = read_truth_time(truth_path, notes);
    if ~isempty(t_truth)
        truth_dt = diff(t_truth);
        truth_hz = 1/median(truth_dt,'omitnan');
        fprintf('TRUTH | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s\n',...
            numel(t_truth), truth_hz, median(truth_dt,'omitnan'), min(truth_dt), max(truth_dt), t_truth(end)-t_truth(1), t_truth(1), t_truth(end), string(all(truth_dt>0)));
    else
        fprintf('TRUTH | present but unreadable (see Notes).\n');
    end
else
    fprintf('%s\n', truth_line);
end

% Save to file
if nargin>=5 && ~isempty(out_txt)
    fid = fopen(out_txt,'w');
    fprintf(fid,'== Timeline summary: %s ==\n', run_id);
    fprintf(fid,'IMU   | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s\n',...
        numel(t_imu), imu_hz, median(imu_dt,'omitnan'), min(imu_dt), max(imu_dt), t_imu(end)-t_imu(1), t_imu(1), t_imu(end), string(all(imu_dt>0)));
    fprintf(fid,'GNSS  | n=%d     hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s\n',...
        numel(t_gnss), gnss_hz, median(gnss_dt,'omitnan'), min(gnss_dt), max(gnss_dt), t_gnss(end)-t_gnss(1), t_gnss(1), t_gnss(end), string(all(gnss_dt>0)));
    if exist('t_truth','var') && ~isempty(t_truth)
        fprintf(fid,'TRUTH | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s\n',...
            numel(t_truth), truth_hz, median(truth_dt,'omitnan'), min(truth_dt), max(truth_dt), t_truth(end)-t_truth(1), t_truth(1), t_truth(end), string(all(truth_dt>0)));
    elseif ~isempty(truth_path) && isfile(truth_path)
        fprintf(fid,'TRUTH | present but unreadable (see Notes).\n');
    else
        fprintf(fid,'%s\n', truth_line);
    end
    fclose(fid);
    fprintf('[DATA TIMELINE] Saved %s\n', out_txt);
end
end
