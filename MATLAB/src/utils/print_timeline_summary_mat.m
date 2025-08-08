function out_txt = print_timeline_summary_mat(rid, imu_path, gnss_path, truth_path, out_dir)
%PRINT_TIMELINE_SUMMARY_MAT Print concise timing summary for datasets.
%   OUT_TXT = PRINT_TIMELINE_SUMMARY_MAT(RID, IMU_PATH, GNSS_PATH, TRUTH_PATH,
%   OUT_DIR) writes a text summary named "<RID>_timeline.txt" to OUT_DIR and
%   prints a 3-line summary block to the console. This mirrors the Python
%   ``timeline.print_timeline_summary`` function.
%
%   Usage:
%       print_timeline_summary_mat(rid, imu_path, gnss_path, truth_path, out_dir)
%
%   rid        - identifier string used in header and filename
%   imu_path   - path to IMU ``.dat`` file
%   gnss_path  - path to GNSS ``.csv`` file
%   truth_path - path to truth state file (optional)
%   out_dir    - directory to write ``<rid>_timeline.txt``
%
%   The function attempts to detect IMU and GNSS sampling rates and whether
%   their time vectors are strictly monotonic. If ``truth_path`` is provided
%   and readable, its timing information is summarized as well.

    if ~exist(out_dir,'dir'), mkdir(out_dir); end
    lines = {};

    % ---------- IMU ----------
    imu = readmatrix(imu_path);
    n_imu = size(imu,1);
    % try to find a [0,1) resetting clock; else synthesize @ 400 Hz
    t_imu = [];
    for c = 1:min(4,size(imu,2))
        col = imu(:,c);
        if all(isfinite(col)) && min(col) >= 0 && max(col) <= 1.0000001
            t_imu = unwrap_clock01(col, 0.0025);  % unwrap seconds
            break;
        end
    end
    if isempty(t_imu)
        t_imu = (0:n_imu-1)' * 0.0025; % fallback 400 Hz
    end
    dt_imu = diff(t_imu);
    hz_imu = 1/median(dt_imu);
    imu_line = sprintf('IMU   | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s',...
        n_imu, hz_imu, median(dt_imu), min(dt_imu), max(dt_imu), t_imu(end)-t_imu(1), t_imu(1), t_imu(end), tf(all(dt_imu>0)));

    % ---------- GNSS ----------
    T = readtable(gnss_path);
    if any(strcmp(T.Properties.VariableNames,'Posix_Time'))
        tg = T.Posix_Time;
    else
        tg = datetime(T.UTC_yyyy, T.UTC_MM, T.UTC_dd, T.UTC_HH, T.UTC_mm, T.UTC_ss);
        tg = seconds(tg - tg(1));
    end
    tg = tg(:);
    dtg = diff(tg);
    gnss_line = sprintf('GNSS  | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s',...
        numel(tg), 1/median(dtg), median(dtg), min(dtg), max(dtg), tg(end)-tg(1), tg(1), tg(end), tf(all(dtg>0)));

    % ---------- TRUTH ----------
    truth_line = 'TRUTH | present but unreadable (see Notes).';
    if ~isempty(truth_path) && isfile(truth_path)
        try
            opts = detectImportOptions(truth_path,'FileType','text','CommentStyle','#');
            S = readmatrix(truth_path, opts);
            tt = S(:,1); tt = tt(:);
            if numel(tt) > 1 && all(isfinite(tt))
                dtt = diff(tt);
                truth_line = sprintf('TRUTH | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s',...
                    numel(tt), 1/median(dtt), median(dtt), min(dtt), max(dtt), tt(end)-tt(1), tt(1), tt(end), tf(all(dtt>0)));
            end
        catch
            % keep default line
        end
    else
        truth_line = 'TRUTH | (not provided)';
    end

    head = sprintf('== Timeline summary: %s ==', rid);
    disp(head); disp(imu_line); disp(gnss_line); disp(truth_line);
    out_txt = fullfile(out_dir, sprintf('%s_timeline.txt', rid));
    fid = fopen(out_txt,'w');
    fprintf(fid, '%s\n%s\n%s\n%s\n', head, imu_line, gnss_line, truth_line);
    fclose(fid);
    fprintf('[DATA TIMELINE] Saved %s\n', out_txt);
end

function s = tf(x), if x, s='true'; else, s='false'; end, end

function t = unwrap_clock01(x, dt)
% unwrap a [0,1) repeating clock into continuous seconds
    wrap = [false; diff(x) < -0.5];
    k = cumsum(wrap);
    t = x + k;
    t = t - t(1);
    if abs(median(diff(t)) - dt) > 1e-4
        t = (0:numel(x)-1)' * dt;
    end
end

