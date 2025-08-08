function timeline_txt = print_timeline_matlab(run_id_str, imu_path, gnss_path, truth_path, out_dir)
%PRINT_TIMELINE_MATLAB Print and save concise dataset timeline summary.
%   TIMELINE_TXT = PRINT_TIMELINE_MATLAB(RUN_ID, IMU_PATH, GNSS_PATH,
%   TRUTH_PATH, OUT_DIR) prints summary statistics for IMU, GNSS and truth
%   files. The summary is also written to ``OUT_DIR`` with filename
%   ``<run_id>_timeline.txt``. The returned value is the path to the saved
%   text file.

    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    notes = strings(0,1);

    % --- IMU: assume column 2 contains fractional seconds at 400 Hz ---
    imu = readmatrix(imu_path);
    [t_i, n_i] = fix_time_vector(imu(:,2), 1/400);
    dt_i = diff(t_i);
    hz_i = 1/median(dt_i);
    imu_line = sprintf(['IMU   | n=%d  hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
        'dur≈%.3fs  monotonic=%s'], ...
        numel(t_i), hz_i, median(dt_i), min(dt_i), max(dt_i), t_i(end)-t_i(1), lower(string(all(dt_i>0))));
    notes = [notes; n_i]; %#ok<AGROW>

    % --- GNSS: use Posix_Time column ---
    T = readtable(gnss_path);
    if any(strcmpi(T.Properties.VariableNames, 'Posix_Time'))
        t_g_raw = T.Posix_Time;
    else
        t_g_raw = (0:height(T)-1)';
    end
    [t_g, n_g] = fix_time_vector(t_g_raw, 1);
    dt_g = diff(t_g);
    hz_g = 1/median(dt_g);
    gnss_line = sprintf(['GNSS  | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
        'dur≈%.3fs  monotonic=%s'], ...
        numel(t_g), hz_g, median(dt_g), min(dt_g), max(dt_g), t_g(end)-t_g(1), lower(string(all(dt_g>0))));
    notes = [notes; n_g]; %#ok<AGROW>

    % --- TRUTH: ignore lines starting with '#'
    if ~isempty(truth_path) && isfile(truth_path)
        fid = fopen(truth_path,'r');
        C = textscan(fid,'%f%f%f%f%f%f%f%f%f%f','CommentStyle','#');
        fclose(fid);
        t_truth_raw = C{1};
        [t_t, n_t] = fix_time_vector(t_truth_raw, 0.1);
        if isempty(t_t)
            truth_line = 'TRUTH | present but unreadable (see Notes).';
        else
            dt_t = diff(t_t);
            hz_t = 1/median(dt_t);
            truth_line = sprintf(['TRUTH | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
                'dur≈%.3fs  monotonic=%s'], ...
                numel(t_t), hz_t, median(dt_t), min(dt_t), max(dt_t), t_t(end)-t_t(1), lower(string(all(dt_t>0))));
        end
        notes = [notes; n_t]; %#ok<AGROW>
    else
        truth_line = 'TRUTH | present but unreadable (see Notes).';
    end

    header = sprintf('== Timeline summary: %s ==\n', run_id_str);
    lines = {header, imu_line, gnss_line, truth_line};
    if ~isempty(notes)
        lines{end+1} = ['Notes: ' strjoin(notes, '; ')];
    else
        lines{end+1} = 'Notes:';
    end
    txt = strjoin(lines, newline);
    disp(txt);

    fp = fullfile(out_dir, sprintf('%s_timeline.txt', run_id_str));
    fid = fopen(fp,'w'); fprintf(fid,'%s\n',txt); fclose(fid);
    fprintf('[DATA TIMELINE] Saved %s\n', fp);
    timeline_txt = fp;
end

