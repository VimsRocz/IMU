function timeline_txt = print_timeline_matlab(run_id_str, imu_path, gnss_path, truth_path, out_dir)
% PRINT_TIMELINE_MATLAB — emit IMU/GNSS/TRUTH timing summary and save .txt

    if ~exist(out_dir,'dir'); mkdir(out_dir); end

    % --- IMU: assume col2 is time (s) at ~400 Hz
    imu = readmatrix(imu_path);
    t_i = double(imu(:,2));
    di  = diff(t_i); di = di(isfinite(di));
    if isempty(di), di = 0; end
    hz_i = 1/median(di(di>0 | di<0 | di==0));  % safe even if single-valued
    imu_line = sprintf(['IMU   | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ', ...
        'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
        numel(t_i), hz_i, median(di), min(di), max(di), t_i(end)-t_i(1), t_i(1), t_i(end), lower(string(all(di>0))));

    % --- GNSS: use Posix_Time when present
    TG = readtable(gnss_path);
    if any(strcmpi(TG.Properties.VariableNames,'Posix_Time'))
        t_g = double(TG.Posix_Time);
    else
        t_g = (0:height(TG)-1)';  % fallback
    end
    dg = diff(t_g); dg = dg(isfinite(dg));
    if isempty(dg), dg = 0; end
    hz_g = 1/median(dg(dg>0 | dg<0 | dg==0));
    gnss_line = sprintf(['GNSS  | n=%d     hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ', ...
        'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
        numel(t_g), hz_g, median(dg), min(dg), max(dg), t_g(end)-t_g(1), t_g(1), t_g(end), lower(string(all(dg>0))));

    % --- TRUTH: ignore comment lines beginning with '#'
    if ~isempty(truth_path) && isfile(truth_path)
        fid = fopen(truth_path,'r');
        % read first numeric column as time, skipping comments
        C = textscan(fid,'%f%*[^\n]','CommentStyle','#','CollectOutput',true);
        fclose(fid);
        t_t = C{1};
        if isempty(t_t) || ~isfinite(t_t(1))
            truth_line = 'TRUTH | present but unreadable (see Notes).';
        else
            dt = diff(t_t); dt = dt(isfinite(dt));
            if isempty(dt), dt = 0; end
            hz_t = 1/median(dt(dt>0 | dt<0 | dt==0));
            truth_line = sprintf(['TRUTH | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ', ...
                'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
                numel(t_t), hz_t, median(dt), min(dt), max(dt), t_t(end)-t_t(1), t_t(1), t_t(end), lower(string(all(dt>0))));
        end
    else
        truth_line = 'TRUTH | present but unreadable (see Notes).';
    end

    header = sprintf('== Timeline summary: %s ==\n', run_id_str);
    lines = strjoin({header, imu_line, gnss_line, truth_line}, newline);
    disp(lines);

    fp = fullfile(out_dir, sprintf('%s_timeline.txt', run_id_str));
    fid = fopen(fp,'w'); fprintf(fid,'%s\n',lines); fclose(fid);
    fprintf('[DATA TIMELINE] Saved %s\n', fp);
    timeline_txt = fp;
end
