function txt = print_timeline_matlab(rid, imu_path, gnss_path, truth_path, results_dir)
%PRINT_TIMELINE_MATLAB Show & save IMU/GNSS/TRUTH timing (robust to comments/# etc.)

    if ~exist(results_dir,'dir'), mkdir(results_dir); end
    lines = {};
    lines{end+1} = sprintf('== Timeline summary: %s ==', rid);
    lines{end+1} = '';

    % --- IMU (assume 400 Hz, build time from sample index) ---
    imu = readmatrix(imu_path,'FileType','text');
    nI  = size(imu,1);
    hzI = 400;  dtI = 1/hzI;
    tI  = (0:nI-1)'*dtI;
    dI  = diff(tI);
    lines{end+1} = sprintf(['IMU   | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
                            'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
                            nI, hzI, median(dI), min(dI), max(dI), tI(end)-tI(1), tI(1), tI(end), logical2str(issorted(tI)));

    % --- GNSS (uses Posix_Time) ---
    gnss = readtable(gnss_path);
    tG   = gnss.Posix_Time; tG = tG(:);
    dG   = diff(tG);
    lines{end+1} = sprintf(['GNSS  | n=%d     hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
                            'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
                            numel(tG), 1/median(dG), median(dG), min(dG), max(dG), tG(end)-tG(1), tG(1), tG(end), logical2str(issorted(tG)));

    % --- TRUTH (optional; handle # comments & headers) ---
    if exist('read_truth_state','file') && ~isempty(truth_path) && isfile(truth_path)
        tT = read_truth_state(truth_path);   % returns time vector (may be 0.1s ticks)
        % Normalize to seconds if needed (expect ~10 Hz)
        if ~isempty(tT)
            dtm = median(diff(tT));
            if isfinite(dtm) && dtm > 0.5 && dtm < 1.5
                tT = tT / 10;
            end
            tT = tT - tT(1);
        end
        dT = diff(tT);
        hzT = 1/max(eps,median(dT));
        lines{end+1} = sprintf(['TRUTH | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
                                'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
                                numel(tT), hzT, median(dT), min(dT), max(dT), tT(end)-tT(1), tT(1), tT(end), logical2str(issorted(tT)));
    else
        lines{end+1} = 'TRUTH | present but unreadable (see Notes).';
    end

    % --- Notes (example hook; fill as you like) ---
    lines{end+1} = '[DATA TIMELINE]';

    if ~iscellstr(lines), lines = cellfun(@char, lines, 'UniformOutput', false); end
    txt = strjoin(lines, newline);
    fprintf('%s\n', txt);

    out_txt = fullfile(results_dir, sprintf('%s_timeline.txt', rid));
    fid = fopen(out_txt,'w'); fprintf(fid,'%s\n', txt); fclose(fid);
end

function s = logical2str(tf)
    s = lower(string(tf));
    s = char(s);
end
