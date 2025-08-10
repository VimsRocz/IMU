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

    % --- TRUTH (optional) ---
    if isempty(truth_path) || ~isfile(truth_path)
        lines{end+1} = 'TRUTH | missing (cfg.truth_path not set or path does not exist).';
    else
        try
            Truth = TruthLoader(truth_path, struct());
            if Truth.n > 0
                dt = diff(Truth.t_posix);
                hz_est = 1/median(dt);
                lines{end+1} = sprintf(['TRUTH | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  ' ...
                                       'dur=%.3fs  t0=%.6f  t1=%.6f  monotonic=%s'], ...
                                       Truth.n, hz_est, median(dt), min(dt), max(dt), ...
                                       Truth.t_posix(end)-Truth.t_posix(1), Truth.t_posix(1), Truth.t_posix(end), logical2str(all(dt>0)));
            else
                lines{end+1} = 'TRUTH | found but empty.';
            end
        catch ME
            lines{end+1} = 'TRUTH | present but unreadable.';
            lines{end+1} = sprintf('[TRUTH diagnostics] %s', ME.message);
            if isfield(ME,'cause') && ~isempty(ME.cause)
                for k = 1:numel(ME.cause)
                    lines{end+1} = sprintf('  cause %d: %s', k, ME.cause{k}.message);
                end
            end
        end
        if exist('Truth','var') && isfield(Truth,'notes') && ~isempty(Truth.notes)
            lines{end+1} = 'Notes:';
            for i = 1:numel(Truth.notes)
                lines{end+1} = ['  - ' Truth.notes{i}];
            end
        end
    end

    % --- Notes marker ---
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

