function timeline_matlab(run_id, imu_path, gnss_path, truth_path)
%TIMELINE_MATLAB Print and save dataset timelines (IMU/GNSS/TRUTH).
%   TIMELINE_MATLAB(RUN_ID, IMU_PATH, GNSS_PATH, TRUTH_PATH) prints a
%   concise summary of the dataset timebases and writes it to
%   ``<results>/<run_id>_timeline.txt``. The summary mirrors
%   ``print_timeline_summary`` in ``src/utils/timeline.py``.
%
%   Usage:
%       timeline_matlab(run_id, imu_path, gnss_path, truth_path)
%
%   Inputs:
%       run_id    - identifier used in header
%       imu_path  - path to IMU data file
%       gnss_path - path to GNSS CSV file
%       truth_path- optional path to truth file

    paths = project_paths();
    out_txt = fullfile(paths.matlab_results, [run_id '_timeline.txt']);
    if exist(out_txt,'file'), delete(out_txt); end

    notes = strings(0,1);

    fprintf('%s\n', ['== Timeline summary: ' run_id ' ==']);

    % --- IMU ---
    imu = readmatrix(imu_path);
    [t_imu, n_i] = fix_time_vector(imu(:,2), 1/400);
    imu_dt = diff(t_imu);
    imu_hz = 1/median(imu_dt);
    fprintf('IMU   | n=%d  hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur≈%.3fs  monotonic=%s\n', ...
        numel(t_imu), imu_hz, median(imu_dt), min(imu_dt), max(imu_dt), t_imu(end)-t_imu(1), lower(string(all(imu_dt>0))));
    notes = [notes; n_i]; %#ok<AGROW>

    % --- GNSS ---
    Tg = readtable(gnss_path);
    if any(strcmpi(Tg.Properties.VariableNames, 'Posix_Time'))
        t_g_raw = Tg.Posix_Time;
    else
        t_g_raw = (0:height(Tg)-1)';
    end
    [t_gnss, n_g] = fix_time_vector(t_g_raw, 1);
    gnss_dt = diff(t_gnss);
    gnss_hz = 1/median(gnss_dt);
    fprintf('GNSS  | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur≈%.3fs  monotonic=%s\n', ...
        numel(t_gnss), gnss_hz, median(gnss_dt), min(gnss_dt), max(gnss_dt), t_gnss(end)-t_gnss(1), lower(string(all(gnss_dt>0))));
    notes = [notes; n_g]; %#ok<AGROW>

    % --- TRUTH ---
    truth_line = 'TRUTH | (not provided)';
    if ~isempty(truth_path) && isfile(truth_path)
        fid = fopen(truth_path,'r');
        C = textscan(fid,'%f%f%f%f%f%f%f%f%f%f','CommentStyle','#');
        fclose(fid);
        [t_truth, n_t] = fix_time_vector(C{1}, 0.1);
        if ~isempty(t_truth)
            truth_dt = diff(t_truth);
            truth_hz = 1/median(truth_dt);
            fprintf('TRUTH | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur≈%.3fs  monotonic=%s\n', ...
                numel(t_truth), truth_hz, median(truth_dt), min(truth_dt), max(truth_dt), t_truth(end)-t_truth(1), lower(string(all(truth_dt>0))));
            truth_line = '';
        else
            fprintf('TRUTH | present but unreadable.\n');
        end
        notes = [notes; n_t]; %#ok<AGROW>
    else
        fprintf('%s\n', truth_line);
    end

    % Save to file
    fid = fopen(out_txt,'w');
    fprintf(fid,'== Timeline summary: %s ==\n', run_id);
    fprintf(fid,'IMU   | n=%d  hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur≈%.3fs  monotonic=%s\n', ...
        numel(t_imu), imu_hz, median(imu_dt), min(imu_dt), max(imu_dt), t_imu(end)-t_imu(1), lower(string(all(imu_dt>0))));
    fprintf(fid,'GNSS  | n=%d    hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur≈%.3fs  monotonic=%s\n', ...
        numel(t_gnss), gnss_hz, median(gnss_dt), min(gnss_dt), max(gnss_dt), t_gnss(end)-t_gnss(1), lower(string(all(gnss_dt>0))));
    if exist('t_truth','var') && ~isempty(t_truth)
        fprintf(fid,'TRUTH | n=%d   hz=%.6f  dt_med=%.6f  min/max dt=(%.6f,%.6f)  dur≈%.3fs  monotonic=%s\n', ...
            numel(t_truth), truth_hz, median(truth_dt), min(truth_dt), max(truth_dt), t_truth(end)-t_truth(1), lower(string(all(truth_dt>0))));
    else
        fprintf(fid,'%s\n', truth_line);
    end
    if ~isempty(notes)
        fprintf(fid,'Notes: %s\n', strjoin(notes,'; '));
    else
        fprintf(fid,'Notes:\n');
    end
    fclose(fid);
    fprintf('[DATA TIMELINE] Saved %s\n', out_txt);
end
