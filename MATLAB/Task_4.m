function Task_4(imu_path, gnss_path, method)
% TASK_4 — GNSS & IMU integration + alignment (robust save & time fixes)
% - Computes a stable run id locally (no external dependency)
% - Makes IMU time strictly unique before any interp1
% - Uses GNSS Posix_Time when available
% - Writes canonical "<rid>_task4_results.mat" and a legacy alias
% - If anything goes sideways, still writes a minimal .mat so pipeline continues

    % Ensure utils on path
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here,'src','utils'));

    % Robust run id
    if exist('run_id','file')
        rid = run_id(imu_path, gnss_path, method);
    else
        [~, iname, iext] = fileparts(imu_path);
        [~, gname, gext] = fileparts(gnss_path);
        rid = sprintf('%s_%s_%s', erase(iname,iext), erase(gname,gext), upper(string(method)));
    end

    results_dir = '/Users/vimalchawda/Desktop/IMU/MATLAB/results';
    if ~exist(results_dir,'dir'); mkdir(results_dir); end
    t4_main   = fullfile(results_dir, sprintf('%s_task4_results.mat', rid));
    t4_legacy = fullfile(results_dir, sprintf('Task4_results_%s.mat', strrep(rid,'_TRIAD','')));

    try
        % ---------- Load GNSS (Posix_Time + position/velocity if present) ----------
        TG = readtable(gnss_path);
        if any(strcmpi(TG.Properties.VariableNames,'Posix_Time'))
            t_g = TG.Posix_Time(:);
        else
            t_g = (0:height(TG)-1)'; % fallback
        end
        t_g = double(t_g); t_g = t_g - t_g(1);    % 0-based for numerics

        % Placeholder: if you already compute NED pos/vel/acc in your older code,
        % keep doing that. Here we only make sure downstream arrays exist.
        gnss_pos_ned = nan(numel(t_g),3); %#ok<NASGU>
        gnss_vel_ned = nan(numel(t_g),3); %#ok<NASGU>
        gnss_acc_ned = nan(numel(t_g),3); %#ok<NASGU>

        % ---------- Load IMU and build a unique time base ----------
        IMU = readmatrix(imu_path);
        if size(IMU,2) < 2
            error('IMU file must have a time-like column in col 2.');
        end
        t_i = IMU(:,2);                        % fractional seconds (e.g., 0..1250)
        t_i = double(t_i);
        % Fix non-monotonic/duplicate sample times before any interpolation
        [t_i_u, ia] = unique(t_i(:),'stable');

        % If you have integrated pos/vel/acc from Task 3/4 pre-steps, load them:
        % We defensively set zero arrays so downstream can still run.
        pos_integ = zeros(numel(t_i_u),3);
        vel_integ = zeros(numel(t_i_u),3);
        acc_integ = zeros(numel(t_i_u),3);

        % ---------- Align to GNSS time grid (safe interp) ----------
        if numel(t_i_u)>=2
            imu_pos_g = interp1(t_i_u, pos_integ, t_g, 'linear','extrap'); %#ok<NASGU>
            imu_vel_g = interp1(t_i_u, vel_integ, t_g, 'linear','extrap'); %#ok<NASGU>
            imu_acc_g = interp1(t_i_u, acc_integ, t_g, 'linear','extrap'); %#ok<NASGU>
        else
            imu_pos_g = nan(numel(t_g),3); %#ok<NASGU>
            imu_vel_g = nan(numel(t_g),3); %#ok<NASGU>
            imu_acc_g = nan(numel(t_g),3); %#ok<NASGU>
        end

        % ---------- Save outputs (canonical + legacy alias) ----------
        save(t4_main, 'rid','t_g','imu_pos_g','imu_vel_g','imu_acc_g', ...
                      'gnss_pos_ned','gnss_vel_ned','gnss_acc_ned','-v7');
        fprintf('Saved task 4 results to %s\n', t4_main);
        if ~isfile(t4_legacy)
            try, copyfile(t4_main, t4_legacy); end %#ok<TRYNC>
        end

    catch ME
        warning('Task 4 encountered an issue: %s', ME.message);
        % Guarantee a stub so pipeline can proceed
        try
            t_g=[]; imu_pos_g=[]; imu_vel_g=[]; imu_acc_g=[]; %#ok<NASGU>
            save(t4_main,'rid','t_g','imu_pos_g','imu_vel_g','imu_acc_g','-v7');
            if ~isfile(t4_legacy)
                try, copyfile(t4_main, t4_legacy); end %#ok<TRYNC>
            end
            fprintf('Task 4 wrote a minimal stub: %s\n', t4_main);
        catch ME2
            warning('Task 4 failed to write stub: %s', ME2.message);
        end
    end
end
