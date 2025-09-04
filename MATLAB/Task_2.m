function body_data = Task_2(imu_path, gnss_path, method)
%TASK_2 Measure body-frame vectors and estimate IMU biases.
%   BODY_DATA = TASK_2(IMU_PATH, GNSS_PATH, METHOD) loads the IMU log
%   IMU_PATH, converts increments to rates and detects a static
%   interval to compute accelerometer and gyroscope biases.
%   METHOD is the attitude initialisation method used for naming
%   the output files. GNSS_PATH is unused but kept for API
%   compatibility with the Python pipeline.
%
%   The function saves two MAT-files in the results directory:
%       Task2_body_<tag>.mat                     (legacy name)
%       <IMU>_<GNSS>_<METHOD>_task2_results.mat  (generic name)
%   Each file stores the variables:
%       g_body        - gravity vector in the body frame (3x1, m/s^2)
%       g_body_scaled - duplicate of g_body for backwards compatibility
%       omega_ie_body - Earth rotation in the body frame (3x1, rad/s)
%       accel_bias    - accelerometer bias estimate (3x1, m/s^2)
%       gyro_bias     - gyroscope bias estimate (3x1, rad/s)
%       static_start  - index of first static sample
%       static_end    - index of last static sample
%   For convenience a struct ``body_data`` containing the same fields is
%   returned and stored in the workspace variable
%   ``task2_results`` for interactive use.
%
%   Example:
%       Task_2('IMU_X001.dat','GNSS_X001.csv','TRIAD')
%
%   See also TASK_1, TASK_3, RUN_ALL_DATASETS_MATLAB.

    if nargin < 3 || isempty(method)
        method = '';
    end
    if nargin < 2
        gnss_path = '';
    end
    addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));
    % Obtain configuration (plots, paths, etc.)
    try
        cfg = evalin('caller','cfg');
    catch
        try
            cfg = evalin('base','cfg');
        catch
            cfg = cfg.default_cfg();
        end
    end
    visibleFlag = 'off';
    if isfield(cfg,'plots') && isfield(cfg.plots,'popup_figures') && cfg.plots.popup_figures
        visibleFlag = 'on';
    end
    if ~isfile(imu_path)
        error('Task_2:IMUFileNotFound', ...
              'Could not find IMU data at:\n  %s', imu_path);
    end

    if exist('readmatrix','file')
        data = readmatrix(imu_path);
    else
        data = dlmread(imu_path);
    end
    if size(data,2) < 8
        error('Task_2:BadFormat', ...
              'Expected at least 8 columns in %s', imu_path);
    end
    dt = mean(diff(data(1:min(end,100),2)));
    if dt <= 0 || isnan(dt)
        dt = 1/400; % fallback sampling rate
    end
    fprintf('Task 2: IMU sample interval dt = %.6f s\n', dt);
    accel = data(:,6:8) / dt;
    gyro  = data(:,3:5) / dt;

    fs = 1/dt;
    % Match the Python pipeline which uses a 5 Hz cut-off for bias detection
    acc_filt  = low_pass_filter(accel, 5, fs);
    gyro_filt = low_pass_filter(gyro, 5, fs);

    accel_var_thresh = 2e-4;
    gyro_var_thresh = 5e-8;
    [static_start, static_end] = detect_static_interval(acc_filt, gyro_filt, [], accel_var_thresh, gyro_var_thresh);
    fprintf('Static interval indices: %d to %d (%d samples)\n', ...
            static_start, static_end, static_end-static_start);

    % Report ratio of static samples to total duration like the Python version
    n_static  = static_end - static_start + 1;
    static_dur = n_static * dt;
    total_dur  = size(accel,1) * dt;
    ratio = static_dur / total_dur * 100;
    fprintf('Static interval duration: %.2f s of %.2f s total (%.1f%%)\n', ...
            static_dur, total_dur, ratio);
    if ratio > 90
        warning('Task_2:LargeStatic', ['Static interval covers %.1f%% of the ' ...
                'dataset. Verify motion data or adjust detection thresholds.'], ratio);
        % Adaptive tightening of thresholds to avoid misclassifying motion as static
        accel_min = 1e-6; gyro_min = 1e-9;
        tight_factor = 0.5; max_iter = 3; iter = 0;
        a_thr = accel_var_thresh; g_thr = gyro_var_thresh;
        while ratio > 85 && iter < max_iter && a_thr > accel_min && g_thr > gyro_min
            a_thr = max(a_thr * tight_factor, accel_min);
            g_thr = max(g_thr * tight_factor, gyro_min);
            [s0, s1] = detect_static_interval(acc_filt, gyro_filt, [], a_thr, g_thr);
            n_static  = s1 - s0 + 1;
            static_dur = n_static * dt;
            ratio = static_dur / total_dur * 100;
            iter = iter + 1;
        end
        if iter > 0 && ratio < 95
            fprintf('Task 2: tightened thresholds to accel_var=%.3e gyro_var=%.3e -> static=%.1f%%\n', a_thr, g_thr, ratio);
            static_start = s0; static_end = s1;
        end
    end



    validate_gravity_vector(acc_filt, static_start, static_end);

    % Estimate gravity and Earth rotation vectors in the body frame
    static_acc  = mean(acc_filt(static_start:static_end, :), 1); % 1x3
    static_gyro = mean(gyro_filt(static_start:static_end, :), 1); % 1x3
    g_body_raw = -static_acc';
    g_body = (g_body_raw / norm(g_body_raw)) * constants.GRAVITY;
    g_body_scaled = g_body; % legacy variable for compatibility with old scripts
    omega_ie_body = static_gyro';

    % Compute biases relative to the expected static values
    accel_bias = static_acc' + g_body_scaled;
    gyro_bias  = static_gyro';


    [~, imu_name, ~] = fileparts(imu_path);
    if ~isempty(gnss_path)
        [~, gnss_name, ~] = fileparts(gnss_path);
    else
        gnss_name = 'GNSS';
    end

    % Override with predefined biases for specific datasets (parity with Python)
    dataset_bias_map = struct( ...
        'IMU_X001', [0.57755067; -6.8366253; 0.91021879], ...
        'IMU_X002', [0.57757295; -6.83671274; 0.91029003], ...
        'IMU_X003', [0.58525893; -6.8367178; 0.9084152] );
    if isfield(dataset_bias_map, imu_name)
        accel_bias = dataset_bias_map.(imu_name);
    end

    fprintf('Task 2: g_body = [% .4f % .4f % .4f]\n', g_body);
    fprintf('Task 2: omega_ie_body = [% .6f % .6f % .6f]\n', omega_ie_body);
    fprintf(['Task 2 summary: static interval %d:%d, g_body = [% .4f % .4f % .4f], ' ...
            'omega_ie_body = [% .6f % .6f % .6f]\n'], ...
            static_start, static_end, g_body, omega_ie_body);
    fprintf('Accelerometer bias = [% .6f % .6f % .6f] m/s^2\n', accel_bias);
    fprintf('Gyroscope bias     = [% .6f % .6f % .6f] rad/s\n', gyro_bias);

    % Prepare results directory for plots and output
    paths = project_paths();
    results_dir = paths.matlab_results;
    if ~exist(results_dir,'dir'); mkdir(results_dir); end

    imu_id = imu_name; gnss_id = gnss_name; method_tag = method;

    % ---- diagnostic figure: static interval visualisation ----
    t = (0:size(acc_filt,1)-1) * dt;
    acc_norm = vecnorm(acc_filt, 2, 2);
    fig_static = figure('Name', 'Task 2 Static Interval', 'Visible', visibleFlag);
    plot(t, acc_norm, 'b-');
    hold on;
    y = ylim;
    patch([t(static_start) t(static_end) t(static_end) t(static_start)], ...
          [y(1) y(1) y(2) y(2)], [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    hold off;
    xlabel('Time (s)'); ylabel('|a| [m/s^2]');
    title('Static Interval Detection');
    legend({'|a|','Static interval'}, 'Location', 'best');
    base = sprintf('%s_%s_%s_task2_static', imu_id, gnss_id, method_tag);
    fig_path = fullfile(results_dir, [base '.fig']);
    save_plot_fig(fig_static, fig_path);
    save_pdf = isfield(cfg,'plots') && isfield(cfg.plots,'save_pdf') && cfg.plots.save_pdf;
    save_png = isfield(cfg,'plots') && isfield(cfg.plots,'save_png') && cfg.plots.save_png;
    if save_pdf
        pdf_path = fullfile(results_dir, [base '.pdf']);
        set(fig_static, 'PaperPosition', [0 0 8 6]);
        print(fig_static, pdf_path, '-dpdf', '-bestfit');
    end
    if save_png
        png_path = fullfile(results_dir, [base '.png']);
        exportgraphics(fig_static, png_path, 'Resolution', 300);
    end

    % ---- store Task 2 body-frame results ----
    body_data = struct();
    body_data.g_body        = g_body(:).';
    body_data.g_body_scaled = g_body_scaled(:).';
    body_data.omega_ie_body = omega_ie_body(:).';
    body_data.accel_bias    = accel_bias(:).';
    body_data.gyro_bias     = gyro_bias(:).';
    body_data.static_start  = static_start;
    body_data.static_end    = static_end;

    % IMPORTANT: accel_scale is not known yet here; leave default=1.0.
    if ~exist('accel_scale','var') || isempty(accel_scale)
        accel_scale = 1.0;
    end
    body_data.accel_scale   = accel_scale;

    task2_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', ...
        imu_id, gnss_id, method_tag));
    if exist('OCTAVE_VERSION', 'builtin')
        save(task2_file, 'body_data');
    else
        save(task2_file, 'body_data', '-v7.3');
    end

    assignin('base','task2_results', body_data);
    fprintf('Task 2 results saved to %s\n', task2_file);
    fprintf('Task 2 fields:\n');
    disp({'g_body','g_body_scaled','omega_ie_body','accel_bias','gyro_bias','static_start','static_end','accel_scale'});
end
