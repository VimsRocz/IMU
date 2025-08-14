function run_triad_method(imu_file, gnss_file)
%RUN_TRIAD_METHOD  Run Tasks 1-5 using the TRIAD method on a single dataset.
%   RUN_TRIAD_METHOD(IMU_FILE, GNSS_FILE) loads the specified files from the
%   "Data" directory relative to the repository root.  Pass both filenames
%   explicitly.  An error is raised when either file does not exist.  This
%   helper mirrors the Python ``check_files`` functionality for parity.
%
%   Valid dataset pairs bundled with the repository are:
%       * IMU_X001.dat with GNSS_X001.csv
%       * IMU_X002.dat with GNSS_X002.csv
%       * IMU_X003.dat with GNSS_X002.csv (shared GNSS log)
%
%   Usage:
%       run_triad_method('IMU_X002.dat', 'GNSS_X002.csv')

    if nargin < 2
        error(['Usage: run_triad_method(IMU_FILE, GNSS_FILE)\n', ...
            'Example: run_triad_method(''IMU_X002.dat'', ''GNSS_X002.csv'')']);
    end

    data_dir = 'Data';
    imu_path = fullfile(data_dir, imu_file);
    gnss_path = fullfile(data_dir, gnss_file);
    [~, imu_name, ~]  = fileparts(imu_file);
    [~, gnss_name, ~] = fileparts(gnss_file);
    if ~exist(imu_path, 'file') || ~exist(gnss_path, 'file')
        error('File not found: %s or %s', imu_path, gnss_path);
    end

    % Load data
    imu_data = readmatrix(imu_path);
    gnss_data = readtable(gnss_path);

    % Proceed with TRIAD processing (Tasks 1--5)
    Task_1(imu_path, gnss_path, 'TRIAD');
    Task_2(imu_path, gnss_path, 'TRIAD');
    Task_3(imu_path, gnss_path, 'TRIAD');
    Task_4(imu_path, gnss_path, 'TRIAD');
    Task_5(imu_path, gnss_path, 'TRIAD');

    % Optional enhanced plots for Tasks 4--6 if data exists in workspace
    try
        hasVars = evalin('base', ['exist(''GNSS'',''var'') && exist(''IMU'',''var'') && ' ...
            'exist(''Fused'',''var'') && exist(''time_gnss'',''var'') && exist(''time_imu'',''var'')']);
        if hasVars
            GNSS  = evalin('base','GNSS');
            IMU   = evalin('base','IMU');
            Fused = evalin('base','Fused');
            if evalin('base','exist(''Truth'',''var'')');
                Truth = evalin('base','Truth');
            else
                Truth = struct();
            end
            time_gnss = evalin('base','time_gnss');
            time_imu  = evalin('base','time_imu');
            if evalin('base','exist(''Fused_triad'',''var'')');
                Fused_triad = evalin('base','Fused_triad');
            else
                Fused_triad = struct();
            end
            if evalin('base','exist(''Fused_davenport'',''var'')');
                Fused_davenport = evalin('base','Fused_davenport');
            else
                Fused_davenport = struct();
            end
            if evalin('base','exist(''Fused_svd'',''var'')');
                Fused_svd = evalin('base','Fused_svd');
            else
                Fused_svd = struct();
            end
            if evalin('base','exist(''time_truth'',''var'')');
                time_truth = evalin('base','time_truth');
            else
                time_truth = [];
            end
            plot_tasks_4_to_6(GNSS, IMU, Fused, Truth, time_gnss, time_imu, ...
                Fused_triad, Fused_davenport, Fused_svd, imu_name, gnss_name, 'TRIAD', time_truth);
        end
    catch ME
        warning('Enhanced plotting skipped: %s', ME.message);
    end
end

function plot_tasks_4_to_6(GNSS, IMU, Fused, Truth, time_gnss, time_imu, Fused_triad, Fused_davenport, Fused_svd, imu_name, gnss_name, method, time_truth)
%PLOT_TASKS_4_TO_6 Create enhanced Task 4--6 plots with PNG export and validation.
%   See repository documentation for expected structure of inputs.

    if nargin < 12
        error('plot_tasks_4_to_6:BadArgs', 'Missing required arguments');
    end
    if nargin < 13 || isempty(time_truth)
        time_truth = [];
    end

    set(groot, 'defaultAxesFontSize', 12);
    set(groot, 'defaultLineLineWidth', 1.5);

    results_dir = get_matlab_results_dir();
    if ~exist(results_dir, 'dir'); mkdir(results_dir); end
    prefix = sprintf('%s_%s_%s_', upper(imu_name), upper(gnss_name), upper(method));

    n_imu = size(getfield(IMU, 'ned').position, 1);
    n_gnss = size(getfield(GNSS, 'ned').position, 1);
    if ~isempty(Truth) && isfield(Truth, 'ned') && isfield(Truth.ned, 'position')
        n_truth = size(Truth.ned.position, 1);
    else
        n_truth = 0;
    end
    dataset_info = sprintf('IMU n=%d | GNSS n=%d | Truth n=%d', n_imu, n_gnss, n_truth);

    frames = struct( ...
        'ned',  struct('title', 'NED',  'axes', {{'North','East','Down'}}), ...
        'ecef', struct('title', 'ECEF', 'axes', {{'X','Y','Z'}}), ...
        'body', struct('title', 'Body', 'axes', {{'X','Y','Z'}}) );

    %% Task 4: Compare all signals
    frame_keys = fieldnames(frames);
    for k = 1:numel(frame_keys)
        key = frame_keys{k};
        fr  = frames.(key);
        title_str = sprintf('Task 4: All data in %s frame', fr.title);
        fname = sprintf('%stask4_all_%s.png', prefix, lower(fr.title));
        plot_task4_all(key, fr, title_str, fname);
    end
    fname = sprintf('%stask4_comparison_ned.png', prefix);
    plot_task4_velocity_ned(frames.ned, 'Task 4: NED velocity comparison', fname);

    %% Task 5: Fused results
    for k = 1:numel(frame_keys)
        key = frame_keys{k}; fr = frames.(key);
        title_str = sprintf('Task 5: Fused results (%s)', fr.title);
        fname = sprintf('%stask5_results_%s.png', prefix, lower(fr.title));
        plot_task5_fused(key, fr, title_str, fname);
    end
    fname = sprintf('%stask5_residuals_quicklook.png', prefix);
    plot_task5_residuals('Task 5: Residuals Quick-Look', fname);

    %% Task 6: Overlay fused vs truth and compare methods
    for k = 1:numel(frame_keys)
        key = frame_keys{k}; fr = frames.(key);
        title_str = sprintf('Task 6: Fused vs Truth (%s)', fr.title);
        fname = sprintf('%stask6_overlay_%s.png', prefix, upper(key));
        plot_task6_overlay(key, fr, title_str, fname);
    end
    for k = 1:numel(frame_keys)
        key = frame_keys{k}; fr = frames.(key);
        title_str = sprintf('Task 6: Compare methods (%s)', fr.title);
        fname = sprintf('%stask6_compare_methods_%s.png', prefix, upper(key));
        plot_task6_compare_methods(key, fr, title_str, fname);
    end

    %% Nested helpers -----------------------------------------------------

    function plot_task4_all(frame_key, frame_meta, task_title, file_name)
        labels = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
        quantities = {'position','velocity','acceleration'};
        fig = figure('Visible','off','Position',[100 100 1800 1200]);
        tl = tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
        notes = {};
        for r = 1:3
            q = quantities{r};
            for c = 1:3
                ax = nexttile; hold(ax,'on'); grid(ax,'on');
                [d, m] = get_series(GNSS, frame_key, q);
                if ~m
                    plot(ax, time_gnss, d(:,c), '-', 'DisplayName','GNSS');
                else
                    notes{end+1} = sprintf('GNSS %s %s missing', q, frame_meta.axes{c}); %#ok<AGROW>
                end
                [d, m] = get_series(IMU, frame_key, q);
                if ~m
                    plot(ax, time_imu, d(:,c), '--', 'DisplayName','IMU only');
                else
                    notes{end+1} = sprintf('IMU %s %s missing', q, frame_meta.axes{c}); %#ok<AGROW>
                end
                [d, m] = get_series(Fused, frame_key, q);
                if ~m
                    plot(ax, time_imu, d(:,c), '-', 'LineWidth',2, 'DisplayName','Fused');
                else
                    notes{end+1} = sprintf('Fused %s %s missing', q, frame_meta.axes{c}); %#ok<AGROW>
                end
                xlabel(ax,'Time [s]'); ylabel(ax, labels{r});
                title(ax, frame_meta.axes{c});
                hold(ax,'off');
            end
        end
        legend(tl, 'Location','northoutside','Orientation','horizontal');
        title_lines = {task_title; dataset_info};
        if ~isempty(notes)
            title_lines{end+1} = ['\color{red}⚠ ' strjoin(unique(notes), '; ')];
        end
        title(tl, title_lines);
        save_validated_png(fig, fullfile(results_dir, file_name));
        close(fig);
    end

    function plot_task4_velocity_ned(frame_meta, task_title, file_name)
        fig = figure('Visible','off','Position',[100 100 1800 1200]);
        tl = tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
        notes = {};
        [gnss_vel, mG] = get_series(GNSS, 'ned', 'velocity');
        [imu_vel,  mI] = get_series(IMU,  'ned', 'velocity');
        [fus_vel,  mF] = get_series(Fused,'ned', 'velocity');
        for i = 1:3
            ax = nexttile; hold(ax,'on'); grid(ax,'on');
            if ~mG, plot(ax, time_gnss, gnss_vel(:,i), '-', 'DisplayName','GNSS'); else, notes{end+1}='GNSS velocity missing'; end %#ok<AGROW>
            if ~mI, plot(ax, time_imu,  imu_vel(:,i), '--', 'DisplayName','IMU only'); else, notes{end+1}='IMU velocity missing'; end %#ok<AGROW>
            if ~mF, plot(ax, time_imu,  fus_vel(:,i), '-', 'LineWidth',2, 'DisplayName','Fused'); else, notes{end+1}='Fused velocity missing'; end %#ok<AGROW>
            ylabel(ax,'Velocity [m/s]'); title(ax, frame_meta.axes{i}); xlabel(ax,'Time [s]');
        end
        legend(tl,'Location','northoutside','Orientation','horizontal');
        title_lines = {task_title; dataset_info};
        if ~isempty(notes)
            title_lines{end+1} = ['\color{red}⚠ ' strjoin(unique(notes), '; ')];
        end
        title(tl, title_lines);
        save_validated_png(fig, fullfile(results_dir, file_name));
        close(fig);
    end

    function plot_task5_fused(frame_key, frame_meta, task_title, file_name)
        labels = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
        quantities = {'position','velocity','acceleration'};
        fig = figure('Visible','off','Position',[100 100 1800 1200]);
        tl = tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
        notes = {};
        for r = 1:3
            q = quantities{r};
            [d, m] = get_series(Fused, frame_key, q);
            for c = 1:3
                ax = nexttile; hold(ax,'on'); grid(ax,'on');
                if ~m
                    plot(ax, time_imu, d(:,c), '-', 'DisplayName','Fused');
                else
                    notes{end+1} = sprintf('Fused %s %s missing', q, frame_meta.axes{c}); %#ok<AGROW>
                end
                xlabel(ax,'Time [s]'); ylabel(ax, labels{r});
                title(ax, frame_meta.axes{c});
                hold(ax,'off');
            end
        end
        legend(tl, 'Fused', 'Location','northoutside','Orientation','horizontal');
        title_lines = {task_title; dataset_info};
        if ~isempty(notes)
            title_lines{end+1} = ['\color{red}⚠ ' strjoin(unique(notes), '; ')];
        end
        title(tl, title_lines);
        save_validated_png(fig, fullfile(results_dir, file_name));
        close(fig);
    end

    function plot_task5_residuals(task_title, file_name)
        fig = figure('Visible','off','Position',[100 100 1800 1200]);
        tl = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
        notes = {};
        [pos_f, mPf] = get_series(Fused, 'ned', 'position');
        [pos_g, mPg] = get_series(GNSS, 'ned', 'position');
        [vel_f, mVf] = get_series(Fused, 'ned', 'velocity');
        [vel_g, mVg] = get_series(GNSS, 'ned', 'velocity');
        if ~(mPf || mPg)
            pos_g_i = interp1(time_gnss, pos_g, time_imu, 'linear', 'extrap');
            res_pos = pos_f - pos_g_i;
            ax = nexttile; hold(ax,'on'); grid(ax,'on');
            plot(ax, time_imu, res_pos); xlabel(ax,'Time [s]'); ylabel(ax,'Position residual [m]');
            hold(ax,'off');
        else
            notes{end+1} = 'Position residual missing';
        end
        if ~(mVf || mVg)
            vel_g_i = interp1(time_gnss, vel_g, time_imu, 'linear', 'extrap');
            res_vel = vel_f - vel_g_i;
            ax = nexttile; hold(ax,'on'); grid(ax,'on');
            plot(ax, time_imu, res_vel); xlabel(ax,'Time [s]'); ylabel(ax,'Velocity residual [m/s]');
            hold(ax,'off');
        else
            notes{end+1} = 'Velocity residual missing';
        end
        legend(tl, 'Fused - GNSS', 'Location','northoutside','Orientation','horizontal');
        title_lines = {task_title; dataset_info};
        if ~isempty(notes)
            title_lines{end+1} = ['\color{red}⚠ ' strjoin(unique(notes), '; ')];
        end
        title(tl, title_lines);
        save_validated_png(fig, fullfile(results_dir, file_name));
        close(fig);
    end

    function plot_task6_overlay(frame_key, frame_meta, task_title, file_name)
        labels = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
        quantities = {'position','velocity','acceleration'};
        fig = figure('Visible','off','Position',[100 100 1800 1200]);
        tl = tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
        notes = {};
        for r = 1:3
            q = quantities{r};
            [df, mf] = get_series(Fused, frame_key, q);
            [dt, mt] = get_series(Truth, frame_key, q);
            for c = 1:3
                ax = nexttile; hold(ax,'on'); grid(ax,'on');
                if ~mf
                    plot(ax, time_imu, df(:,c), '-', 'LineWidth',2, 'DisplayName','Fused');
                else
                    notes{end+1} = sprintf('Fused %s %s missing', q, frame_meta.axes{c}); %#ok<AGROW>
                end
                if ~mt && ~isempty(time_truth)
                    plot(ax, time_truth, dt(:,c), ':', 'DisplayName','Truth');
                else
                    notes{end+1} = sprintf('Truth %s %s missing', q, frame_meta.axes{c}); %#ok<AGROW>
                end
                xlabel(ax,'Time [s]'); ylabel(ax, labels{r});
                title(ax, frame_meta.axes{c});
                hold(ax,'off');
            end
        end
        legend(tl, 'Location','northoutside','Orientation','horizontal');
        title_lines = {task_title; dataset_info};
        if ~isempty(notes)
            title_lines{end+1} = ['\color{red}⚠ ' strjoin(unique(notes), '; ')];
        end
        title(tl, title_lines);
        save_validated_png(fig, fullfile(results_dir, file_name));
        close(fig);
    end

    function plot_task6_compare_methods(frame_key, frame_meta, task_title, file_name)
        labels = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
        quantities = {'position','velocity','acceleration'};
        fig = figure('Visible','off','Position',[100 100 1800 1200]);
        tl = tiledlayout(fig,3,3,'TileSpacing','compact','Padding','compact');
        notes = {};
        methods = {Fused_triad, Fused_davenport, Fused_svd};
        names   = {'TRIAD','Davenport','SVD'};
        for r = 1:3
            q = quantities{r};
            for c = 1:3
                ax = nexttile; hold(ax,'on'); grid(ax,'on');
                for m = 1:3
                    [d, miss] = get_series(methods{m}, frame_key, q);
                    if ~miss
                        plot(ax, time_imu, d(:,c), '-', 'DisplayName', names{m});
                    else
                        notes{end+1} = sprintf('%s %s %s missing', names{m}, q, frame_meta.axes{c}); %#ok<AGROW>
                    end
                end
                xlabel(ax,'Time [s]'); ylabel(ax, labels{r});
                title(ax, frame_meta.axes{c});
                hold(ax,'off');
            end
        end
        legend(tl, names, 'Location','northoutside','Orientation','horizontal');
        title_lines = {task_title; dataset_info};
        if ~isempty(notes)
            title_lines{end+1} = ['\color{red}⚠ ' strjoin(unique(notes), '; ')];
        end
        title(tl, title_lines);
        save_validated_png(fig, fullfile(results_dir, file_name));
        close(fig);
    end

    function [data, missing] = get_series(S, frame_key, field)
        data = [];
        missing = true;
        if ~isstruct(S) || ~isfield(S, frame_key)
            return; end
        F = S.(frame_key);
        if ~isfield(F, field); return; end
        data = F.(field);
        if isempty(data) || all(isnan(data(:))) || all(abs(data(:)) < eps)
            data = []; return; end
        missing = false;
    end

    function save_validated_png(fig, file_path)
        exportgraphics(fig, file_path, 'Resolution',200);
        d = dir(file_path);
        if isempty(d) || d.bytes < 5*1024
            error('Export failed for %s (file too small)', file_path);
        else
            fprintf('Saved plot to %s (%d bytes)\n', file_path, d.bytes);
        end
    end

end

