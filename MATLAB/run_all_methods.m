function run_all_methods(datasets)
%RUN_ALL_METHODS Run GNSS/IMU fusion for multiple dataset pairs.
%   RUN_ALL_METHODS() processes the default datasets with all initialisation
%   methods. A custom cell array of structs can be provided in the same
%   format as ``DEFAULT_DATASETS`` below.
%
%   Each element of ``datasets`` must be a struct with fields ``imu`` and
%   ``gnss`` specifying the filenames relative to the ``Data/`` directory.
%
%   Example:
%       run_all_methods();
%       run_all_methods({struct('imu','IMU_X002.dat','gnss','GNSS_X002.csv')});
%
%   This mirrors the behaviour of ``src/run_all_methods.py``.

    DEFAULT_DATASETS = {...
        struct('imu', 'IMU_X001.dat', 'gnss', 'GNSS_X001.csv'), ...
        struct('imu', 'IMU_X002.dat', 'gnss', 'GNSS_X002.csv'), ...
        struct('imu', 'IMU_X003.dat', 'gnss', 'GNSS_X002.csv')};

    if nargin < 1 || isempty(datasets)
        datasets = DEFAULT_DATASETS;
    end

    for i = 1:numel(datasets)
        imu_file = datasets{i}.imu;
        gnss_file = datasets{i}.gnss;
        fprintf('Processing %s and %s\n', imu_file, gnss_file);
        process_dataset(imu_file, gnss_file);
    end
end

function process_dataset(imu_file, gnss_file)
%PROCESS_DATASET Run Tasks 1--5 for a single dataset pair.
%   PROCESS_DATASET(IMU_FILE, GNSS_FILE) performs the full initialisation
%   pipeline with all attitude initialisation methods and saves the results
%   to the ``results`` directory.

    data_dir = fullfile('Data');
    imu_path  = fullfile(data_dir, imu_file);
    gnss_path = fullfile(data_dir, gnss_file);
    if ~isfile(imu_path)
        imu_path = get_data_file(imu_file);
    end
    if ~isfile(gnss_path)
        gnss_path = get_data_file(gnss_file);
    end
    if ~isfile(imu_path) || ~isfile(gnss_path)
        error('File not found: %s or %s', imu_file, gnss_file);
    end

    if size(readmatrix(imu_path),1) < 10
        warning('IMU data length shorter than expected');
    end

    methods = {'TRIAD','Davenport','SVD'};
    colors  = {'r','g','b'}; %#ok<NASGU> % reserved for plotting
    resultsDir = get_results_dir();
    if ~exist(resultsDir,'dir'); mkdir(resultsDir); end

    [~, imu_name, ~]  = fileparts(imu_path);
    [~, gnss_name, ~] = fileparts(gnss_path);
    tag = regexp(imu_name, '(X\d+)', 'match', 'once');
    if isempty(tag)
        tag = regexp(gnss_name, '(X\d+)', 'match', 'once');
    end

    % Always reference the common STATE_X001.txt trajectory for Tasks 6 and 7
    stateName = 'STATE_X001.txt';
    cand = fullfile(fileparts(mfilename('fullpath')), '..', stateName);
    haveTruth = isfile(cand);

    % Load GNSS data and derive NED trajectory
    Tgnss = readtable(gnss_path);
    t_gnss = Tgnss.Posix_Time - Tgnss.Posix_Time(1);
    pos_ecef = [Tgnss.X_ECEF_m Tgnss.Y_ECEF_m Tgnss.Z_ECEF_m];
    vel_ecef = [Tgnss.VX_ECEF_mps Tgnss.VY_ECEF_mps Tgnss.VZ_ECEF_mps];
    first_idx = find(pos_ecef(:,1) ~= 0, 1, 'first');
    ref_r0 = pos_ecef(first_idx,:)';
    [lat_deg, lon_deg, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
    C = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
    pos_ned_gnss = (C * (pos_ecef' - ref_r0))';
    vel_ned_gnss = (C * vel_ecef')';
    dt_gnss = [diff(t_gnss); mean(diff(t_gnss))];
    acc_ned_gnss = [zeros(1,3); diff(vel_ned_gnss)./dt_gnss(1:end-1)];

    imu_raw = readmatrix(imu_path);
    dt_imu = mean(diff(imu_raw(1:100,2)));
    t_imu = (0:size(imu_raw,1)-1)'*dt_imu + t_gnss(1);
    gnss_pos_interp = interp1(t_gnss, pos_ned_gnss, t_imu, 'linear', 'extrap');
    gnss_vel_interp = interp1(t_gnss, vel_ned_gnss, t_imu, 'linear', 'extrap');
    gnss_acc_interp = interp1(t_gnss, acc_ned_gnss, t_imu, 'linear', 'extrap');

    fused_pos = cell(size(methods));
    fused_vel = cell(size(methods));
    fused_acc = cell(size(methods));

    for m = 1:numel(methods)
        method = methods{m};
        fprintf('Running %s with %s...\n', tag, method);
        Task_1(imu_path, gnss_path, method);
        Task_2(imu_path, gnss_path, method);
        Task_3(imu_path, gnss_path, method);
        Task_4(imu_path, gnss_path, method);
        Task_5(imu_path, gnss_path, method);

        pair_tag = [imu_name '_' gnss_name];
        method_file = fullfile(resultsDir, sprintf('%s_%s_task5_results.mat', pair_tag, method));
        if isfile(method_file)
            data = load(method_file);
            fused_pos{m} = data.x_log(1:3,:)';
            fused_vel{m} = data.vel_log';
            fused_acc{m} = data.accel_from_vel';
        else
            warning('Result file not found: %s', method_file);
            fused_pos{m} = [];
            fused_vel{m} = [];
            fused_acc{m} = [];
        end

        outfile = fullfile(resultsDir, sprintf('%s_task5_results_%s.pdf', tag, method));
        save_pva_grid(t_imu, fused_pos{m}, fused_vel{m}, fused_acc{m}, outfile);

        out_kf = fullfile(resultsDir, sprintf('%s_%s_%s_kf_output.mat', imu_name, gnss_name, method));
        if isfile(method_file)
            save(out_kf, '-struct', 'data');
        end

        if haveTruth
            fprintf('Starting Task 6 for %s + %s ...\n', imu_name, gnss_name);
            try
                Task_6(method_file, imu_path, gnss_path, cand);
            catch ME
                warning('Task 6 failed for %s: %s', method, ME.message);
            end
            fprintf('Starting Task 7 for %s + %s ...\n', imu_name, gnss_name);
            try
                tag_m = sprintf('%s_%s_%s', imu_name, gnss_name, method);
                outDir = fullfile(resultsDir, 'task7', tag_m);
                summary = task7_fused_truth_error_analysis(out_kf, cand, outDir);
                save(fullfile(outDir,'task7_summary.mat'), 'summary');
            catch ME
                warning('Task 7 failed for %s: %s', method, ME.message);
            end
        end
    end

    pos_struct = struct('TRIAD', fused_pos{1}, 'Davenport', fused_pos{2}, 'SVD', fused_pos{3});
    vel_struct = struct('TRIAD', fused_vel{1}, 'Davenport', fused_vel{2}, 'SVD', fused_vel{3});
    acc_struct = struct('TRIAD', fused_acc{1}, 'Davenport', fused_acc{2}, 'SVD', fused_acc{3});
    plot_task5_results_all_methods(t_imu, pos_struct, vel_struct, acc_struct, ...
        gnss_pos_interp, gnss_vel_interp, gnss_acc_interp);

    fig = figure('Visible','off','Units','pixels','Position',[0 0 1200 900]);
    labels = {'North [m]','East [m]','Down [m]'};
    rowTitle = {'Position','Velocity','Acceleration'};
    for j = 1:3
        subplot(3,3,j); hold on; grid on;
        plot(t_gnss, pos_ned_gnss(:,j),'k','DisplayName','GNSS');
        for m=1:numel(methods)
            plot(t_imu, fused_pos{m}(:,j), 'DisplayName',methods{m});
        end
        if j==1; legend('show'); end
        title(labels{j}); ylabel(rowTitle{1});
        subplot(3,3,3+j); hold on; grid on;
        plot(t_gnss, vel_ned_gnss(:,j),'k');
        for m=1:numel(methods)
            plot(t_imu, fused_vel{m}(:,j));
        end
        ylabel(rowTitle{2});
        subplot(3,3,6+j); hold on; grid on;
        plot(t_gnss, acc_ned_gnss(:,j),'k');
        for m=1:numel(methods)
            plot(t_imu, fused_acc{m}(:,j));
        end
        ylabel(rowTitle{3}); xlabel('Time [s]');
    end
    sgtitle('Task 5 Comparison - All Methods');
    set(fig,'PaperPositionMode','auto');
    allfile = fullfile(resultsDir, sprintf('%s_task5_results_all_methods.pdf', tag));
    saveas(fig, allfile);
    close(fig);
end
