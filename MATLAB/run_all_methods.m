function run_all_methods(imu_file, gnss_file)
%RUN_ALL_METHODS Process one dataset with TRIAD, Davenport and SVD.
%   RUN_ALL_METHODS(IMU_FILE, GNSS_FILE) executes Tasks 1--5 for the
%   specified IMU/GNSS pair using all three initialisation methods.
%   Per-method Task 5 plots are saved as results/<tag>_task5_results_<method>.pdf
%   where <tag> is the dataset identifier extracted from the filenames
%   (e.g. X002).  An overlay comparing all methods is saved as
%   results/<tag>_task5_results_all_methods.pdf.
%
%   When IMU_FILE or GNSS_FILE are omitted the X002 sample data is used.

if nargin < 1 || isempty(imu_file)
    imu_file = 'IMU_X002.dat';
end
if nargin < 2 || isempty(gnss_file)
    gnss_file = 'GNSS_X002.csv';
end

imu_path  = get_data_file(imu_file);
gnss_path = get_data_file(gnss_file);
[~, imu_name, ~]  = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);

% Extract dataset tag like 'X002'
tag = regexp(imu_name, '(X\d+)', 'match', 'once');
if isempty(tag)
    tag = regexp(gnss_name, '(X\d+)', 'match', 'once');
end

methods = {'TRIAD','Davenport','SVD'};
colors  = {'r','g','b'};
resultsDir = 'results';
if ~exist(resultsDir,'dir'); mkdir(resultsDir); end

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

% Load IMU time for plotting
imu_raw = readmatrix(imu_path);
dt_imu = mean(diff(imu_raw(1:100,2)));
t_imu = (0:size(imu_raw,1)-1)'*dt_imu + t_gnss(1);

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

    % Task_5 saves the full filter logs to a MAT file
    pair_tag = [imu_name '_' gnss_name];
    method_file = fullfile(resultsDir, sprintf('%s_%s_task5_results.mat', pair_tag, method));
    if isfile(method_file)
        data = load(method_file);
        fused_pos{m} = data.x_log(1:3,:)';
        fused_vel{m} = data.vel_log';
        fused_acc{m} = data.accel_from_vel';
        plot_task456_gnss_imu_fused(imu_file, gnss_file, data);
    else
        warning('Result file not found: %s', method_file);
        fused_pos{m} = [];
        fused_vel{m} = [];
        fused_acc{m} = [];
    end

    % Per-method plot
    outfile = fullfile(resultsDir, sprintf('%s_task5_results_%s.pdf', tag, method));
    save_pva_grid(t_imu, fused_pos{m}, fused_vel{m}, fused_acc{m}, outfile);
end

% Overlay plot of all methods
fig = figure('Visible','off','Units','pixels','Position',[0 0 1200 900]);
labels = {'North [m]','East [m]','Down [m]'};
rowTitle = {'Position','Velocity','Acceleration'};
for j = 1:3
    subplot(3,3,j); hold on; grid on;
    plot(t_gnss, pos_ned_gnss(:,j),'k','DisplayName','GNSS');
    for m=1:numel(methods)
        plot(t_imu, fused_pos{m}(:,j), colors{m},'DisplayName',methods{m});
    end
    if j==1; legend('show'); end
    title(labels{j}); ylabel(rowTitle{1});
    subplot(3,3,3+j); hold on; grid on;
    plot(t_gnss, vel_ned_gnss(:,j),'k');
    for m=1:numel(methods)
        plot(t_imu, fused_vel{m}(:,j), colors{m});
    end
    ylabel(rowTitle{2});
    subplot(3,3,6+j); hold on; grid on;
    plot(t_gnss, acc_ned_gnss(:,j),'k');
    for m=1:numel(methods)
        plot(t_imu, fused_acc{m}(:,j), colors{m});
    end
    ylabel(rowTitle{3}); xlabel('Time [s]');
end
sgtitle('Task 5 Comparison - All Methods');
set(fig,'PaperPositionMode','auto');
allfile = fullfile(resultsDir, sprintf('%s_task5_results_all_methods.pdf', tag));
saveas(fig, allfile);
close(fig);
end

function save_pva_grid(t, pos_ned, vel_ned, acc_ned, outfile)
    fig = figure('Visible','off','Units','pixels','Position',[0 0 1200 900]);
    tl = tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
    labels = {'North [m]','East [m]','Down [m]'};
    rowTitle = {'Position','Velocity','Acceleration'};
    data = {pos_ned, vel_ned, acc_ned};
    for row = 1:3
        for col = 1:3
            nexttile((row-1)*3+col);
            plot(t, data{row}(:,col), 'LineWidth',1.1);
            if row == 1, title(labels{col}); end
            if col == 1, ylabel(rowTitle{row}); end
            grid on;
        end
    end
    xlabel(tl, 'Time [s]');
    set(fig,'PaperPositionMode','auto');
    saveas(fig, outfile);
    close(fig);
end
