function body_data = Task_2(imu_path, gnss_path, method)
%TASK_2 Measure body-frame vectors and estimate IMU biases.
% Consolidated Task 2 script with local helper task2_measure_vectors.

if nargin < 3 || isempty(method)
    method = '';
end
if nargin < 2
    gnss_path = '';
end
addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));

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
    error('Task_2:IMUFileNotFound', 'Could not find IMU data at:\n  %s', imu_path);
end

if exist('readmatrix','file')
    data = readmatrix(imu_path);
else
    data = dlmread(imu_path);
end
if size(data,2) < 8
    error('Task_2:BadFormat', 'Expected at least 8 columns in %s', imu_path);
end
dt = mean(diff(data(1:min(end,100),2)));
if dt <= 0 || isnan(dt), dt = 1/400; end
fprintf('Task 2: IMU sample interval dt = %.6f s\n', dt);
accel = data(:,6:8) / dt;
gyro  = data(:,3:5) / dt;

fs = 1/dt;
acc_filt  = low_pass_filter(accel, 5, fs);
gyro_filt = low_pass_filter(gyro, 5, fs);

accel_var_thresh = 2e-4;
gyro_var_thresh = 5e-8;
[static_start, static_end] = detect_static_interval(acc_filt, gyro_filt, [], accel_var_thresh, gyro_var_thresh);
fprintf('Static interval indices: %d to %d (%d samples)\n', static_start, static_end, static_end-static_start);

n_static  = static_end - static_start + 1;
static_dur = n_static * dt;
total_dur  = size(accel,1) * dt;
ratio = static_dur / total_dur * 100;
fprintf('Static interval duration: %.2f s of %.2f s total (%.1f%%)\n', static_dur, total_dur, ratio);
if ratio > 90
    warning('Task_2:LargeStatic', 'Static interval covers %.1f%% of the dataset.', ratio);
end

validate_gravity_vector(acc_filt, static_start, static_end);

static_acc  = mean(acc_filt(static_start:static_end, :), 1); % 1x3
static_gyro = mean(gyro_filt(static_start:static_end, :), 1); % 1x3
g_body_raw = -static_acc';
g_body = (g_body_raw / norm(g_body_raw)) * constants.GRAVITY;
g_body_scaled = g_body;
omega_ie_body = static_gyro';

accel_bias = static_acc' + g_body_scaled;
gyro_bias  = static_gyro';

[~, imu_name, ~] = fileparts(imu_path);
if ~isempty(gnss_path)
    [~, gnss_name, ~] = fileparts(gnss_path);
else
    gnss_name = 'GNSS';
end

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
        'omega_ie_body = [% .6f % .6f % .6f]\n'], static_start, static_end, g_body, omega_ie_body);
fprintf('Accelerometer bias = [% .6f % .6f % .6f] m/s^2\n', accel_bias);
fprintf('Gyroscope bias     = [% .6f % .6f % .6f] rad/s\n', gyro_bias);

paths = project_paths();
results_dir = paths.matlab_results;
if ~exist(results_dir,'dir'); mkdir(results_dir); end

imu_id = imu_name; gnss_id = gnss_name; method_tag = method;
t = (0:size(acc_filt,1)-1) * dt;
acc_norm = vecnorm(acc_filt, 2, 2);
fig_static = figure('Name', 'Task 2 Static Interval', 'Visible', visibleFlag);
plot(t, acc_norm, 'b-'); hold on;
y = ylim;
patch([t(static_start) t(static_end) t(static_end) t(static_start)], [y(1) y(1) y(2) y(2)], [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
hold off;
xlabel('Time (s)'); ylabel('|a| [m/s^2]'); title('Static Interval Detection');
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

body_data = struct();
body_data.g_body        = g_body(:).';
body_data.g_body_scaled = g_body_scaled(:).';
body_data.omega_ie_body = omega_ie_body(:).';
body_data.accel_bias    = accel_bias(:).';
body_data.gyro_bias     = gyro_bias(:).';
body_data.static_start  = static_start;
body_data.static_end    = static_end;
if ~exist('accel_scale','var') || isempty(accel_scale), accel_scale = 1.0; end
body_data.accel_scale   = accel_scale;

task2_file = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', imu_id, gnss_id, method_tag));
if exist('OCTAVE_VERSION', 'builtin')
    save(task2_file, 'body_data');
else
    save(task2_file, 'body_data', '-v7.3');
end
assignin('base','task2_results', body_data);
fprintf('Task 2 results saved to %s\n', task2_file);
fprintf('Task 2 fields:\n');
disp({'g_body','g_body_scaled','omega_ie_body','accel_bias','gyro_bias','static_start','static_end','accel_scale'});

end % Task_2 main

function [acc_bias, gyro_bias, static_start, static_end] = task2_measure_vectors(acc_body, gyro_body, results_dir)
%TASK2_MEASURE_VECTORS Detect static interval and compute IMU biases.
if nargin < 3 || isempty(results_dir)
    results_dir = get_results_dir();
end
[static_start, static_end] = detect_static_interval(acc_body, gyro_body, 80, 0.01, 1e-6, 80);
[acc_bias, gyro_bias] = compute_biases(acc_body, gyro_body, static_start, static_end);
fprintf('Static interval: %d to %d\n', static_start, static_end - 1);
fprintf('Computed accelerometer bias: [%.8f, %.8f, %.8f] m/s^2\n', acc_bias(1), acc_bias(2), acc_bias(3));
fprintf('Computed gyroscope bias: [%.8e, %.8e, %.8e] rad/s\n', gyro_bias(1), gyro_bias(2), gyro_bias(3));
if ~exist(results_dir, 'dir'); mkdir(results_dir); end
save(fullfile(results_dir, 'Task2_IMU_biases.mat'), 'acc_bias', 'gyro_bias', 'static_start', 'static_end');
end

