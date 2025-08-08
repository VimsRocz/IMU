function plot_overlay(frame, method, t_imu, pos_imu, vel_imu, acc_imu, ...
    t_gnss, pos_gnss, vel_gnss, acc_gnss, t_fused, pos_fused, vel_fused, acc_fused, out_dir, varargin)
%PLOT_OVERLAY  Save overlay plot comparing measured IMU, GNSS and fused tracks.
%   PLOT_OVERLAY(FRAME, METHOD, T_IMU, POS_IMU, VEL_IMU, ACC_IMU, T_GNSS,
%   POS_GNSS, VEL_GNSS, ACC_GNSS, T_FUSED, POS_FUSED, VEL_FUSED, ACC_FUSED,
%   OUT_DIR) creates a 4x1 subplot figure showing the norms of position,
%   velocity and acceleration as well as the XY trajectory. The figure is
%   saved as METHOD_FRAME_overlay.pdf in OUT_DIR.
%
%   Optional name-value pairs:
%     't_truth'   - time vector for ground truth
%     'pos_truth' - ground truth positions
%     'vel_truth' - ground truth velocities
%     'acc_truth' - ground truth accelerations
%     'suffix'    - custom file suffix
%
%   When ground truth data is provided the default suffix becomes
%   '_overlay_truth.pdf'.

p = inputParser;
addParameter(p, 't_truth', []);
addParameter(p, 'pos_truth', []);
addParameter(p, 'vel_truth', []);
addParameter(p, 'acc_truth', []);
addParameter(p, 'suffix', '');
addParameter(p, 'filename', '');
addParameter(p, 'visible', 'off');
parse(p, varargin{:});
Ttruth = p.Results.t_truth;
ptruth = p.Results.pos_truth;
vtruth = p.Results.vel_truth;
atruth = p.Results.acc_truth;
suffix = p.Results.suffix;
custom_name = p.Results.filename;
visible_flag = p.Results.visible;

if isempty(suffix) && isempty(custom_name)
    if ~isempty(Ttruth)
        suffix = '_overlay_state.pdf';
    else
        suffix = '_overlay.pdf';
    end
end

h = figure('Visible', visible_flag);
tiledlayout(3,3,'Padding','compact','TileSpacing','compact');

labels = {frame+' X', frame+' Y', frame+' Z'};
ylabels = {'Position [m]', 'Velocity [m/s]', 'Acceleration [m/s^2]'};
data_gnss = {pos_gnss, vel_gnss, acc_gnss};
data_imu  = {pos_imu,  vel_imu,  acc_imu};
data_fused = {pos_fused, vel_fused, acc_fused};
data_truth = {ptruth, vtruth, atruth};

for r = 1:3
    for c = 1:3
        nexttile; hold on;
        plot(t_gnss, data_gnss{r}(:,c), 'k-', 'DisplayName', 'Measured GNSS');
        plot(t_imu, data_imu{r}(:,c), 'c--', 'DisplayName', 'Derived IMU');
        if ~isempty(data_truth{r})
            plot(Ttruth, data_truth{r}(:,c), 'm-', 'DisplayName', 'Truth');
        end
        plot(t_fused, data_fused{r}(:,c), 'g:', 'DisplayName', ['Fused ' method]);
        grid on;
        if r==1
            title(labels{c});
        end
        if c==1
            ylabel(ylabels{r});
        end
        xlabel('Time [s]');
        if r==1 && c==1
            legend('show');
        end
    end
end

sgtitle([method ' - ' frame ' Frame (Fused vs. Truth)']);
set(h,'PaperPositionMode','auto');
if ~isempty(custom_name)
    [~,~,ext] = fileparts(custom_name);
    if isempty(ext)
        pdf_file = fullfile(out_dir, [custom_name '.pdf']);
        png_file = fullfile(out_dir, [custom_name '.png']);
    else
        pdf_file = fullfile(out_dir, custom_name);
        png_file = strrep(pdf_file, '.pdf', '.png');
    end
else
    pdf_file = fullfile(out_dir, [method '_' frame suffix]);
    png_file = strrep(pdf_file, '.pdf', '.png');
end
print(h, pdf_file, '-dpdf', '-bestfit');
print(h, png_file, '-dpng');
close(h);
fprintf('Saved overlay figure to %s\n', pdf_file);
end
