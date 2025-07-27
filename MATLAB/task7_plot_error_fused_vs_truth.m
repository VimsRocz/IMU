function task7_plot_error_fused_vs_truth(fused_file, truth_file, output_dir)
%TASK7_PLOT_ERROR_FUSED_VS_TRUTH  Plot velocity error between fused result and truth.
%
%   TASK7_PLOT_ERROR_FUSED_VS_TRUTH(FUSED_FILE, TRUTH_FILE, OUTPUT_DIR) loads
%   the fused estimator output in FUSED_FILE and the reference trajectory in
%   TRUTH_FILE. The difference ``FUSED - Truth`` is computed for each velocity
%   component and plotted over time. The figure is saved as both PDF and PNG
%   under OUTPUT_DIR.
%
%   Corresponds to src/task7_plot_error_fused_vs_truth.py
%
%   Usage:
%       task7_plot_error_fused_vs_truth('fused_results.mat', ...
%           'truth_results.mat', get_results_dir());

if nargin < 3 || isempty(output_dir)
    output_dir = 'output_matlab';
end
if ~exist(output_dir, 'dir'); mkdir(output_dir); end

F = load(fused_file);
% Truth file may include a comment header
if endsWith(truth_file, '.txt')
    T = read_state_file(truth_file);
else
    T = load(truth_file);
end

time = F.time_s(:);
if isfield(F, 'vel_ned_ms')
    vel_f = F.vel_ned_ms;
else
    vel_f = [F.vx(:), F.vy(:), F.vz(:)];
end
if isnumeric(T)
    vel_t = T(:,5:7);
else
    if isfield(T, 'vel_ned_ms')
        vel_t = T.vel_ned_ms;
    else
        vel_t = [T.vx(:), T.vy(:), T.vz(:)];
    end
end

err = vel_f - vel_t;

f = figure('Visible','off');
plot(time, err(:,1), 'r', time, err(:,2), 'g', time, err(:,3), 'b');
legend({'Error X','Error Y','Error Z'}, 'Location','best');
xlabel('Time [s]');
ylabel('Velocity Error [m/s]');
title('Task 7.1: FUSED - Truth Velocity Error');
grid on; box on;
set(f,'PaperPositionMode','auto');

pdf = fullfile(output_dir, 'task7_fused_vs_truth_error.pdf');
png = fullfile(output_dir, 'task7_fused_vs_truth_error.png');
print(f, pdf, '-dpdf', '-bestfit');
print(f, png, '-dpng', '-bestfit');
close(f);
end
