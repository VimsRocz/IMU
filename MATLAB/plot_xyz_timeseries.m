function plot_xyz_timeseries(t, pos, vel, acc, title_str, file_base, labels)
%PLOT_XYZ_TIMESERIES  Plot position/velocity/acceleration timeseries.
%   PLOT_XYZ_TIMESERIES(T, POS, VEL, ACC, TITLE_STR, FILE_BASE) creates a 3x3
%   subplot figure with rows [position, velocity, acceleration] and columns
%   representing the X/Y/Z axes.  The figure is saved to FILE_BASE.pdf and
%   FILE_BASE.png.  LABELS is an optional cell array of axis labels.

if nargin < 7 || isempty(labels)
    labels = {'X','Y','Z'};
end

fig = figure('Visible','off','Position',[100 100 1200 900]);
for k = 1:3
    subplot(3,3,k);
    plot(t, pos(:,k), 'b-', 'LineWidth', 1.5);
    grid on; ylabel('[m]'); title(sprintf('Position %s', labels{k}));

    subplot(3,3,3+k);
    plot(t, vel(:,k), 'b-', 'LineWidth', 1.5);
    grid on; ylabel('[m/s]'); title(sprintf('Velocity %s', labels{k}));

    subplot(3,3,6+k);
    plot(t, acc(:,k), 'b-', 'LineWidth', 1.5);
    grid on; ylabel('[m/s^2]'); title(sprintf('Acceleration %s', labels{k}));
end
xlabel('Time (s)');
sgtitle(title_str);

pdf_file = [file_base '.pdf'];
png_file = [file_base '.png'];
set(fig,'PaperPositionMode','auto');
print(fig, pdf_file, '-dpdf', '-bestfit');
print(fig, png_file, '-dpng');
close(fig);
fprintf('Saved plot: %s\n', pdf_file);
end
