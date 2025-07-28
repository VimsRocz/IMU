function save_plot(fig, imu_name, gnss_name, method, task)
%SAVE_PLOT Save figure as PDF and PNG in the results directory.
%   SAVE_PLOT(FIG, IMU_NAME, GNSS_NAME, METHOD, TASK) writes FIG to the
%   repository 'results' folder using the naming convention:
%   IMU_NAME_GNSS_NAME_METHOD_taskTASK_results.pdf and .png.
%   This mirrors the Python pipeline output structure.
%
%   Example:
%       save_plot(fig, 'IMU_X002', 'GNSS_X002', 'TRIAD', 5)
%   saves results/IMU_X002_GNSS_X002_TRIAD_task5_results.pdf.

    results_dir = get_results_dir();
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    base = sprintf('%s_%s_%s_task%d_results', imu_name, gnss_name, method, task);
    pdf_path = fullfile(results_dir, [base '.pdf']);
    png_path = fullfile(results_dir, [base '.png']);

    set(fig, 'PaperPosition', [0 0 8 6]);
    print(fig, pdf_path, '-dpdf', '-bestfit');
    print(fig, png_path, '-dpng', '-r300');
    fprintf('Saved plot to %s and %s\n', pdf_path, png_path);
end
