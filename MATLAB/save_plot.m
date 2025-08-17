function save_plot(fig, imu_name, gnss_name, method, task, save_pdf, save_png)
%SAVE_PLOT  Save figure in the results directory.
%   SAVE_PLOT(FIG, IMU_NAME, GNSS_NAME, METHOD, TASK, SAVE_PDF, SAVE_PNG)
    %   writes FIG to the repository 'results' folder using the naming
    %   convention IMU_NAME_GNSS_NAME_METHOD_taskTASK_results.*. A MATLAB
    %   ``.fig`` file is always created while optional PDF and PNG copies
    %   can also be generated.
    %
    %   SAVE_PDF and SAVE_PNG are optional and default to false.
%
%   Example:
%       save_plot(fig, 'IMU_X002', 'GNSS_X002', 'TRIAD', 5)
%   saves results/IMU_X002_GNSS_X002_TRIAD_task5_results.pdf and .png.

    if nargin < 6 || isempty(save_pdf); save_pdf = false; end
    if nargin < 7 || isempty(save_png); save_png = false; end

    results_dir = get_results_dir();
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    base = sprintf('%s_%s_%s_task%d_results', imu_name, gnss_name, method, task);
    pdf_path = fullfile(results_dir, [base '.pdf']);
    png_path = fullfile(results_dir, [base '.png']);
    fig_path = fullfile(results_dir, [base '.fig']);

    % ensure consistent figure appearance and always save MATLAB .fig
    set(fig, 'PaperPosition', [0 0 8 6]);
    save_plot_fig(fig, fig_path);

    % optionally save additional formats
    did_pdf = false;
    did_png = false;
    if save_pdf
        print(fig, pdf_path, '-dpdf', '-bestfit');
        did_pdf = true;
    end
    if save_png
        exportgraphics(fig, png_path, 'Resolution', 300);
        did_png = true;
    end

    if did_pdf && did_png
        fprintf('Saved plot to %s, %s and %s\n', fig_path, pdf_path, png_path);
    elseif did_pdf
        fprintf('Saved plot to %s and %s\n', fig_path, pdf_path);
    elseif did_png
        fprintf('Saved plot to %s and %s\n', fig_path, png_path);
    else
        fprintf('Saved plot to %s\n', fig_path);
    end
end
