function save_plot(fig, imu_name, gnss_name, method, task, cfg)
%SAVE_PLOT Save figure to the results directory respecting plot config.
%   SAVE_PLOT(FIG, IMU_NAME, GNSS_NAME, METHOD, TASK, CFG) writes FIG to the
%   repository 'results' folder using the naming convention:
%   IMU_NAME_GNSS_NAME_METHOD_taskTASK_results.*.  PDF and PNG files are
%   only created when the corresponding cfg.plots flags are true.  A MATLAB
%   .fig file is always saved for interactive inspection.

    if nargin < 6 || isempty(cfg)
        try
            cfg = evalin('caller','cfg');
        catch
            cfg = struct();
        end
    end

    results_dir = get_results_dir();
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    base = sprintf('%s_%s_%s_task%d_results', imu_name, gnss_name, method, task);

    set(fig, 'PaperPosition', [0 0 8 6]);
    if isfield(cfg,'plots') && isfield(cfg.plots,'save_pdf') && cfg.plots.save_pdf
        pdf_path = fullfile(results_dir, [base '.pdf']);
        print(fig, pdf_path, '-dpdf', '-bestfit');
    end
    if isfield(cfg,'plots') && isfield(cfg.plots,'save_png') && cfg.plots.save_png
        png_path = fullfile(results_dir, [base '.png']);
        exportgraphics(fig, png_path, 'Resolution', 300);
    end
    fig_path = fullfile(results_dir, [base '.fig']);
    savefig(fig, fig_path);
    fprintf('Saved plot to %s\n', fig_path);
end
