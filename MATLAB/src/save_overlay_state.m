function save_overlay_state(t, est_pos, est_vel, truth_pos, truth_vel, frame, run_id, method, out_dir)
%SAVE_OVERLAY_STATE Plot fused vs. truth position and velocity.
%   SAVE_OVERLAY_STATE(T, EST_POS, EST_VEL, TRUTH_POS, TRUTH_VEL, FRAME,
%   RUN_ID, METHOD, OUT_DIR) creates a 2x3 subplot figure mirroring the
%   Python ``plot_overlay`` used in Task 6. ``FRAME`` is one of 'NED',
%   'ECEF' or 'Body'.  The figure is saved as
%   ``<run_id>_task6_overlay_state_<frame>.pdf`` and ``.png`` in OUT_DIR.
%
%   This helper ensures parity between MATLAB and Python visualisations.
%
%   Example:
%       save_overlay_state(t, pos, vel, truth_p, truth_v, 'NED', run_id, method, out_dir);

    labels = {'X','Y','Z'};
    if strcmpi(frame, 'NED')
        labels = {'N','E','D'};
    end
    colors = [0.2157 0.4824 0.7216; 0.8941 0.1020 0.1098; 0.3010 0.6863 0.2902];

    f = figure('Visible','off','Position',[100 100 900 450]);
    data_est = {est_pos, est_vel};
    data_truth = {truth_pos, truth_vel};
    ylabels = {'Position [m]','Velocity [m/s]'};

    for row = 1:2
        for col = 1:3
            subplot(2,3,(row-1)*3 + col); hold on; grid on;
            plot(t, data_est{row}(:,col), 'Color', colors(col,:), ...
                'DisplayName', ['Fused ' labels{col}]);
            plot(t, data_truth{row}(:,col), '--', 'Color', colors(col,:), ...
                'DisplayName', ['Truth ' labels{col}]);
            if col == 1
                ylabel(ylabels{row});
            end
            xlabel('Time [s]');
            if row == 1
                title(labels{col});
            end
            if row == 1 && col == 1
                legend('Location','northeastoutside');
            end
            hold off;
        end
    end
    sgtitle(sprintf('%s Task 6 Overlay — %s (%s frame)', run_id, method, frame));
    pdf_file = fullfile(out_dir, sprintf('%s_task6_overlay_state_%s.pdf', run_id, frame));
    png_file = strrep(pdf_file, '.pdf', '.png');
    set(f,'PaperPositionMode','auto');
    print(f, pdf_file, '-dpdf', '-bestfit');
    print(f, png_file, '-dpng');
    close(f);
end

