%% =========================
%  TASK 5 – FUSED-ONLY PLOTS
%  Requires:
%    t_imu                [N x 1] time (epoch or relative)
%    fusedBODY.pos/vel/acc, fusedECEF.pos/vel/acc, fusedNED.pos/vel/acc  [N x 3]
% ==========================

% --- use a common time origin (IMU) for x-axes
timu = t_imu - t_imu(1);

% --- plot the three frames
plot_fused_grid3x3('Task5_BODY_fused', 'Body', timu, fusedBODY);
plot_fused_grid3x3('Task5_ECEF_fused', 'ECEF', timu, fusedECEF);
plot_fused_grid3x3('Task5_NED_fused',  'NED',  timu, fusedNED);

% --- export PDFs (optional)
outdir = fullfile(pwd,'MATLAB','results');
if ~exist(outdir,'dir'); mkdir(outdir); end
exportgraphics(findobj('Name','Task5_BODY_fused'), fullfile(outdir,'IMU_X002_GNSS_X002_TRIAD_task5_fused_body.pdf'), 'ContentType','vector');
exportgraphics(findobj('Name','Task5_ECEF_fused'), fullfile(outdir,'IMU_X002_GNSS_X002_TRIAD_task5_fused_ecef.pdf'), 'ContentType','vector');
exportgraphics(findobj('Name','Task5_NED_fused'),  fullfile(outdir,'IMU_X002_GNSS_X002_TRIAD_task5_fused_ned.pdf'),  'ContentType','vector');

%% -------- helper (keep this function at the end of the script) --------
function plot_fused_grid3x3(figName, frameName, t, fused)
    set(0,'DefaultFigureVisible','on');
    figure('Name',figName,'WindowState','maximized');
    tl = tiledlayout(3,3,'TileSpacing','compact','Padding','compact'); %#ok<NASGU>
    sgtitle(sprintf('Task 5 – TRIAD – Fused GNSS+IMU only (%s frame)', frameName), 'FontWeight','bold');

    rows = {'Position','Velocity','Acceleration'};
    if strcmpi(frameName,'NED')
        cols = {'North','East','Down'};
    else
        cols = {'X','Y','Z'};
    end
    ylab = {'[m]','[m/s]','[m/s^2]'};

    data = {fused.pos, fused.vel, fused.acc};

    ax = gobjects(9,1); k = 0;
    for r = 1:3
        for c = 1:3
            k = k + 1; ax(k) = nexttile; hold on; grid on;
            plot(t, data{r}(:,c), 'LineWidth', 1.4);
            title(sprintf('%s %s', rows{r}, cols{c}));
            xlabel('Time [s]'); ylabel(ylab{r});
            axis tight;
        end
    end
    linkaxes(ax,'x'); xlim([t(1) t(end)]);
    % put a single legend on the first tile so we don't clutter all
    legend(ax(1), {'Fused GNSS+IMU (TRIAD)'}, 'Location','best');
end

