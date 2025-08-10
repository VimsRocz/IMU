function plot_xyz_timeseries(tF, pF, vF, aF, tG, pG, vG, aG, figTitle, outPrefix, axisLabels, method)
%PLOT_XYZ_TIMESERIES  Plot fused and GNSS timeseries in a 3x3 grid.
%   plot_xyz_timeseries(TF, PF, VF, AF, TG, PG, VG, AG, TITLE, PREFIX, LABELS, METHOD)
%   mirrors the Python helper used for Task 5 plots. ``PF``, ``VF`` and ``AF``
%   are ``3xN`` arrays containing position, velocity and acceleration from the
%   Kalman filter. GNSS measurements ``PG``, ``VG`` and ``AG`` are optional but
%   when provided are drawn as black dashed lines for comparison. ``METHOD`` is
%   inserted into the legend to clearly identify the fused data source.
%   ``axisLabels`` defaults to ``{'X','Y','Z'}``.  Figures are saved to
%   ``PREFIX_fixed.pdf`` and ``PREFIX_fixed.png``.

    if nargin < 11 || isempty(axisLabels)
        axisLabels = {'X','Y','Z'};
    end
    if nargin < 12
        method = '';
    end

    % ----- 1. Normalize time -------------------------------------------------
    tF = tF - tF(1);
    if ~isempty(tG)
        tG = tG - tF(1); % Align zero with fused start
    end

    % ----- 2. Symmetric axis limits -----------------------------------------
    limPos = max([abs(pF), abs(pG)], [], 'all');
    limVel = max([abs(vF), abs(vG)], [], 'all');
    limAcc = max([abs(aF), abs(aG)], [], 'all');
    yPos = [-limPos limPos];
    yVel = [-limVel limVel];
    yAcc = [-limAcc limAcc];

    % ----- 3. Layout ---------------------------------------------------------
    fig = figure('Name', figTitle, 'Units','pixels', 'Position',[80 80 1100 800]);
    tl  = tiledlayout(3,3,'TileSpacing','compact','Padding','compact'); %#ok<NASGU>

    rowNames  = {'Position','Velocity','Acceleration'};
    units     = {'[m]','[m/s]','[m/s^2]'};
    fusedSet  = {pF, vF, aF};
    gnssSet   = {pG, vG, aG};
    ylims     = {yPos, yVel, yAcc};

    for r = 1:3
        for c = 1:3
            nexttile
            % Fused data ------------------------------------------------------
            plot(tF, fusedSet{r}(c,:), 'b-', 'LineWidth', 1.0); hold on
            % GNSS overlay ---------------------------------------------------
            if ~isempty(gnssSet{r})
                plot(tG, gnssSet{r}(c,:), 'k:', 'LineWidth', 1.0);
                if ~isempty(method)
                    legend({sprintf('Fused (GNSS+IMU, %s)', method), 'Measured GNSS'}, 'Location','best');
                else
                    legend({'Fused (GNSS+IMU)','Measured GNSS'}, 'Location','best');
                end
            end
            grid on; ylim(ylims{r});
            xlabel('Time [s]'); ylabel(units{r});
            title(sprintf('%s %s', rowNames{r}, axisLabels{c}));
        end
    end
    sgtitle(figTitle,'FontWeight','bold');

    % ----- 4. Save -----------------------------------------------------------
    saveas(fig, [outPrefix, '_fixed.png']);
    saveas(fig, [outPrefix, '_fixed.pdf']);
end
