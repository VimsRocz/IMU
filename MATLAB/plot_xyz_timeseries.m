function plot_xyz_timeseries(tF, pF, vF, aF, tG, pG, vG, aG, figTitle, outPrefix, axisLabels, method)
%PLOT_XYZ_TIMESERIES Plot fused and optional GNSS time series.
%   plot_xyz_timeseries(tF, pF, vF, aF, tG, pG, vG, aG, TITLE, PREFIX, LABELS, METHOD)
%   creates a 3x3 grid of subplots showing position, velocity and
%   acceleration along the provided axes. Arrays ``pF``, ``vF`` and ``aF``
%   must be 3xN with rows representing the X/Y/Z axes. ``tF`` is the time
%   vector for the fused data. If GNSS arrays ``pG``, ``vG`` or ``aG`` are
%   non-empty, they are overlaid as solid black lines using ``tG`` as the
%   timestamp vector. ``axisLabels`` defaults to {'X','Y','Z'}. ``METHOD``
%   customises the legend label ``Fused (GNSS+IMU, METHOD)``.
%   The figure is saved to PREFIX_fixed.pdf and PREFIX_fixed.png.

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
                plot(tG, gnssSet{r}(c,:), 'k-', 'LineWidth', 1.0);
                leg1 = sprintf('Fused (GNSS+IMU, %s)', method);
                legend({leg1,'Measured GNSS'}, 'Location','best');
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
