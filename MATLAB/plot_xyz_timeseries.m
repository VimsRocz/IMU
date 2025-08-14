function plot_xyz_timeseries(tF, pF, vF, aF, tG, pG, vG, aG, figTitle, outPrefix, axisLabels, method)
%PLOT_XYZ_TIMESERIES  Plot fused and GNSS time series in a 3x3 grid.
%   plot_xyz_timeseries(TF, PF, VF, AF, TG, PG, VG, AG, TITLE, PREFIX, LABELS, METHOD)
%   mirrors the Python helper used for Task 5 plots. ``PF``, ``VF`` and ``AF``
%   are ``3xN`` arrays containing position, velocity and acceleration from the
%   Kalman filter. GNSS measurements ``PG``, ``VG`` and ``AG`` are optional but
%   when provided are drawn as dashed black lines for comparison. ``METHOD`` is
%   inserted into the legend to clearly identify the fused data source.
%   ``axisLabels`` defaults to ``{'X','Y','Z'}``.  Figures are saved to
%   ``PREFIX_fixed.png`` using ``save_png_checked`` which verifies successful
%   creation.

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
    fig = figure('Name', figTitle, 'Units','pixels', 'Position',[100 100 1800 1200]);
    tl  = tiledlayout(3,3,'TileSpacing','compact','Padding','compact'); %#ok<NASGU>

    rowNames  = {'Position','Velocity','Acceleration'};
    units     = {'[m]','[m/s]','[m/s^2]'};
    fusedSet  = {pF, vF, aF};
    gnssSet   = {pG, vG, aG};
    ylims     = {yPos, yVel, yAcc};
    hf0 = []; hg0 = [];
    for r = 1:3
        for c = 1:3
            nexttile;
            % Fused data ------------------------------------------------------
            hf = plot(tF, fusedSet{r}(c,:), 'b-', 'LineWidth', 1.5); hold on
            % GNSS overlay ---------------------------------------------------
            if ~isempty(gnssSet{r})
                hg = plot(tG, gnssSet{r}(c,:), 'k--', 'LineWidth', 1.5);
            else
                hg = gobjects(0);
            end
            if isempty(hf0); hf0 = hf; end
            if isempty(hg0) && ~isempty(hg); hg0 = hg; end
            grid on; ylim(ylims{r});
            xlabel('Time [s]');
            ylabel(sprintf('%s %s', rowNames{r}, units{r}));
            title(sprintf('%s %s', rowNames{r}, axisLabels{c}));
        end
    end
    if isempty(hg0)
        legend(tl, hf0, sprintf('Fused%s', ternary(~isempty(method), [' (' method ')'], '')), ...
            'Orientation','horizontal','Location','northoutside');
    else
        legend(tl, [hf0 hg0], {sprintf('Fused%s', ternary(~isempty(method), [' (' method ')'], '')), 'GNSS'}, ...
            'Orientation','horizontal','Location','northoutside');
    end
    sgtitle(figTitle,'FontWeight','bold');
    set(findall(fig,'-property','FontSize'),'FontSize',12);

    % ----- 4. Save -----------------------------------------------------------
    save_png_checked(fig, [outPrefix, '_fixed.png']);
end
