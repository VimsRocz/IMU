function plot_state_grid(t, pos, vel, acc, frame, varargin)
%PLOT_STATE_GRID Plot N/E/D position, velocity and acceleration over time.
%   PLOT_STATE_GRID(T, POS, VEL, ACC, FRAME) creates three figures showing
%   the components of POS, VEL and ACC in the specified FRAME. Additional
%   name-value pairs:
%       'visible' - 'on' or 'off' (default 'off')
%       'save_dir' - directory to save plots (optional)
%       'run_id'   - identifier used in filenames (optional)
%
%   Usage:
%       plot_state_grid(t, pos, vel, acc, 'NED', 'visible','on', ...
%                       'save_dir','results', 'run_id','IMU_GNSS_TRIAD')
%
%   This helper mirrors ``plot_state_grid`` in ``src/utils/plot_state_grid.py``.

p = inputParser;
addParameter(p,'visible','off',@(s)ischar(s)||isstring(s));
addParameter(p,'save_dir','',@(s)ischar(s)||isstring(s));
addParameter(p,'run_id','',@(s)ischar(s)||isstring(s));
parse(p,varargin{:});
vis = p.Results.visible; outdir = char(p.Results.save_dir); run_id = char(p.Results.run_id);

labels = {'North','East','Down'};
data = {pos, vel, acc};
names = {'position','velocity','acceleration'};
for k = 1:3
    h = figure('Visible',vis); hold on; grid on;
    plot(t, data{k}(:,1));
    plot(t, data{k}(:,2));
    plot(t, data{k}(:,3));
    legend(labels, 'Location','best'); legend boxoff;
    xlabel('Time [s]');
    ylabel(sprintf('%s (%s)', names{k}, frame));
    title(sprintf('%s vs time (%s)', names{k}, frame));
    if ~isempty(outdir) && ~isempty(run_id)
        if ~exist(outdir,'dir'), mkdir(outdir); end
        saveas(h, fullfile(outdir, sprintf('%s_task4_%s_%s.png', run_id, frame, names{k})));
        saveas(h, fullfile(outdir, sprintf('%s_task4_%s_%s.pdf', run_id, frame, names{k})));
        close(h);
    end
end
end
