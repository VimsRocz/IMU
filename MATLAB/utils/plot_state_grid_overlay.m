function hfig = plot_state_grid_overlay(t_ref, fused, truth, frame_label, varargin)
%PLOT_STATE_GRID_OVERLAY  3x3 grid comparing fused and truth states.
%   PLOT_STATE_GRID_OVERLAY(T_REF, FUSED, TRUTH, FRAME_LABEL) plots a
%   3-by-3 grid where rows correspond to position, velocity and
%   acceleration and columns are the axes of the chosen FRAME_LABEL
%   ('NED','ECEF','Body').  FUSED and TRUTH are structs with fields
%   ``t``, ``pos``, ``vel`` and ``acc``.  Optional name-value pairs:
%       'Title'   - figure title prefix
%       'Visible' - 'on' or 'off' (default 'off')
%
%   This helper mirrors ``plot_state_grid_overlay`` in
%   ``src/utils/plot_state_grid_overlay.py``.

p = inputParser;
addParameter(p,'Title','Fused vs Truth');
addParameter(p,'Visible','off');
addParameter(p,'MaxPlotPoints',20000); % downsample for interactive .fig
parse(p,varargin{:});
opt = p.Results;

% axis labels depending on frame
if strcmpi(frame_label,'NED')
    axes_labels = {'N','E','D'};
elseif strcmpi(frame_label,'ECEF')
    axes_labels = {'X','Y','Z'};
else
    axes_labels = {'X','Y','Z'};
end
rows = {'Position','Velocity','Acceleration'};

% resample fused onto reference timeline (robust to duplicates)
try
    [f_t, f_pos] = ensure_unique_increasing('plot fused t', fused.t, fused.pos);
    [~,   f_vel] = ensure_unique_increasing('plot fused t', fused.t, fused.vel);
    [~,   f_acc] = ensure_unique_increasing('plot fused t', fused.t, fused.acc);
catch
    f_t = fused.t; f_pos = fused.pos; f_vel = fused.vel; f_acc = fused.acc;
end

f_pos = interp1(f_t, f_pos, t_ref, 'linear', 'extrap');
f_vel = interp1(f_t, f_vel, t_ref, 'linear', 'extrap');
f_acc = interp1(f_t, f_acc, t_ref, 'linear', 'extrap');

% Optional downsampling for interactive figure size and loadability (fused)
idx_f = 1:numel(t_ref);
if ~isempty(opt.MaxPlotPoints) && isfinite(opt.MaxPlotPoints) && numel(t_ref) > opt.MaxPlotPoints
    idx_f = round(linspace(1, numel(t_ref), opt.MaxPlotPoints));
end
t_plot = t_ref(idx_f);
f_pos = f_pos(idx_f,:);
f_vel = f_vel(idx_f,:);
f_acc = f_acc(idx_f,:);

% Truth timeline: use provided truth.t if available; else assume aligned to t_ref
if isfield(truth,'t') && ~isempty(truth.t)
    t_truth = truth.t(:);
else
    t_truth = t_ref(:);
end

% Optional downsampling for truth independent of fused
idx_t = 1:numel(t_truth);
if ~isempty(opt.MaxPlotPoints) && isfinite(opt.MaxPlotPoints) && numel(t_truth) > opt.MaxPlotPoints
    idx_t = round(linspace(1, numel(t_truth), opt.MaxPlotPoints));
end
t_truth_plot = t_truth(idx_t);
pos_truth_plot = []; vel_truth_plot = []; acc_truth_plot = [];
if isfield(truth,'pos') && ~isempty(truth.pos)
    pos_truth_plot = truth.pos(idx_t,:);
end
if isfield(truth,'vel') && ~isempty(truth.vel)
    vel_truth_plot = truth.vel(idx_t,:);
end
if isfield(truth,'acc') && ~isempty(truth.acc)
    acc_truth_plot = truth.acc(idx_t,:);
end

T = figure('Name',[opt.Title,' ',frame_label],'Visible',opt.Visible, 'Renderer','painters');
set(T,'Units','centimeters','Position',[2 2 18 9]); % FIX: page width
set(T,'PaperPositionMode','auto');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');

for r = 1:3
    for c = 1:3
        nexttile;
        switch r
            case 1
                y_f = f_pos(:,c);
                y_t = [];
                if ~isempty(pos_truth_plot) && size(pos_truth_plot,2) >= c
                    y_t = pos_truth_plot(:,c);
                end
                ylab = sprintf('%s %s [m]', rows{r}, axes_labels{c});
            case 2
                y_f = f_vel(:,c);
                y_t = [];
                if ~isempty(vel_truth_plot) && size(vel_truth_plot,2) >= c
                    y_t = vel_truth_plot(:,c);
                end
                ylab = sprintf('%s %s [m/s]', rows{r}, axes_labels{c});
            case 3
                y_f = f_acc(:,c);
                y_t = [];
                if ~isempty(acc_truth_plot) && size(acc_truth_plot,2) >= c
                    y_t = acc_truth_plot(:,c);
                end
                ylab = sprintf('%s %s [m/s^2]', rows{r}, axes_labels{c});
        end
        hold on;
        if ~isempty(y_t)
            plot(t_truth_plot, y_t, 'LineWidth',1.0);
        end
        plot(t_plot, y_f, 'LineWidth',1.0);
        grid on; xlabel('Time [s]'); ylabel(ylab);
        if r==1 && c==1
            if ~isempty(y_t)
                legend({'Truth','Fused'},'Location','best');
            else
                legend({'Fused'},'Location','best');
            end
            title(sprintf('%s frame', frame_label));
        end
    end
end
% Return figure handle for callers that want to save/rename
hfig = T;
end
