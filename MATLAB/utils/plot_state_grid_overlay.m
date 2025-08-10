function plot_state_grid_overlay(t_ref, fused, truth, frame_label, varargin)
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
addParameter(p,'Method',''); % e.g., 'TRIAD', used in legend label
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

% resample fused onto reference timeline
[f_t, f_pos] = ensure_unique_increasing('plot fused t', fused.t, fused.pos);
[~,   f_vel] = ensure_unique_increasing('plot fused t', fused.t, fused.vel);
[~,   f_acc] = ensure_unique_increasing('plot fused t', fused.t, fused.acc);

f_pos = interp1(f_t, f_pos, t_ref, 'linear', 'extrap');
f_vel = interp1(f_t, f_vel, t_ref, 'linear', 'extrap');
f_acc = interp1(f_t, f_acc, t_ref, 'linear', 'extrap');

T = figure('Name',[opt.Title,' ',frame_label],'Visible',opt.Visible);
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');

for r = 1:3
    for c = 1:3
        nexttile;
        switch r
            case 1
                y_f = f_pos(:,c); y_t = truth.pos(:,c);
                ylab = sprintf('%s %s [m]', rows{r}, axes_labels{c});
            case 2
                y_f = f_vel(:,c); y_t = truth.vel(:,c);
                ylab = sprintf('%s %s [m/s]', rows{r}, axes_labels{c});
            case 3
                y_f = f_acc(:,c); y_t = truth.acc(:,c);
                ylab = sprintf('%s %s [m/s^2]', rows{r}, axes_labels{c});
        end
        plot(t_ref, y_t, 'm-', 'LineWidth',1.1, 'DisplayName','Truth'); hold on;
        if isempty(opt.Method)
            fused_name = 'Fused GNSS+IMU';
        else
            fused_name = sprintf('Fused GNSS+IMU (%s)', opt.Method);
        end
        plot(t_ref, y_f, 'b-', 'LineWidth',1.1, 'DisplayName', fused_name);
        grid on; axis tight; xlabel('Time [s]'); ylabel(ylab);
        legend('Location','best');
        title(sprintf('%s %s (%s)', rows{r}, axes_labels{c}, frame_label));
    end
end
end
