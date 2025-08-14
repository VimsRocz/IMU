function task5_plot_fusion_results(fused_data, raw_data, time_vec, blowup_times)
%TASK5_PLOT_FUSION_RESULTS Plot fused vs raw IMU signals with blow-up lines.
%   TASK5_PLOT_FUSION_RESULTS(FUSED_DATA, RAW_DATA, TIME_VEC, BLOWUP_TIMES)
%   creates three figures (NED, ECEF and Body frames).  Each figure is a 3x3
%   grid where rows correspond to position, velocity and acceleration and
%   columns correspond to the three axes in that frame.  Fused estimates are
%   drawn with a blue solid line and raw IMU integration results with a red
%   dashed line.  Vertical dashed black lines mark times of blow-up events.
%   Missing data are rendered as flat zero lines with a "(missing)" legend
%   entry.  Figures are saved as interactive ``.fig`` files under
%   ``MATLAB/results`` and closed after saving.

if nargin < 4 || isempty(blowup_times)
    blowup_times = [];
end

out_dir = fullfile('MATLAB','results');
if ~exist(out_dir,'dir'); mkdir(out_dir); end

frames = {'ned','ecef','body'};
frame_names = {'NED','ECEF','Body'};
quantities = {'pos','vel','acc'};

for k = 1:numel(frames)
    frame = frames{k};
    frame_name = frame_names{k};
    row_labels = { ...
        ['Position [m] ' frame_name], ...
        ['Velocity [m/s] ' frame_name], ...
        ['Acceleration [m/s^2] ' frame_name]};

    try
        fig = figure('Position',[100 100 1800 1200],'Color','w');
        try
            suptitle(sprintf('Task 5: Fused vs Raw in %s Frame | IMU_X002_GNSS_X002_TRIAD | Blow-ups=%d', ...
                frame_name, numel(blowup_times)));
        catch
            sgtitle(sprintf('Task 5: Fused vs Raw in %s Frame | IMU_X002_GNSS_X002_TRIAD | Blow-ups=%d', ...
                frame_name, numel(blowup_times)));
        end

        for r = 1:3
            qty = quantities{r};
            for c = 1:3
                ax = subplot(3,3,(r-1)*3 + c); hold(ax,'on'); grid(ax,'on');
                set(ax,'FontSize',12);

                [raw_col, raw_missing] = get_column(raw_data, frame, qty, c, numel(time_vec));
                [fus_col, fus_missing] = get_column(fused_data, frame, qty, c, numel(time_vec));

                if raw_missing
                    plot(ax, time_vec, raw_col, 'r--', 'DisplayName','IMU raw (missing)');
                else
                    plot(ax, time_vec, raw_col, 'r--', 'DisplayName','IMU raw');
                end

                if fus_missing
                    plot(ax, time_vec, fus_col, 'b-', 'DisplayName','Fused (missing)');
                else
                    plot(ax, time_vec, fus_col, 'b-', 'DisplayName','Fused');
                end

                if ~isempty(blowup_times)
                    xl = xline(ax, blowup_times, 'k--', 'Blow-up');
                    if numel(xl) > 1
                        set(xl(2:end),'HandleVisibility','off');
                    end
                end

                axis(ax,'tight');
                if c == 1
                    ylabel(ax, row_labels{r});
                end
                if r == 3
                    xlabel(ax,'Time [s]');
                end
                legend(ax,'Location','northwest');
            end
        end

        filename = sprintf('IMU_X002_GNSS_X002_TRIAD_task5_%s.fig', frame);
        fullpath = fullfile(out_dir, filename);
        savefig(fig, fullpath);
        fprintf('[SAVE] %s saved as interactive .fig\n', filename);
        close(fig);
    catch ME
        if exist('fig','var') && isvalid(fig)
            close(fig);
        end
        warning('Failed to plot Task 5 results for %s frame: %s', frame_name, ME.message);
    end
end

end

function [col, missing] = get_column(S, frame, qty, axis_idx, N)
%GET_COLUMN Extract a column from nested struct; return flat line if missing.
col = zeros(N,1);
missing = true;
if isstruct(S) && isfield(S,frame)
    F = S.(frame);
    if isstruct(F) && isfield(F,qty)
        data = F.(qty);
        if isnumeric(data) && size(data,2) >= axis_idx
            d = data(:,axis_idx);
            if numel(d) ~= N && numel(d) > 1
                t_orig = linspace(1,N,numel(d));
                d = interp1(t_orig, d(:), 1:N, 'linear','extrap');
            end
            col = d(:);
            missing = false;
        end
    end
end
end

