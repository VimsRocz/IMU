

function Task_6(task5_matfile, truth_file, run_id)
%TASK_6  Overlay fused estimator results with ground truth trajectory.
%   TASK_6(TASK5_MATFILE, TRUTH_FILE, RUN_ID) loads the fused navigation
%   solution saved by Task 5 and the corresponding ``STATE_X`` truth
%   file. All series are interpolated to the estimator time base and
%   overlay plots for the NED, ECEF and body frames are produced. Figures
%   are written to ``<results>/<run_id>/`` within the repository using the
%   naming convention ``<run_id>_task6_overlay_state_<frame>.pdf``.

% ------------------------------------------------------------------
% Ensure results directory
% ------------------------------------------------------------------
results_root = get_results_dir();
out_dir = fullfile(results_root, run_id);
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

% ------------------------------------------------------------------
% Load fused estimator results
% ------------------------------------------------------------------
fprintf('Loading fused results from %s\n', task5_matfile);
S = load(task5_matfile);
if isfield(S,'pos_est_ned'); pos_est_ned = S.pos_est_ned; elseif isfield(S,'pos_fused_ned'); pos_est_ned = S.pos_fused_ned; else; error('pos_est_ned missing'); end
if isfield(S,'vel_est_ned'); vel_est_ned = S.vel_est_ned; elseif isfield(S,'vel_fused_ned'); vel_est_ned = S.vel_fused_ned; else; vel_est_ned = []; end
if isfield(S,'acc_est_ned'); acc_est_ned = S.acc_est_ned; elseif isfield(S,'acc_fused_ned'); acc_est_ned = S.acc_fused_ned; else; acc_est_ned = []; end
if isfield(S,'pos_est_ecef'); pos_est_ecef = S.pos_est_ecef; elseif isfield(S,'pos_fused_ecef'); pos_est_ecef = S.pos_fused_ecef; else; pos_est_ecef = []; end
if isfield(S,'vel_est_ecef'); vel_est_ecef = S.vel_est_ecef; elseif isfield(S,'vel_fused_ecef'); vel_est_ecef = S.vel_fused_ecef; else; vel_est_ecef = []; end
C_b_n = S.C_b_n;
if isfield(S,'time_s'); t_est = S.time_s(:); elseif isfield(S,'time'); t_est = S.time(:); else; t_est = (0:size(pos_est_ned,1)-1).'; end

% ------------------------------------------------------------------
% Load ground truth data
% ------------------------------------------------------------------
fprintf('Loading ground truth from %s\n', truth_file);
if endsWith(lower(truth_file), '.txt')
    data = readmatrix(truth_file);
    t_truth = data(:,2);
    pos_truth_ecef = data(:,3:5);
    if size(data,2) >= 11
        pos_truth_ned = data(:,9:11);
    else
        [lat_deg, lon_deg, ~] = ecef_to_geodetic(pos_truth_ecef(1,1), ...
            pos_truth_ecef(1,2), pos_truth_ecef(1,3));
        C_e_n = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
        ref = pos_truth_ecef(1,:).';
        pos_truth_ned = (C_e_n * (pos_truth_ecef.' - ref)).';
    end
else
    T = load(truth_file);
    if ~isfield(T,'pos_truth_ecef') || ~isfield(T,'pos_truth_ned')
        error('Truth file missing required fields.');
    end
    pos_truth_ecef = T.pos_truth_ecef;
    pos_truth_ned  = T.pos_truth_ned;
    if isfield(T,'time'); t_truth = T.time(:); else; t_truth = (0:size(pos_truth_ecef,1)-1).'; end
end

pos_truth_ecef = interp1(t_truth, pos_truth_ecef, t_est, 'linear', 'extrap');
pos_truth_ned  = interp1(t_truth, pos_truth_ned,  t_est, 'linear', 'extrap');

% ------------------------------------------------------------------
% Convert NED positions to body frame
% ------------------------------------------------------------------
if ndims(C_b_n) == 3
    N = size(pos_est_ned,1);
    pos_est_body  = zeros(N,3);
    pos_truth_body = zeros(size(pos_truth_ned));
    for k = 1:N
        idx = min(k, size(C_b_n,3));
        pos_est_body(k,:)  = (C_b_n(:,:,idx)' * pos_est_ned(k,:)').';
        pos_truth_body(k,:) = (C_b_n(:,:,idx)' * pos_truth_ned(k,:)').';
    end
else
    pos_est_body  = (C_b_n' * pos_est_ned').';
    pos_truth_body = (C_b_n' * pos_truth_ned').';
end
fprintf('Converted fused and truth NED positions to body frame.\n');

% ------------------------------------------------------------------
% Overlay plots for each frame
% ------------------------------------------------------------------
dt_est = median(diff(t_est));
if isempty(vel_est_ned)
    vel_est_ned = [zeros(3,1), diff(pos_est_ned,1,2)/dt_est];
end
if isempty(acc_est_ned)
    acc_est_ned = diff(vel_est_ned,1,2)/dt_est;
end
if isempty(vel_est_ecef) && ~isempty(pos_est_ecef)
    vel_est_ecef = [zeros(3,1), diff(pos_est_ecef,1,2)/dt_est];
end
if ~exist('acc_est_ecef','var') && ~isempty(vel_est_ecef)
    acc_est_ecef = diff(vel_est_ecef,1,2)/dt_est;
end

vel_truth_ned = [zeros(3,1), diff(pos_truth_ned.',1,2)/dt_est];
acc_truth_ned = diff(vel_truth_ned,1,2)/dt_est;
vel_truth_ecef = [zeros(3,1), diff(pos_truth_ecef.',1,2)/dt_est];
acc_truth_ecef = diff(vel_truth_ecef,1,2)/dt_est;

% Convert to body frame
if ndims(C_b_n) == 3
    N = size(pos_est_ned,2);
    pos_est_body = zeros(3,N);
    vel_est_body = zeros(3,N);
    pos_truth_body = zeros(3,N);
    vel_truth_body = zeros(3,N);
    for k = 1:N
        idx = min(k, size(C_b_n,3));
        R = C_b_n(:,:,idx)';
        pos_est_body(:,k) = R * pos_est_ned(:,k);
        vel_est_body(:,k) = R * vel_est_ned(:,k);
        pos_truth_body(:,k) = R * pos_truth_ned(k,:).';
        vel_truth_body(:,k) = R * vel_truth_ned(:,k);
    end
else
    R = C_b_n';
    pos_est_body = R * pos_est_ned;
    vel_est_body = R * vel_est_ned;
    pos_truth_body = (R * pos_truth_ned.').';
    vel_truth_body = R * vel_truth_ned;
end
acc_est_body = diff(vel_est_body,1,2)/dt_est;
acc_truth_body = diff(vel_truth_body,1,2)/dt_est;

frames = {'NED','ECEF','Body'};
for f = 1:numel(frames)
    fr = frames{f};
    switch fr
        case 'NED'
            fig = plot_task6_overlay(t_est, pos_est_ned, vel_est_ned, acc_est_ned, pos_truth_ned.', vel_truth_ned, acc_truth_ned, fr, run_id);
        case 'ECEF'
            fig = plot_task6_overlay(t_est, pos_est_ecef, vel_est_ecef, acc_est_ecef, pos_truth_ecef.', vel_truth_ecef, acc_truth_ecef, fr, run_id);
        otherwise
            fig = plot_task6_overlay(t_est, pos_est_body, vel_est_body, acc_est_body, pos_truth_body, vel_truth_body, acc_truth_body, fr, run_id);
    end
    pdf = fullfile(out_dir, sprintf('%s_task6_overlay_state_%s.pdf', run_id, fr));
    png = strrep(pdf, '.pdf', '.png');
    print(fig, pdf, '-dpdf', '-bestfit');
    print(fig, png, '-dpng', '-r300');
    close(fig);
    fprintf('Saved overlay plot for %s frame.\n', fr);
end

fprintf('Task 6 complete: All overlay plots generated.\n');
end

% -------------------------------------------------------------------------
function fig = plot_task6_overlay(t, pos_est, vel_est, acc_est, pos_truth, vel_truth, acc_truth, frame, tag)
%PLOT_TASK6_OVERLAY  Create 3x1 overlay plot of fused vs truth data.
    if strcmpi(frame,'ECEF')
        labels = {'X','Y','Z'};
    else
        labels = {'N','E','D'};
    end
    colors = lines(3);
    fig = figure('Name', sprintf('Task6 %s', frame));
    tl = tiledlayout(3,1,'TileSpacing','compact');
    data_est = {pos_est, vel_est, acc_est};
    data_truth = {pos_truth, vel_truth, acc_truth};
    ylab = {'Position (m)','Velocity (m/s)','Acceleration (m/s^2)'};
    for r = 1:3
        ax = nexttile; hold(ax,'on'); grid(ax,'on');
        for j = 1:3
            plot(ax, t(1:size(data_est{r},2)), data_est{r}(j,:), 'Color', colors(j,:), 'DisplayName', sprintf('Fused %s', labels{j}));
            plot(ax, t(1:size(data_truth{r},2)), data_truth{r}(j,:), '--', 'Color', colors(j,:), 'DisplayName', sprintf('Truth %s', labels{j}));
        end
        ylabel(ax, ylab{r});
        if r == 1
            title(ax, sprintf('%s Task 6 Overlay — %s frame', tag, frame));
            legend(ax,'Location','best');
        else
            legend(ax,'off');
        end
    end
    xlabel(tl,'Time (s)');
end
