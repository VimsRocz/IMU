function Task_6(task5_matfile, truth_file, output_tag)
%TASK_6 Overlay fused and truth trajectories in three frames.
%   TASK_6(TASK5_MATFILE, TRUTH_FILE, OUTPUT_TAG) loads the fused
%   navigation results from Task 5 and the provided ground truth file
%   then overlays the trajectories in the NED, ECEF and body frames.
%   Each figure is displayed on screen and saved as PDF and PNG in the
%   ``results/`` directory using OUTPUT_TAG as the filename prefix.
%
%   Example:
%       Task_6('results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat', ...
%              'STATE_X001.txt', 'IMU_X002_GNSS_X002_TRIAD');
%
%   See also TASK6_FULL_OVERLAY.PY

fprintf('Task 6: Overlaying fused and truth trajectories...\n');

% ------------------------------------------------------------------
% Ensure results directory
% ------------------------------------------------------------------
if ~exist('results', 'dir')
    mkdir('results');
end

% ------------------------------------------------------------------
% Load fused estimator results
% ------------------------------------------------------------------
fprintf('Loading fused results from %s\n', task5_matfile);
S = load(task5_matfile);
req_fields = {'pos_est_ned', 'pos_est_ecef', 'C_b_n'};
for i = 1:numel(req_fields)
    if ~isfield(S, req_fields{i})
        error('%s missing from fused results!', req_fields{i});
    end
end
pos_est_ned  = S.pos_est_ned;
pos_est_ecef = S.pos_est_ecef;
C_b_n        = S.C_b_n;

% ------------------------------------------------------------------
% Load ground truth data
% ------------------------------------------------------------------
fprintf('Loading ground truth from %s\n', truth_file);
if endsWith(lower(truth_file), '.txt')
    data = readmatrix(truth_file);
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
end

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
frames = {'NED','ECEF','Body'};
for f = 1:numel(frames)
    fr = frames{f};
    figure('Name', sprintf('Task 6 Overlay %s', fr)); hold on; grid on;
    switch fr
        case 'NED'
            plot(pos_est_ned(:,1), pos_est_ned(:,2), 'b-', 'LineWidth', 1.5);
            plot(pos_truth_ned(:,1), pos_truth_ned(:,2), 'r--', 'LineWidth', 1.5);
            xlabel('North [m]'); ylabel('East [m]');
            title('Task 6 Overlay: NED Frame');
        case 'ECEF'
            plot(pos_est_ecef(:,1), pos_est_ecef(:,2), 'b-', 'LineWidth', 1.5);
            plot(pos_truth_ecef(:,1), pos_truth_ecef(:,2), 'r--', 'LineWidth', 1.5);
            xlabel('X [m]'); ylabel('Y [m]');
            title('Task 6 Overlay: ECEF Frame');
        case 'Body'
            plot(pos_est_body(:,1), pos_est_body(:,2), 'b-', 'LineWidth', 1.5);
            plot(pos_truth_body(:,1), pos_truth_body(:,2), 'r--', 'LineWidth', 1.5);
            xlabel('X [m]'); ylabel('Y [m]');
            title('Task 6 Overlay: Body Frame');
    end
    legend('Fused','Truth','Location','best');
    drawnow;
    pdf = fullfile('results', sprintf('%s_task6_overlay_state_%s.pdf', output_tag, fr));
    png = strrep(pdf, '.pdf', '.png');
    saveas(gcf, pdf);
    saveas(gcf, png);
    fprintf('Saved overlay plot for %s frame to PDF.\n', fr);
end

fprintf('Task 6 complete: All overlay plots (NED, ECEF, Body) generated and saved.\n');
end
