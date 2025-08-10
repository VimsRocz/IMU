function task3_quaternion_plot(task3_file, truth_file, tag)
%TASK3_QUATERNION_PLOT Plot Task 3 quaternions and compare to truth.
%   TASK3_QUATERNION_PLOT(TASK3_FILE, TRUTH_FILE, TAG) loads the estimated
%   quaternion time series from TASK3_FILE and the truth quaternions from
%   TRUTH_FILE. Two figures are produced:
%       1. Computed quaternion components over time.
%       2. Truth vs. computed quaternion comparison.
%   Each figure is saved to <repository>/results as
%   <TAG>_task3_quaternions_*.png and .pdf. Figures are also displayed.
%
% Usage:
%   task3_quaternion_plot('Task3_results_IMU_X002_GNSS_X002.mat', ...
%                         'truth_quat.mat', 'IMU_X002_GNSS_X002_TRIAD')
%
% Inputs:
%   task3_file - path to MAT file containing a struct ``task3_results`` with
%                fields:
%                  t    - time vector [s]
%                  quat - Nx4 quaternion array [w x y z]
%   truth_file - path to MAT file providing ``t`` and ``quat`` for truth.
%   tag        - string identifier used in output file names.
%
% The truth quaternion is interpolated onto the computed time base if
% lengths differ.
%
% See also: PROJECT_PATHS

    % Load Task 3 results
    data = load(task3_file);
    if isfield(data, 'task3_results')
        res = data.task3_results;
    else
        error('task3_results variable not found in %s', task3_file);
    end
    if isfield(res, 't')
        t = res.t(:);
    elseif isfield(res, 'time')
        t = res.time(:);
    else
        error('Time vector not found in task3_results');
    end
    if isfield(res, 'quat')
        q_est = res.quat;
    elseif isfield(res, 'q')
        q_est = res.q;
        if isstruct(q_est)
            fn = fieldnames(q_est);
            q_est = q_est.(fn{1});
        end
    else
        error('Quaternion data not found in task3_results');
    end
    if size(q_est,2) ~= 4
        q_est = q_est';
    end

    % Load truth quaternion
    T = load(truth_file);
    if isfield(T, 'quat')
        q_truth = T.quat;
    elseif isfield(T, 'truth_quat')
        q_truth = T.truth_quat;
    else
        error('Truth quaternion variable not found in %s', truth_file);
    end
    if size(q_truth,2) ~= 4
        q_truth = q_truth';
    end
    if isfield(T, 't')
        t_truth = T.t(:);
    else
        t_truth = linspace(t(1), t(end), size(q_truth,1))';
    end

    % Interpolate truth onto computed timeline
    q_truth_i = interp1(t_truth, q_truth, t, 'linear', 'extrap');
    trel = t - t(1);

    % Prepare results directory
    p = project_paths();
    results_dir = fullfile(p.root, 'results');
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end

    labs = {'q_w','q_x','q_y','q_z'};

    % Figure 1: computed quaternion components
    f1 = figure('Name','Task3 Quaternion Components');
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for i = 1:4
        nexttile; hold on; grid on;
        plot(trel, q_est(:,i), 'b-', 'DisplayName','Computed');
        xlabel('Time [s]'); ylabel(labs{i});
        title(labs{i});
        legend('Location','best');
    end
    base1 = sprintf('%s_task3_quaternions_components', tag);
    pdf1 = fullfile(results_dir, [base1 '.pdf']);
    png1 = fullfile(results_dir, [base1 '.png']);
    exportgraphics(f1, pdf1, 'ContentType','vector');
    exportgraphics(f1, png1, 'Resolution', 300);
    fprintf('Saved %s and %s\n', pdf1, png1);

    % Figure 2: computed vs truth quaternions
    f2 = figure('Name','Task3 Quaternion Comparison');
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for i = 1:4
        nexttile; hold on; grid on;
        plot(trel, q_truth_i(:,i), 'k-', 'DisplayName','Truth');
        plot(trel, q_est(:,i), 'b--', 'DisplayName','Computed');
        xlabel('Time [s]'); ylabel(labs{i});
        title(labs{i});
        if i == 1
            legend('Location','best');
        end
    end
    base2 = sprintf('%s_task3_quaternions_comparison', tag);
    pdf2 = fullfile(results_dir, [base2 '.pdf']);
    png2 = fullfile(results_dir, [base2 '.png']);
    exportgraphics(f2, pdf2, 'ContentType','vector');
    exportgraphics(f2, png2, 'Resolution', 300);
    fprintf('Saved %s and %s\n', pdf2, png2);
end
