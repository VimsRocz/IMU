function Task_7(pos_data, truth_pos, method)
%TASK_7 Plot residuals between estimator output and truth.
%   TASK_7(POS_DATA, TRUTH_POS, METHOD) computes residuals and saves a
%   plot using the standard naming helper.
%
%   POS_DATA and TRUTH_POS are Nx3 matrices in the same frame.
%
%   This function is a lightweight counterpart to the Python
%   implementation so that Tasks 1--7 exist in MATLAB as well.

    if nargin < 3
        method = 'TRIAD';
    end
    if isempty(pos_data) || isempty(truth_pos)
        error('Position data and truth must be provided');
    end

    res = pos_data - truth_pos;
    fig = figure;
    plot(res);
    title('Position Residuals');
    xlabel('Sample');
    ylabel('Residual (m)');
    grid on;

    save_plot(fig, 'IMU_X', 'GNSS_X', method, 7);
end
