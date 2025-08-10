function [lag, t_shift] = compute_time_shift(est_series, truth_series, dt)
%COMPUTE_TIME_SHIFT Estimate sample lag between estimate and truth.
%   [LAG, T_SHIFT] = COMPUTE_TIME_SHIFT(EST_SERIES, TRUTH_SERIES, DT) returns
%   the integer sample lag LAG that maximises the cross-correlation between
%   EST_SERIES and TRUTH_SERIES. The corresponding time shift is
%   T_SHIFT = LAG * DT. Positive LAG means the truth data starts later than
%   the estimate.
%
%   Inputs:
%     est_series   - estimator samples (Nx1)
%     truth_series - truth samples    (Mx1)
%     dt           - sample interval of the estimator [s]
%
%   Outputs:
%     lag      - best integer lag in samples (truth relative to estimate)
%     t_shift  - corresponding time shift [s]
%
%   The series are demeaned before computing the cross-correlation to avoid
%   bias. This function mirrors the alignment step in the Python
%   ``assemble_frames`` implementation.
%
%   Example:
%       [lag, t0] = compute_time_shift(est(:,1), truth(:,1), 0.01);
%
%   See also XCORR.

    if nargin < 3
        error('compute_time_shift:NotEnoughInputs', 'Need EST_SERIES, TRUTH_SERIES and DT');
    end
    est_series = est_series(:) - mean(est_series);
    truth_series = truth_series(:) - mean(truth_series);

    corr = xcorr(est_series, truth_series);
    [~, idx] = max(corr);
    lag = idx - numel(est_series);
    t_shift = lag * dt;
end

