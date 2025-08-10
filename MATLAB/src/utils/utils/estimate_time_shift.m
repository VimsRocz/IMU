function [dt_hat, corrN, corrE] = estimate_time_shift(n_truth, n_fused, e_truth, e_fused, dt)
%ESTIMATE_TIME_SHIFT Estimate time shift using cross-correlation of N/E velocities.
%   [dt_hat, corrN, corrE] = ESTIMATE_TIME_SHIFT(n_truth, n_fused, e_truth, e_fused, dt)
%   normalises the input series, computes xcorr on North and East components,
%   selects the median lag (in samples) to improve robustness and returns the
%   corresponding time shift in seconds (negative lag aligns fused to truth).

% Ensure column vectors
n_truth = n_truth(:); n_fused = n_fused(:);
e_truth = e_truth(:); e_fused = e_fused(:);

% Demean to stabilise cross-correlation
n_truth = n_truth - mean(n_truth);  n_fused = n_fused - mean(n_fused);
e_truth = e_truth - mean(e_truth);  e_fused = e_fused - mean(e_fused);

[cN, lN] = xcorr(n_fused, n_truth, 'coeff');
[cE, lE] = xcorr(e_fused, e_truth, 'coeff');

[~,iN] = max(cN);  [~,iE] = max(cE);
lagN = lN(iN);     lagE = lE(iE);

lag = round(median([lagN, lagE]));
dt_hat = -lag * dt;

corrN = cN(iN);
corrE = cE(iE);
end

