function scale_factor = estimate_scale_factor(acc_body, static_start, static_end, expected_gravity)
%ESTIMATE_SCALE_FACTOR Compute accelerometer scale factor from static data.
%   SCALE_FACTOR = ESTIMATE_SCALE_FACTOR(ACC_BODY, START, END, EXPECTED_GRAVITY)
%   returns the ratio between EXPECTED_GRAVITY and the mean norm of the
%   accelerometer samples within the static interval START:END. This is
%   useful for simple scale calibration and mirrors the Python helper used
%   in the fusion pipeline.
%
%   ACC_BODY should be in m/s^2 and EXPECTED_GRAVITY in m/s^2.

static_acc = acc_body(static_start:static_end, :);
mean_acc_norm = mean(vecnorm(static_acc, 2, 2));
scale_factor = expected_gravity / mean_acc_norm;
end
