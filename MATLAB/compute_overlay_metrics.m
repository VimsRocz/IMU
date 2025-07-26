function [metrics, residuals] = compute_overlay_metrics(t, pos_est, vel_est, pos_truth, vel_truth)
%COMPUTE_OVERLAY_METRICS  Return RMSE and final error between fused and truth.
%   [METRICS, RESIDUALS] = COMPUTE_OVERLAY_METRICS(T, POS_EST, VEL_EST,
%   POS_TRUTH, VEL_TRUTH) computes residual position, velocity and
%   acceleration given the fused estimate and ground truth arrays. ``t`` is
%   the common time vector. The returned METRICS struct contains RMSE and
%   final error magnitudes.  RESIDUALS.pos, RESIDUALS.vel and
%   RESIDUALS.acc hold the per-sample differences.

if nargin < 5
    error('Not enough input arguments.');
end

res_pos = pos_est - pos_truth;
res_vel = vel_est - vel_truth;
acc_est = [zeros(1,3); diff(vel_est)./diff(t)];
acc_truth = [zeros(1,3); diff(vel_truth)./diff(t)];
res_acc = acc_est - acc_truth;

metrics.rmse_pos = sqrt(mean(vecnorm(res_pos,2,2).^2));
metrics.final_pos = norm(res_pos(end,:));
metrics.rmse_vel = sqrt(mean(vecnorm(res_vel,2,2).^2));
metrics.final_vel = norm(res_vel(end,:));
metrics.rmse_acc = sqrt(mean(vecnorm(res_acc,2,2).^2));
metrics.final_acc = norm(res_acc(end,:));

residuals.pos = res_pos;
residuals.vel = res_vel;
residuals.acc = res_acc;
end
