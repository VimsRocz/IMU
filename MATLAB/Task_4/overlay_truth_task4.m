function overlay_truth_task4(kfFile, stateFile, method)
%OVERLAY_TRUTH_TASK4 Overlay ground truth on Task 4 figures.
%   OVERLAY_TRUTH_TASK4(KFFILE, STATEFILE, METHOD) loads the Kalman filter
%   output MAT file and the matching STATE_X*.txt trajectory and saves
%   figures with "_overlay_truth.png" suffix next to KFFILE.

S = load(kfFile);
truth = load(stateFile);
if isfield(S,'ref_lat'); ref_lat = S.ref_lat; else; ref_lat = deg2rad(-32.026554); end
if isfield(S,'ref_lon'); ref_lon = S.ref_lon; else; ref_lon = deg2rad(133.455801); end
if isfield(S,'ref_r0');  ref_r0 = S.ref_r0;  else;  ref_r0 = [-3729051 3935676 -3348394]; end
C = compute_C_ECEF_to_NED(ref_lat, ref_lon);

pos_truth = (C*(truth(:,3:5)'-ref_r0(:)))';
vel_truth = (C*truth(:,6:8)')';
t_truth   = truth(:,2);
acc_truth = [zeros(1,3); diff(vel_truth)./diff(t_truth)];

t_est = S.time_residuals;
if isempty(t_est); t_est = S.time; end
pos_truth_i = interp1(t_truth, pos_truth, t_est);
vel_truth_i = interp1(t_truth, vel_truth, t_est);
acc_truth_i = interp1(t_truth, acc_truth, t_est);

acc_imu = [zeros(1,3); diff(S.vel_ned)./diff(t_est)];

plot_overlay('NED', method, t_est, S.pos_ned, S.vel_ned, acc_imu, ...
             S.gnss_time, S.gnss_pos_ned, S.gnss_vel_ned, S.gnss_accel_ned, ...
             t_est, S.pos_ned, S.vel_ned, acc_imu, fileparts(kfFile), ...
             't_truth', t_est, 'pos_truth', pos_truth_i, 'vel_truth', vel_truth_i, ...
             'acc_truth', acc_truth_i, 'suffix', '_overlay_truth.png');
end
