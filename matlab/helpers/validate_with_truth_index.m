function validate_with_truth_index(matFile, stateFile)
%VALIDATE_WITH_TRUTH_INDEX Compare KF output with reference states by index.
%   VALIDATE_WITH_TRUTH_INDEX(MATFILE, STATEFILE) loads the filter results
%   from MATFILE and the reference trajectory from STATEFILE. The function
%   computes position, velocity and attitude errors by sample index (no time
%   interpolation) and prints summary metrics.

S = load(matFile);
truth = load(stateFile);

n = min(size(S.pos_ned,1), size(truth,1));
pos_err = S.pos_ned(1:n,:) - truth(1:n,3:5);
vel_err = S.vel_ned(1:n,:) - truth(1:n,6:8);

% -- Attitude error -------------------------------------------------------
q_est = [];
if isfield(S, 'attitude_q')
    q_est = S.attitude_q;
elseif isfield(S, 'quat_log')
    q_est = S.quat_log;
elseif isfield(S, 'quat')
    q_est = S.quat;
end
if isempty(q_est) && isfield(S, 'euler_log')
    eul = S.euler_log;
    if size(eul,1) ~= 3 && size(eul,2) == 3
        eul = eul';
    end
    n_eul = size(eul,2);
    q_est = zeros(n_eul,4);
    for i = 1:n_eul
        R = euler_to_rot(eul(:,i));
        q_est(i,:) = rot_to_quaternion(R)';
    end
end

if ~isempty(q_est)
    if size(q_est,1) == 4 && size(q_est,2) ~= 4
        q_est = q_est';
    end
    q_true = truth(1:n,9:12);
    n_att = min(size(q_est,1), size(q_true,1));
    q_est = q_est(1:n_att,:);
    q_true = q_true(1:n_att,:);
    inv_true = [q_true(:,1), -q_true(:,2:4)];
    w0 = q_est(:,1); x0 = q_est(:,2); y0 = q_est(:,3); z0 = q_est(:,4);
    w1 = inv_true(:,1); x1 = inv_true(:,2); y1 = inv_true(:,3); z1 = inv_true(:,4);
    q_err = [
        w0.*w1 - x0.*x1 - y0.*y1 - z0.*z1,
        w0.*x1 + x0.*w1 + y0.*z1 - z0.*y1,
        w0.*y1 - x0.*z1 + y0.*w1 + z0.*x1,
        w0.*z1 + x0.*y1 - y0.*x1 + z0.*w1
    ];
    angle_deg = rad2deg(2*atan2(sqrt(sum(q_err(:,2:4).^2,2)), abs(q_err(:,1))));
    final_att = angle_deg(end);
    rmse_att = sqrt(mean(angle_deg.^2));
end

rmse_pos = sqrt(mean(sum(pos_err.^2,2)));
rmse_vel = sqrt(mean(sum(vel_err.^2,2)));
final_pos = norm(pos_err(end,:));
final_vel = norm(vel_err(end,:));

fprintf('Final position error: %.3f m\n', final_pos);
fprintf('Final velocity error: %.3f m/s\n', final_vel);
fprintf('RMSE position error: %.3f m\n', rmse_pos);
fprintf('RMSE velocity error: %.3f m/s\n', rmse_vel);
if exist('final_att','var')
    fprintf('Final attitude error: %.4f deg\n', final_att);
    fprintf('RMSE attitude error: %.4f deg\n', rmse_att);
end

end

