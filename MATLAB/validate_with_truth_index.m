function validate_with_truth_index(matFile, stateFile)
%VALIDATE_WITH_TRUTH_INDEX Compare KF output with reference states by index.
%   VALIDATE_WITH_TRUTH_INDEX(MATFILE, STATEFILE) loads the filter results
%   from MATFILE and the reference trajectory from STATEFILE. The function
%   computes position and velocity errors by sample index (no time
%   interpolation) and prints summary metrics.

S = load(matFile);
truth = load(stateFile);

n = min(size(S.pos_ned,1), size(truth,1));
pos_err = S.pos_ned(1:n,:) - truth(1:n,3:5);
vel_err = S.vel_ned(1:n,:) - truth(1:n,6:8);

rmse_pos = sqrt(mean(sum(pos_err.^2,2)));
rmse_vel = sqrt(mean(sum(vel_err.^2,2)));
final_pos = norm(pos_err(end,:));
final_vel = norm(vel_err(end,:));

fprintf('Final position error: %.3f m\n', final_pos);
fprintf('Final velocity error: %.3f m/s\n', final_vel);
fprintf('RMSE position error: %.3f m\n', rmse_pos);
fprintf('RMSE velocity error: %.3f m/s\n', rmse_vel);

end

