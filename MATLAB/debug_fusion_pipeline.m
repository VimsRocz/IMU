function debug_fusion_pipeline(fused_file, truth_file, imu_file, out_dir)
%DEBUG_FUSION_PIPELINE  Basic INS/GNSS fusion checks
%   DEBUG_FUSION_PIPELINE(FUSED_FILE, TRUTH_FILE, IMU_FILE, OUT_DIR) loads the
%   fused estimator output and reference trajectory and plots timestamp
%   alignment and position error.  The MATLAB implementation mirrors the
%   Python helper ``debug_fusion_pipeline.py`` but currently only performs
%   rudimentary checks.  All figures are saved in OUT_DIR.
%
%   FUSED_FILE should contain variables ``time_s`` and ``pos_ecef_m``.
%   TRUTH_FILE must be a CSV or TXT file with ``time_s`` and ECEF
%   coordinates.  IMU_FILE is optional and not used yet.
%
%   This is a placeholder for future MATLAB parity.

if nargin < 4 || isempty(out_dir)
    out_dir = 'output_matlab';
end
if ~exist(out_dir, 'dir'); mkdir(out_dir); end

load(fused_file, 'time_s', 'pos_ecef_m');
truth = readmatrix(truth_file);

figure('Visible','off');
plot(time_s, 'DisplayName','Fused time'); hold on; grid on;
plot(truth(:,1), 'DisplayName','Truth time');
legend('Location','best');
title('Timestamps: Fused vs Truth');
print(fullfile(out_dir,'debug_timestamps_matlab.pdf'),'-dpdf');
close;

interp_pos = interp1(truth(:,1), truth(:,2:4), time_s, 'linear', 'extrap');
err = vecnorm(pos_ecef_m - interp_pos, 2, 2);
figure('Visible','off');
plot(time_s, err); grid on;
xlabel('Time [s]'); ylabel('Position error [m]');
title('Position Error Over Time');
print(fullfile(out_dir,'debug_error_norm_matlab.pdf'),'-dpdf');
close;

final_err = norm(err(end));
rmse = sqrt(mean(err.^2));
fprintf('Final position error: %.3f m\n', final_err);
fprintf('Position RMSE: %.3f m\n', rmse);
end
