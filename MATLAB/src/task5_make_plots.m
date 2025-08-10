function task5_make_plots(rid, imu_path, gnss_path)
%TASK5_MAKE_PLOTS Render Task 5 fused-only NED/ECEF/Body plots.
% Uses Task 5 outputs to plot Fused GNSS+IMU (TRIAD) only — no GNSS lines.

paths = project_paths();
results_dir = paths.matlab_results;
mat5 = fullfile(results_dir, sprintf('%s_task5_results.mat', rid));
S = load(mat5);

% Time base (seconds from start)
if isfield(S,'t_est'), t = S.t_est(:); else, t = S.time(:); end
t = t - t(1);
dt = median(diff(t)); if ~isfinite(dt) || dt<=0, dt = 1/400; end

% Fused NED from x_log
pos_ned_f = S.x_log(1:3,:).';
vel_ned_f = S.x_log(4:6,:).';
acc_ned_f = [zeros(1,3); diff(vel_ned_f)./dt];

% No GNSS overlays in fused-only plots

% ECEF conversion using Task-5 ref params
ref_lat = S.ref_lat; ref_lon = S.ref_lon; ref_r0 = S.ref_r0(:);
C_e2n = compute_C_ECEF_to_NED(ref_lat, ref_lon);
C_n2e = C_e2n';
pos_ecef_f = (C_n2e*pos_ned_f.' + ref_r0).';
vel_ecef_f = (C_n2e*vel_ned_f.').';
acc_ecef_f = (C_n2e*acc_ned_f.').';

% No GNSS ECEF overlays in fused-only plots

% Body frame using euler_log
eul = S.euler_log; N = numel(t);
pos_body_f = zeros(N,3); vel_body_f = zeros(N,3); acc_body_f = zeros(N,3);
for k=1:N
    C_B_N = euler_to_rot(eul(:,k))'; % NED->Body
    pos_body_f(k,:) = (C_B_N*pos_ned_f(k,:)')';
    vel_body_f(k,:) = (C_B_N*vel_ned_f(k,:)')';
    acc_body_f(k,:) = (C_B_N*acc_ned_f(k,:)')';
end
% No measured IMU overlays in fused-only plots

% ============ Plot NED (3x3 fused-only) ============
figure('Name','Task 5 – TRIAD – NED','Color','w','Visible','on'); tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
labsP = {'Position N','Position E','Position D'}; labsV = {'Velocity N','Velocity E','Velocity D'}; labsA = {'Acceleration N','Acceleration E','Acceleration D'}; colF=[0 0.45 0.85]; lw=1.4;
for i=1:3, nexttile; plot(t,pos_ned_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('Position [m]'); title([labsP{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
for i=1:3, nexttile; plot(t,vel_ned_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title([labsV{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
for i=1:3, nexttile; plot(t,acc_ned_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); title([labsA{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
sgtitle('Task 5 – TRIAD – NED Frame (Fused GNSS+IMU only)'); linkaxes(findall(gcf,'type','axes'),'x');

% ============ Plot ECEF (3x3 fused-only) ============
figure('Name','Task 5 – TRIAD – ECEF','Color','w','Visible','on'); tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
labsP = {'Position X','Position Y','Position Z'}; labsV = {'Velocity X','Velocity Y','Velocity Z'}; labsA = {'Acceleration X','Acceleration Y','Acceleration Z'};
for i=1:3, nexttile; plot(t,pos_ecef_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('Position [m]'); title([labsP{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
for i=1:3, nexttile; plot(t,vel_ecef_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title([labsV{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
for i=1:3, nexttile; plot(t,acc_ecef_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); title([labsA{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
sgtitle('Task 5 – TRIAD – ECEF Frame (Fused GNSS+IMU only)'); linkaxes(findall(gcf,'type','axes'),'x');

% ============ Plot Body (3x3 fused-only) ============
figure('Name','Task 5 – TRIAD – Body','Color','w','Visible','on'); tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
labsP = {'Position bX','Position bY','Position bZ'}; labsV = {'Velocity bX','Velocity bY','Velocity bZ'}; labsA = {'Acceleration bX','Acceleration bY','Acceleration bZ'};
for i=1:3, nexttile; plot(t,pos_body_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('m'); title([labsP{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
for i=1:3, nexttile; plot(t,vel_body_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('m/s'); title([labsV{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
for i=1:3, nexttile; plot(t,acc_body_f(:,i),'LineWidth',lw,'Color',colF); grid on; xlabel('Time [s]'); ylabel('m/s^2'); title([labsA{i} ' (Fused)']); legend('Fused GNSS+IMU (TRIAD)','Location','best'); legend boxoff; axis tight; end
sgtitle('Task 5 – TRIAD – Body Frame (Fused GNSS+IMU only)'); linkaxes(findall(gcf,'type','axes'),'x');

% Save derived for reuse in Task 6
try
    save(fullfile(results_dir, sprintf('%s_task5_derived.mat', rid)), ...
         't','pos_ned_f','vel_ned_f','acc_ned_f', ...
         'pos_ecef_f','vel_ecef_f','acc_ecef_f', ...
         'pos_body_f','vel_body_f','acc_body_f','-v7');
catch ME
    warning('Task 5 derived save failed: %s', ME.message);
end

% Helpers
function R = euler_to_rot(eul)
    % eul = [roll; pitch; yaw]
    cr = cos(eul(1)); sr = sin(eul(1));
    cp = cos(eul(2)); sp = sin(eul(2));
    cy = cos(eul(3)); sy = sin(eul(3));
    R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
         -sp,   cp*sr,           cp*cr          ];
end

end
