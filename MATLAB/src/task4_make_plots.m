function task4_make_plots(rid, imu_path, gnss_path)
%TASK4_MAKE_PLOTS Generate Task 4 plots with Python-like styling.
%   TASK4_MAKE_PLOTS(RID, IMU_PATH, GNSS_PATH) loads the Task 4 aligned
%   arrays for run id RID from MATLAB/results/<RID>_task4_results.mat and
%   produces three figures (NED, ECEF, Body optional) matching the
%   specification shared for Python/PDF parity.
%
%   Required variables in the Task 4 MAT file:
%     t_g                - 1xN GNSS time vector (s) starting at zero
%     imu_pos_g, imu_vel_g, imu_acc_g - Nx3 IMU-derived (TRIAD) on t_g
%     gnss_pos_ned, gnss_vel_ned, gnss_acc_ned - Nx3 at GNSS rate (trimmed)
%
%   ECEF plots recompute the rotation using GNSS and convert NED->ECEF.
%   Body acceleration is reconstructed from IMU raw + Task-2 biases.

paths = project_paths();
results_dir = paths.matlab_results;
mat4 = fullfile(results_dir, sprintf('%s_task4_results.mat', rid));
if ~isfile(mat4)
    error('Task 4 MAT not found: %s', mat4);
end
S = load(mat4);
fields_needed = {'t_g','imu_pos_g','imu_vel_g','imu_acc_g','gnss_pos_ned','gnss_vel_ned','gnss_acc_ned'};
for f = fields_needed
    if ~isfield(S, f{1})
        error('Task 4 variable missing: %s in %s', f{1}, mat4);
    end
end

% Time axes (seconds from start)
tG = S.t_g(:); tG = tG - tG(1);
tI = tG; % IMU-derived series are aligned to GNSS grid

% Style
set(groot,'defaultAxesFontSize',11);
lw1 = 1.2; lw2 = 1.6;
colIMU = [0 0.45 0.85];

%% NED figure
figure('Name','Task 4 – TRIAD – NED','Color','w');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
labsP = {'Position N','Position E','Position D'};
labsV = {'Velocity N','Velocity E','Velocity D'};
labsA = {'Acceleration N','Acceleration E','Acceleration D'};

for i=1:3
    nexttile;
    plot(tG, S.gnss_pos_ned(:,i), 'k-', 'LineWidth', lw1); hold on;
    plot(tI, S.imu_pos_g(:,i),    ':', 'LineWidth', lw2, 'Color', colIMU);
    grid on; xlabel('Time [s]'); ylabel('Position [m]'); title(labsP{i});
    legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
end
for i=1:3
    nexttile;
    plot(tG, S.gnss_vel_ned(:,i), 'k-', 'LineWidth', lw1); hold on;
    plot(tI, S.imu_vel_g(:,i),    ':', 'LineWidth', lw2, 'Color', colIMU);
    grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title(labsV{i});
    legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
end
for i=1:3
    nexttile;
    plot(tG, S.gnss_acc_ned(:,i), '--', 'Color',[0.4 0.4 0.4], 'LineWidth', lw1); hold on;
    plot(tI, S.imu_acc_g(:,i),     '-',  'LineWidth', lw2, 'Color', colIMU);
    grid on; xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); title(labsA{i});
    legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
end
sgtitle('Task 4 – TRIAD – NED Frame (IMU-derived vs. Measured GNSS)');
linkaxes(findall(gcf,'type','axes'),'x');

%% ECEF figure (reconstruct rotation using GNSS CSV)
Tg = readtable(gnss_path);
pos_ecef = [Tg.X_ECEF_m, Tg.Y_ECEF_m, Tg.Z_ECEF_m];
first_idx = find(pos_ecef(:,1)~=0, 1, 'first');
r0 = pos_ecef(first_idx, :)';
[latd,lond,~] = ecef_to_geodetic(r0(1), r0(2), r0(3));
C_e2n = compute_C_ECEF_to_NED(deg2rad(latd), deg2rad(lond));
C_n2e = C_e2n';

p_ecef_gnss = (C_n2e*S.gnss_pos_ned' + r0)';
v_ecef_gnss = (C_n2e*S.gnss_vel_ned')';
p_ecef_imu  = (C_n2e*S.imu_pos_g')';
v_ecef_imu  = (C_n2e*S.imu_vel_g')';
a_ecef_imu  = (C_n2e*S.imu_acc_g')';

% Compute GNSS ECEF acceleration from velocity for plotting and reuse
dtG = [0; diff(tG)];
a_ecef_gnss = zeros(size(v_ecef_gnss));
mask = dtG > 0;
a_ecef_gnss(mask,:) = diff(v_ecef_gnss,1,1) ./ dtG(mask);
p_ecef_imu  = (C_n2e*S.imu_pos_g')';
v_ecef_imu  = (C_n2e*S.imu_vel_g')';

figure('Name','Task 4 – TRIAD – ECEF','Color','w');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
labsP = {'Position X','Position Y','Position Z'};
labsV = {'Velocity X','Velocity Y','Velocity Z'};
labsA = {'Acceleration X','Acceleration Y','Acceleration Z'};

for i=1:3
    nexttile;
    plot(tG, p_ecef_gnss(:,i), 'k-', 'LineWidth', lw1); hold on;
    plot(tI, p_ecef_imu(:,i),  ':', 'LineWidth', lw2, 'Color', colIMU);
    grid on; xlabel('Time [s]'); ylabel('Position [m]'); title(labsP{i});
    legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
end
for i=1:3
    nexttile;
    plot(tG, v_ecef_gnss(:,i), 'k-', 'LineWidth', lw1); hold on;
    plot(tI, v_ecef_imu(:,i),  ':', 'LineWidth', lw2, 'Color', colIMU);
    grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title(labsV{i});
    legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
end
for i=1:3
    nexttile;
    plot(tG, a_ecef_gnss(:,i), '--', 'Color',[0.4 0.4 0.4], 'LineWidth', lw1); hold on;
    plot(tI, a_ecef_imu(:,i),   '-',  'LineWidth', lw2, 'Color', colIMU);
    grid on; xlabel('Time [s]'); ylabel('Acceleration [m/s^2]'); title(labsA{i});
    legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
end
sgtitle('Task 4 – TRIAD – ECEF Frame (IMU-derived vs. Measured GNSS)');
linkaxes(findall(gcf,'type','axes'),'x');

%% Body figure (GNSS & IMU pos/vel derived; IMU accel measured)
try
    % Load Task-3 attitude (TRIAD) to rotate NED->Body per time if available
    t3file = fullfile(results_dir, sprintf('%s_task3_results.mat', rid));
    C_series = [];
    if isfile(t3file)
        S3 = load(t3file);
        if isfield(S3,'task3_results') && isfield(S3.task3_results,'Rbn') && isfield(S3.task3_results.Rbn,'TRIAD')
            C_series = S3.task3_results.Rbn.TRIAD; % expect 3x3xN_imu
        end
    end
    % Build time index mapping from IMU attitude series to GNSS grid length
    N = numel(tG);
    if ~isempty(C_series) && ndims(C_series) == 3
        nAtt = size(C_series,3);
        idx = max(1, min(nAtt, round(linspace(1, nAtt, N))));
    else
        % Fallback: use a single rotation (identity body=NED) if attitude missing
        idx = ones(1,N);
        C_series = repmat(eye(3), [1 1 N]);
    end

    % Rotate NED -> Body for GNSS- and IMU-derived pos/vel/acc
    gnss_pos_body = zeros(N,3); gnss_vel_body = zeros(N,3); gnss_acc_body = zeros(N,3);
    imu_pos_body  = zeros(N,3); imu_vel_body  = zeros(N,3);
    for k = 1:N
        C_B_Nk = C_series(:,:,idx(k));
        C_N_Bk = C_B_Nk';
        gnss_pos_body(k,:) = (C_N_Bk * S.gnss_pos_ned(k,:)')';
        gnss_vel_body(k,:) = (C_N_Bk * S.gnss_vel_ned(k,:)')';
        gnss_acc_body(k,:) = (C_N_Bk * S.gnss_acc_ned(k,:)')';
        imu_pos_body(k,:)  = (C_N_Bk * S.imu_pos_g(k,:)')';
        imu_vel_body(k,:)  = (C_N_Bk * S.imu_vel_g(k,:)')';
    end

    % Reconstruct IMU body acceleration (measured, bias-corrected)
    D = readmatrix(imu_path);
    dt = median(diff(D(1:min(1000,end),2))); if ~isfinite(dt) || dt<=0, dt = 1/400; end
    acc_body = D(:,6:8) / dt;
    tok = regexp(rid, '(IMU_\w+)_([A-Z_0-9]+)_([A-Za-z]+)$', 'tokens','once');
    acc_bias = [0 0 0];
    if ~isempty(tok)
        t2 = fullfile(results_dir, sprintf('Task2_body_%s_%s_%s.mat', tok{1}, tok{2}, tok{3}));
        if isfile(t2)
            S2 = load(t2);
            if isfield(S2,'body_data') && isfield(S2.body_data,'accel_bias')
                acc_bias = S2.body_data.accel_bias;
            end
        end
    end
    acc_body_corr = acc_body - acc_bias;

    figure('Name','Task 4 – TRIAD – Body','Color','w');
    tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
    labsP = {'Position bX','Position bY','Position bZ'};
    labsV = {'Velocity bX','Velocity bY','Velocity bZ'};
    labsA = {'Acceleration bX','Acceleration bY','Acceleration bZ'};
    for i=1:3
        nexttile; % Position
        plot(tG, gnss_pos_body(:,i), 'k-', 'LineWidth', lw1); hold on;
        plot(tG, imu_pos_body(:,i),  ':', 'LineWidth', lw2, 'Color', colIMU);
        grid on; xlabel('Time [s]'); ylabel('m'); title(labsP{i});
        legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
    end
    for i=1:3
        nexttile; % Velocity
        plot(tG, gnss_vel_body(:,i), 'k-', 'LineWidth', lw1); hold on;
        plot(tG, imu_vel_body(:,i),  ':', 'LineWidth', lw2, 'Color', colIMU);
        grid on; xlabel('Time [s]'); ylabel('m/s'); title(labsV{i});
        legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
    end
    for i=1:3
        nexttile; % Acceleration: Derived GNSS vs Measured IMU
        plot(tG, gnss_acc_body(:,i), '--', 'Color',[0.4 0.4 0.4], 'LineWidth', lw1); hold on;
        plot(tI, acc_body_corr(1:numel(tI),i), '-', 'LineWidth', lw2, 'Color', colIMU);
        grid on; xlabel('Time [s]'); ylabel('m/s^2'); title(labsA{i});
        legend('GNSS (measured)','IMU (derived)','Location','best'); legend boxoff; axis tight;
    end
    sgtitle('Task 4 – TRIAD – Body Frame (IMU-derived vs. GNSS-derived; IMU accel measured)');
    linkaxes(findall(gcf,'type','axes'),'x');
    % Save derived Body/ECEF variables for reuse
    try
        save(fullfile(results_dir, sprintf('%s_task4_derived.mat', rid)), ...
            'p_ecef_gnss','v_ecef_gnss','a_ecef_gnss','p_ecef_imu','v_ecef_imu','a_ecef_imu', ...
            'gnss_pos_body','gnss_vel_body','gnss_acc_body','imu_pos_body','imu_vel_body','acc_body_corr', '-v7');
    catch ME2
        warning('Failed to save Task 4 derived data: %s', ME2.message);
    end
catch ME
    warning('Body frame plot skipped: %s', ME.message);
end

%% Auto-save PNGs
try
    % Save current open figures in order: NED, ECEF, Body
    figs = findall(0,'Type','figure');
    % Reverse to approximate creation order
    for f = flipud(figs(:)')
        nm = get(f,'Name');
        if contains(nm,'NED')
            out = sprintf('%s_task4_comparison_NED.png', rid);
        elseif contains(nm,'ECEF')
            out = sprintf('%s_task4_comparison_ECEF.png', rid);
        elseif contains(nm,'Body')
            out = sprintf('%s_task4_comparison_Body.png', rid);
        else
            continue;
        end
        set(f,'PaperPositionMode','auto');
        print(f, fullfile(results_dir, out), '-dpng', '-r300');
        figure(f); drawnow;
    end
catch ME
    warning('Auto-save of Task 4 plots failed: %s', ME.message);
end

end
