function summary = task7_fused_truth_error_analysis(est_file, truth_file, out_dir)
%TASK7_FUSED_TRUTH_ERROR_ANALYSIS  Residual analysis of fused state vs. truth.
%   SUMMARY = TASK7_FUSED_TRUTH_ERROR_ANALYSIS(EST_FILE, TRUTH_FILE, OUT_DIR)
%   loads the fused estimator result and ground truth trajectory (MAT or NPZ
%   files). Both trajectories are aligned in the local NED frame before
%   computing residuals. The truth ECEF data is converted using
%   ``compute_C_ECEF_to_NED`` with the estimator reference latitude,
%   longitude and origin. When loading NPZ files the function accepts either
%   ``pos_ecef_m``/``vel_ecef_ms`` or ``pos_ecef``/``vel_ecef`` keys.  When the
%   estimate only contains NED states, it is converted to ECEF and then to
%   the common NED frame. Plots of the residual components and their norms
%   are saved under OUT_DIR and a structure of summary statistics is returned.

if nargin < 3 || isempty(out_dir)
    out_dir = get_results_dir();
end
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
start_time = tic;

% Echo estimator attitude convention for clarity in logs
fprintf('Estimator attitude convention: C_nb (Body->NED), quaternion [w x y z] from att_quat (Nx4)\n');

[t_est, pos_est_ecef, vel_est_ecef, acc_est_ecef, lat, lon, r0] = load_est(est_file);
[t_tru, pos_tru_ecef, vel_tru_ecef, ~] = load_est(truth_file);

% ------------------------------------------------------------------
% The estimator may use absolute UNIX time stamps while the truth data
% typically starts from zero.  To ensure consistent interpolation we
% normalise both time vectors so that ``t=0`` corresponds to the first
% sample.  This mirrors ``load_estimate(..., times=t_truth)`` in the
% Python implementation.
% ------------------------------------------------------------------
t_est = t_est - t_est(1);
t_tru = t_tru - t_tru(1);

% ------------------------------------------------------------------
% Convert both trajectories to a common NED frame using the estimator
% reference parameters.  When the estimator file lacks reference
% information, fall back to the first truth sample.
% ------------------------------------------------------------------
if isnan(lat) || isnan(lon) || any(isnan(r0))
    % Use the first truth sample as the shared origin when the estimator
    % does not provide reference parameters. ``ecef_to_geodetic`` returns
    % degrees so convert to radians for ``compute_C_ECEF_to_NED``.
    r0 = pos_tru_ecef(1,:);
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(r0(1), r0(2), r0(3));
    lat = deg2rad(lat_deg);
    lon = deg2rad(lon_deg);
else
    % Some log files store reference lat/lon in degrees.  Detect this and
    % convert to radians so both MATLAB and Python interpret them identically.
    if abs(lat) > pi
        lat = deg2rad(lat);
    end
    if abs(lon) > pi
        lon = deg2rad(lon);
    end
end
C = compute_C_ECEF_to_NED(lat, lon);
% Rotate both trajectories into the same NED frame using the **shared**
% origin ``r0`` so that residuals are computed consistently.  This
% matches the behaviour of ``validate_with_truth.py`` in Python.
pos_est = (C * (pos_est_ecef' - r0)).';
vel_est = (C*vel_est_ecef.').';
pos_tru = (C * (pos_tru_ecef' - r0)).';
vel_tru = (C*vel_tru_ecef.').';

% Crop truth to estimator span before interpolation to reduce extrapolation effects
mask = (t_tru >= t_est(1)) & (t_tru <= t_est(end));
if any(mask)
    t_tru_c = t_tru(mask); pos_tru_c = pos_tru(mask,:); vel_tru_c = vel_tru(mask,:);
else
    t_tru_c = t_tru; pos_tru_c = pos_tru; vel_tru_c = vel_tru;
end
pos_tru_i = interp1(t_tru_c, pos_tru_c, t_est, 'linear', 'extrap');
vel_tru_i = interp1(t_tru_c, vel_tru_c, t_est, 'linear', 'extrap');

res_pos = pos_est - pos_tru_i;
res_vel = vel_est - vel_tru_i;

mean_pos = mean(res_pos, 1);
std_pos  = std(res_pos, [], 1);
mean_vel = mean(res_vel, 1);
std_vel  = std(res_vel, [], 1);

fprintf('Position residual mean [m]: %s\n', mat2str(mean_pos,3));
fprintf('Position residual std  [m]: %s\n', mat2str(std_pos,3));
fprintf('Velocity residual mean [m/s]: %s\n', mat2str(mean_vel,3));
fprintf('Velocity residual std  [m/s]: %s\n', mat2str(std_vel,3));
plot_residuals(t_est, res_pos, res_vel, out_dir);
plot_norms(t_est, res_pos, res_vel, out_dir);
plot_innovation_chi2(est_file, t_est, out_dir);

% Detect divergence time based on residual norms and/or innovation gates
try
    pos_norm = vecnorm(res_pos,2,2);
    vel_norm = vecnorm(res_vel,2,2);
    % Use early segment (first 10%%) to set adaptive thresholds (mean+6*std)
    n0 = max(10, round(0.1*numel(t_est)));
    thr_pos = mean(pos_norm(1:n0)) + 6*std(pos_norm(1:n0));
    thr_vel = mean(vel_norm(1:n0)) + 6*std(vel_norm(1:n0));
    win_s  = 5; % seconds sustained beyond threshold
    % Find first time where either norm exceeds its threshold for win_s continuously
    dt = median(diff(t_est)); kwin = max(1, round(win_s/dt));
    div_idx = NaN;
    for k = 1:(numel(t_est)-kwin+1)
        if all(pos_norm(k:k+kwin-1) > thr_pos) || all(vel_norm(k:k+kwin-1) > thr_vel)
            div_idx = k; break; end
    end
    if ~isnan(div_idx)
        t_div = t_est(div_idx) - t_est(1);
        frac  = t_div / (t_est(end)-t_est(1));
        fprintf('[Task7] Divergence detected at t=%.1f s (%.1f%% of dataset).\n', t_div, 100*frac);
        % Append to metrics file
        fid = fopen(fullfile(out_dir,'task7_metrics.txt'),'a');
        if fid>0
            fprintf(fid, 'Divergence time: %.1f s (%.1f%% of dataset)\n', t_div, 100*frac);
            fclose(fid);
        end
    else
        fprintf('[Task7] No sustained divergence detected by adaptive thresholds.\n');
    end
catch ME
    warning('Task 7: divergence detection failed: %s', ME.message);
end

% Plot NED position alignment zoom (default last 50s; if 1200-1250s present, use that)
try
    t_rel = t_est - t_est(1);
    if t_rel(end) >= 1250
        t0 = 1200; t1 = 1250;
    else
        t1 = t_rel(end); t0 = max(0, t1 - 50);
    end
    maskz = (t_rel >= t0) & (t_rel <= t1);
    fz = figure('Visible','on','Position',[100 100 900 500]);
    lbl = {'North','East','Down'};
    for j=1:3
        subplot(3,1,j); plot(t_rel(maskz), pos_tru_i(maskz,j), 'k-', t_rel(maskz), pos_est(maskz,j), 'b:','LineWidth',1.2);
        grid on; ylabel(lbl{j}); if j==1, title('Task 7 - NED Position Alignment (Zoom)'); end
    end
    xlabel('Time [s]');
    print(fz, fullfile(out_dir, 'task7_ned_pos_zoom_NED.png'), '-dpng','-r200'); close(fz);
catch ME
    warning('Task 7: failed to save NED zoom plot: %s', ME.message);
end

% -------------------------------------------------------------------------
function qwxyz = dcm_to_quat_batch_local(R)
%DCM_TO_QUAT_BATCH_LOCAL Convert 3x3xN DCMs to Nx4 quaternions [w x y z].
    N = size(R,3); qwxyz = zeros(N,4);
    for k = 1:N
        C = R(:,:,k); tr = trace(C);
        if tr > 0
            S = sqrt(tr+1.0)*2; w = 0.25*S;
            x = (C(3,2)-C(2,3))/S; y = (C(1,3)-C(3,1))/S; z = (C(2,1)-C(1,2))/S;
        else
            [~,i] = max([C(1,1),C(2,2),C(3,3)]);
            switch i
                case 1
                    S = sqrt(1+C(1,1)-C(2,2)-C(3,3))*2; w=(C(3,2)-C(2,3))/S; x=0.25*S; y=(C(1,2)+C(2,1))/S; z=(C(1,3)+C(3,1))/S;
                case 2
                    S = sqrt(1+C(2,2)-C(1,1)-C(3,3))*2; w=(C(1,3)-C(3,1))/S; x=(C(1,2)+C(2,1))/S; y=0.25*S; z=(C(2,3)+C(3,2))/S;
                case 3
                    S = sqrt(1+C(3,3)-C(1,1)-C(2,2))*2; w=(C(2,1)-C(1,2))/S; x=(C(1,3)+C(3,1))/S; y=(C(2,3)+C(3,2))/S; z=0.25*S;
            end
        end
        q = [w x y z]; qwxyz(k,:) = q / norm(q);
    end
end

% ------------------------------------------------------------------
% Attitude quaternion error (if both estimate and truth quaternions exist)
% θ = 2*acos(|q_est·q_true|) in degrees, with hemisphere handling.
% ------------------------------------------------------------------
try
    q_est = [];
    q_tru = [];
    % Load estimated quaternion from NPZ or MAT
    if endsWith(string(est_file), '.npz')
        d = py.numpy.load(string(est_file));
        if isKey(d,'att_quat')
            q_est = double(d{'att_quat'}); % [N x 4], [q0 q1 q2 q3]
        end
    else
        Sest = load(est_file);
        if isfield(Sest, 'att_quat')
            q_est = Sest.att_quat;
        elseif isfield(Sest, 'quat_log')
            q_est = Sest.quat_log; % could be 4xN or N x 4
            if size(q_est,1) == 4 && size(q_est,2) ~= 4
                q_est = q_est';
            end
        elseif isfield(Sest, 'euler_log')
            % Build quaternion history from Euler angles [rad] in columns
            eul = Sest.euler_log; % 3 x N (roll,pitch,yaw)
            Nq = size(eul,2);
            q_est = zeros(Nq,4);
            for kk = 1:Nq
                R = euler_to_rot(eul(:,kk));
                q_est(kk,:) = dcm_to_quat(R);
            end
        end
    end
    % Load truth quaternion from STATE_X file
    if endsWith(string(truth_file), '.txt')
        raw_truth = read_state_file(truth_file);
        if size(raw_truth,2) >= 12
            t_truth_q = raw_truth(:,2);
            % Normalize to seconds if needed (expect ~10 Hz)
            dtm_q = median(diff(t_truth_q));
            if isfinite(dtm_q) && dtm_q > 0.5 && dtm_q < 1.5
                t_truth_q = t_truth_q / 10;
            end
            q_tru_src = raw_truth(:,9:12);
            % SLERP truth quaternions to t_est timeline
            try
                q_slerp = attitude_tools('slerp_series', (t_truth_q - t_truth_q(1)).', q_tru_src.', t_est.');
                q_tru = q_slerp.'; % Nx4
            catch
                % Fallback to component-wise interpolation + renormalize
                q_tru = interp1(t_truth_q - t_truth_q(1), q_tru_src, t_est, 'linear', 'extrap');
                q_tru = q_tru ./ vecnorm(q_tru,2,2);
            end
            % Persist interpolated truth quaternions on estimator timeline
            try
                save(fullfile(out_dir,'task7_truth_quat_interp.mat'), 't_est', 'q_tru');
            catch
            end
        end
    elseif endsWith(string(truth_file), '.npz')
        dtru = py.numpy.load(string(truth_file));
        if isKey(dtru,'att_quat')
            t_truth_q = double(dtru{'time_s'});
            q_tru = double(dtru{'att_quat'});
            q_tru = interp1(t_truth_q - t_truth_q(1), q_tru, t_est, 'linear', 'extrap');
        end
    else
        Stru = load(truth_file);
        if isfield(Stru, 'att_quat')
            q_tru = Stru.att_quat;
        end
    end

    % Align estimate quaternion samples to t_est timeline if needed
    if ~isempty(q_est)
        if size(q_est,1) ~= numel(t_est)
            % Try common time field names from NPZ/MAT
            if endsWith(string(est_file), '.npz')
                d = py.numpy.load(string(est_file));
                if isKey(d,'time_s')
                    t_q = double(d{'time_s'});
                    q_est = interp1(t_q - t_q(1), q_est, t_est, 'linear', 'extrap');
                end
            else
                Sest = load(est_file);
                t_q = [];
                if isfield(Sest, 'time_s'); t_q = Sest.time_s(:); end
                if isempty(t_q) && isfield(Sest, 'time'); t_q = Sest.time(:); end
                if ~isempty(t_q)
                    q_est = interp1(t_q - t_q(1), q_est, t_est, 'linear', 'extrap');
                end
            end
        end
    end

    % Final safety: force length alignment with t_est to avoid size mismatches
    % Robust guard: handle missing truth attitude or constant length-1 cases
    doQuatError = true;
    if isempty(q_tru)
        warning('Task 7: TRUTH quaternion not available; skipping quaternion error.');
        doQuatError = false;
    else
        if size(q_tru,1) == 1 && size(q_est,1) > 1
            q_tru = repmat(q_tru, size(q_est,1), 1);
        end
    end

    if doQuatError && ~isempty(q_est) && ~isempty(q_tru)
        n_final = min([size(q_est,1), size(q_tru,1), numel(t_est)]);
        if n_final == 0
            error('Task7:QuatAlign','Empty quaternion series after interpolation');
        end
        if size(q_est,1) ~= n_final, q_est = q_est(1:n_final,:); end
        if size(q_tru,1) ~= n_final, q_tru = q_tru(1:n_final,:); end
        if numel(t_est)   ~= n_final, t_est = t_est(1:n_final); end

        % Normalize (avoid implicit expansion issues on older MATLAB)
        q_est = bsxfun(@rdivide, q_est, max(eps, vecnorm(q_est,2,2)));
        q_tru = bsxfun(@rdivide, q_tru, max(eps, vecnorm(q_tru,2,2)));

        % Convert truth ECEF-based quaternions into Body->NED to match estimator.
        % Try both interpretations (Body<-ECEF and ECEF<-Body) and choose the
        % one with smaller mean attitude error.
        R_est = attitude_tools('quat_to_dcm_batch', q_est.');   % 3x3xN (B->NED)
        R_tru = attitude_tools('quat_to_dcm_batch', q_tru.');   % 3x3xN (unknown)
        Nq = size(q_est,1);
        q_truth_nb_1 = zeros(Nq,4);
        q_truth_nb_2 = zeros(Nq,4);
        for k = 1:Nq
            R_b2e_or_e2b = R_tru(:,:,k);
            % Candidate 1: assume truth DCM = Body<-ECEF (C_B_E)
            R_nb_1 = C * (R_b2e_or_e2b.');
            % Candidate 2: assume truth DCM = ECEF<-Body (C_E_B)
            R_nb_2 = C * R_b2e_or_e2b;
            q_truth_nb_1(k,:) = dcm_to_quat(R_nb_1);
            q_truth_nb_2(k,:) = dcm_to_quat(R_nb_2);
        end
        % Dot products with explicit bsxfun to avoid size broadcasting issues
        dots1 = abs(sum(bsxfun(@times, q_est, q_truth_nb_1), 2)); dots1 = max(-1,min(1,dots1)); ang1 = 2*acosd(dots1);
        dots2 = abs(sum(bsxfun(@times, q_est, q_truth_nb_2), 2)); dots2 = max(-1,min(1,dots2)); ang2 = 2*acosd(dots2);
        m1 = mean(ang1,'omitnan'); m2 = mean(ang2,'omitnan');
        fprintf('Task 7: Est attitude is Body->NED (C_{nb}).\\n');
        fprintf('Task 7: Truth quat mapping test (deg): meanErr C_e2n*(C_B_E)^T=%.2f, C_e2n*C_E_B=%.2f\\n', m1, m2);
        if m1 <= m2
            q_tru_nb = q_truth_nb_1;
        else
            q_tru_nb = q_truth_nb_2;
        end
        % Compute constant rotation C_fix aligning estimator to truth (SO(3) Procrustes)
        R_est_seq = attitude_tools('quat_to_dcm_batch', q_est.');      % 3x3xN (B->NED)
        R_tru_seq = attitude_tools('quat_to_dcm_batch', q_tru_nb.');    % 3x3xN (B->NED)
        M = zeros(3);
        for kk = 1:size(R_est_seq,3)
            M = M + R_tru_seq(:,:,kk) * R_est_seq(:,:,kk)';
        end
        [U,~,V] = svd(M); C_fix = U*diag([1 1 det(U*V')])*V';
        fprintf('Task 7: Solved constant alignment C_fix (Truth ≈ C_fix * Est):\n');
        disp(C_fix);
        dMain = diag(C_fix); fprintf('Task 7: diag(C_fix) ≈ [%+.0f %+.0f %+.0f], max|off|=%.3f\n', round(dMain(1)), round(dMain(2)), round(dMain(3)), max(abs(C_fix(:) - diag(dMain))));
        try
            save(fullfile(out_dir,'task7_attitude_alignment.mat'), 'C_fix');
        catch
        end
        % Apply C_fix to estimator DCMs, then convert back to quaternions
        R_est_aligned = zeros(3,3,size(R_est_seq,3));
        for kk = 1:size(R_est_seq,3)
            R_est_aligned(:,:,kk) = C_fix * R_est_seq(:,:,kk);
        end
        q_est_aligned = zeros(size(q_est));
        for kk = 1:size(R_est_aligned,3)
            q_est_aligned(kk,:) = dcm_to_quat(R_est_aligned(:,:,kk));
        end
        % Hemisphere alignment: flip estimator to agree with truth
        flipMask = sum(bsxfun(@times, q_tru_nb, q_est_aligned), 2) < 0;
        q_est_aligned(flipMask,:) = -q_est_aligned(flipMask,:);
        % Compute final error with aligned estimator
        dots = abs(sum(bsxfun(@times, q_est_aligned, q_tru_nb), 2)); dots = max(-1,min(1,dots));
        ang_deg = 2 * acosd(dots);
        f = figure('Visible','on','Position',[100 100 700 300]);
        plot(t_est, ang_deg, 'LineWidth', 1.2); grid on;
        xlabel('Time [s]'); ylabel('Attitude Error [deg]');
        title('Task 7 - Attitude Quaternion Error');
        set(f,'PaperPositionMode','auto');
        pdf_q = fullfile(out_dir, 'task7_attitude_quat_error_deg_NED.pdf');
        fig_q = fullfile(out_dir, 'task7_attitude_quat_error_deg_NED.fig');
        png_q = fullfile(out_dir, 'task7_attitude_quat_error_deg_NED.png');
        print(f, pdf_q, '-dpdf', '-bestfit');
        try, savefig(f, fig_q); catch, end
        try, print(f, png_q, '-dpng','-r200'); catch, end
        fprintf('Saved %s and %s\n', pdf_q, fig_q);
        % Optional summary
        fprintf('Quaternion error (deg): mean=%.3f median=%.3f p95=%.3f max=%.3f\n', ...
            mean(ang_deg), median(ang_deg), prctile(ang_deg,95), max(ang_deg));

        % New Task 7 overlays: Position/Velocity/Euler and Quaternion components
        eul_est_deg = quat_to_euler_deg_batch(q_est_aligned);
        eul_tru_deg = quat_to_euler_deg_batch(q_tru_nb);
        % Align lengths for attitude overlays to prevent size mismatches
        nE = min([size(eul_est_deg,1), size(eul_tru_deg,1), numel(t_est)]);
        if nE == 0
            error('Task7:EulerAlign','Empty Euler arrays after conversion');
        end
        if size(eul_est_deg,1) ~= nE, eul_est_deg = eul_est_deg(1:nE,:); end
        if size(eul_tru_deg,1) ~= nE, eul_tru_deg = eul_tru_deg(1:nE,:); end
        t_plot = t_est(1:nE);
        % Match position/velocity arrays to plot time
        pos_est_plot = pos_est(1:min(size(pos_est,1), nE), :);
        pos_tru_plot = pos_tru_i(1:min(size(pos_tru_i,1), nE), :);
        vel_est_plot = vel_est(1:min(size(vel_est,1), nE), :);
        vel_tru_plot = vel_tru_i(1:min(size(vel_tru_i,1), nE), :);
        plot_task7_overlay_pveul(t_plot, pos_est_plot, pos_tru_plot, vel_est_plot, vel_tru_plot, eul_est_deg, eul_tru_deg, out_dir);
        plot_task7_quat_components(t_est, q_est_aligned, q_tru_nb, out_dir);

        % Save attitude error series for downstream metrics/inspection
        try
            att_err_deg = ang_deg; %#ok<NASGU>
            mapping_label = 'Body->NED (aligned with C_{fix})'; %#ok<NASGU>
            save(fullfile(out_dir,'task7_attitude_error.mat'), 't_est', 'att_err_deg', 'mapping_label');
        catch
        end

        % Plot DCM relative angle over time after alignment
        try
            Ndc = size(R_est_aligned,3); th = zeros(Ndc,1);
            for kk=1:Ndc
                Rrel = R_est_aligned(:,:,kk) * R_tru_seq(:,:,kk)';
                c = max(-1,min(1,(trace(Rrel)-1)/2)); th(kk) = rad2deg(acos(c));
            end
            fda = figure('Visible','on','Position',[100 100 700 300]);
            plot(t_est, th, 'LineWidth',1.2); grid on;
            xlabel('Time [s]'); ylabel('Angle [deg]'); title('Task 7 - DCM Relative Angle (Aligned)');
            print(fda, fullfile(out_dir, 'task7_attitude_dcm_relative_angle_NED.png'), '-dpng','-r200'); close(fda);
        catch ME
            warning('Task 7: failed to save DCM relative angle plot: %s', ME.message);
        end

        % ---- ECEF attitude comparison and quaternion overlay ----
        try
            Rn2e = C'; % NED->ECEF
            % Truth Body->ECEF from NED mapping: R_tru_e = C_n2e * R_tru_n
            R_tru_e = zeros(3,3,size(R_tru_seq,3));
            for kk=1:size(R_tru_seq,3); R_tru_e(:,:,kk) = Rn2e * R_tru_seq(:,:,kk); end
            % Estimator Body->ECEF from Body->NED
            R_est_e_raw = zeros(3,3,size(R_est_seq,3));
            for kk=1:size(R_est_seq,3); R_est_e_raw(:,:,kk) = Rn2e * R_est_seq(:,:,kk); end
            % Constant alignment C_fix_e in ECEF
            Ae = reshape(R_tru_e(:,1,:), 3, []); Be = reshape(R_est_e_raw(:,1,:), 3, []);
            [Ue,~,Ve] = svd(Ae*Be','econ'); C_fix_e = Ue*Ve'; if det(C_fix_e) < 0, Ue(:,3) = -Ue(:,3); C_fix_e = Ue*Ve'; end
            R_est_e = zeros(3,3,size(R_est_e_raw,3));
            for kk=1:size(R_est_e_raw,3); R_est_e(:,:,kk) = C_fix_e * R_est_e_raw(:,:,kk); end
            % ECEF attitude error series
            att_err_ecef_deg = zeros(size(t_est));
            for kk=1:numel(t_est)
                Rrel = R_tru_e(:,:,kk) * R_est_e(:,:,kk)';
                c = max(-1,min(1,(trace(Rrel)-1)/2)); att_err_ecef_deg(kk) = acosd(c);
            end
            fprintf('Task 7 (ECEF): mean=%.2f deg, median=%.2f, p95=%.2f\n', mean(att_err_ecef_deg), median(att_err_ecef_deg), prctile(att_err_ecef_deg,95));
            fe = figure('Visible','on','Position',[100 100 900 300]);
            plot(t_est, att_err_ecef_deg, 'LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('Attitude Error [deg]');
            title('Task 7 - Attitude Error (Truth vs KF, Body→ECEF, aligned)');
            print(fe, fullfile(out_dir,'task7_attitude_quat_error_ecef.png'), '-dpng','-r200'); close(fe);
            try
                mapping_label_ecef = 'Body->ECEF (aligned with C_{fix}^e)'; %#ok<NASGU>
                save(fullfile(out_dir,'task7_attitude_error_ecef.mat'), 't_est','att_err_ecef_deg','C_fix_e','mapping_label_ecef');
            catch
            end
            % Quaternion components overlay in ECEF
            q_tru_e = dcm_to_quat_batch_local(R_tru_e);
            q_kf_e  = dcm_to_quat_batch_local(R_est_e);
            fc = figure('Visible','on','Position',[50 50 1200 600]); labs = {'q0','q1','q2','q3'};
            for ii=1:4
                subplot(2,2,ii);
                plot(t_est, q_tru_e(:,ii),'k-','LineWidth',0.8); hold on;
                plot(t_est, q_kf_e(:,ii),'b:','LineWidth',0.8); grid on;
                xlabel('Time [s]'); ylabel(labs{ii}); title(['Quaternion ' labs{ii} ' (ECEF)']);
                if ii==1, legend('Truth','KF (aligned)'); end
            end
            print(fc, fullfile(out_dir,'task7_quaternion_components_overlay_ecef.png'), '-dpng','-r200'); close(fc);
        catch ME
            warning('Task 7: ECEF attitude comparison failed: %s', ME.message);
        end

        % Euler RMSE summary (deg) with wrap handling to [-180,180]
        % Ensure same length before subtraction
        nE = min(size(eul_est_deg,1), size(eul_tru_deg,1));
        de = eul_est_deg(1:nE,:) - eul_tru_deg(1:nE,:);
        de = mod(de + 180, 360) - 180;
        rmse_eul = sqrt(mean(de.^2, 1)); % [roll pitch yaw]
        summary.euler_rmse_deg = rmse_eul;
        fprintf('Euler RMSE [deg]: roll=%.3f pitch=%.3f yaw=%.3f\n', rmse_eul(1), rmse_eul(2), rmse_eul(3));
        % Append to metrics file
        try
            txt_file = fullfile(out_dir,'task7_metrics.txt');
            fid = fopen(txt_file,'a');
            if fid>0
                fprintf(fid, 'Euler RMSE [deg]: roll=%.3f pitch=%.3f yaw=%.3f\n', rmse_eul(1), rmse_eul(2), rmse_eul(3));
                fclose(fid);
            end
        catch
        end
    else
        fprintf('Task 7: Attitude quaternion not available in est and/or truth. Skipping quat error plot.\n');
    end
catch ME
    fprintf('Task 7: Quaternion error computation failed: %s\n', ME.message);
end

summary.rmse_pos = sqrt(mean(vecnorm(res_pos,2,2).^2));
summary.final_pos = norm(res_pos(end,:));
summary.rmse_vel = sqrt(mean(vecnorm(res_vel,2,2).^2));
summary.final_vel = norm(res_vel(end,:));
tbl = {
    'Position [m]',    summary.final_pos, summary.rmse_pos;
    'Velocity [m/s]',  summary.final_vel, summary.rmse_vel;
    };
hdr = sprintf('%-18s %-12s %-10s', 'Metric', 'Final Error', 'RMSE');
table_lines = cell(size(tbl,1),1);
for ii = 1:size(tbl,1)
    table_lines{ii} = sprintf('%-18s %10.3f %10.3f', tbl{ii,1}, tbl{ii,2}, tbl{ii,3});
end
table_str = strjoin([{hdr}; table_lines], '\n');
table_str = sprintf('%s\n', table_str); % ensure trailing newline
fprintf('%s\n', table_str);

txt_file = fullfile(out_dir,'task7_metrics.txt');
fid = fopen(txt_file,'w');
if fid>0
    fprintf(fid,'%s\n', table_str);
    fclose(fid);
end

[~, tag] = fileparts(out_dir);
parts = strsplit(tag,'_');
method = parts{end};
runtime = toc(start_time);
fprintf('[SUMMARY] method=%s rmse_pos=%.3fm final_pos=%.3fm rmse_vel=%.3fm/s final_vel=%.3fm/s runtime=%.2fs\n', ...
    method, summary.rmse_pos, summary.final_pos, summary.rmse_vel, summary.final_vel, runtime);

% Compute masked RMSE using innovation gates (if Task-5 saved logs)
try
    t5 = load(est_file);
    if isfield(t5,'innov_d2_pos') && isfield(t5,'innov_d2_vel')
        n = min([numel(t_est), numel(t5.innov_d2_pos), numel(t5.innov_d2_vel)]);
        % Force column vectors to avoid implicit expansion (Nx1 vs 1xN)
        d2p = t5.innov_d2_pos(1:n); d2v = t5.innov_d2_vel(1:n);
        d2p = d2p(:); d2v = d2v(:);
        gp = 11.345; gv = 11.345;
        if isfield(t5,'gate_chi2_pos'), gp = t5.gate_chi2_pos; end
        if isfield(t5,'gate_chi2_vel'), gv = t5.gate_chi2_vel; end
        valid_gate = (d2p <= gp) & (d2v <= gv);
        tail_window_s = 30; 
        t_n = t_est(1:n); t_n = t_n(:);
        valid_tail = t_n <= (t_n(end) - tail_window_s);
        mask_valid = (valid_gate(:) & valid_tail(:));

        % Safe O(N) masked RMSE (no NxN allocations)
        if ~any(mask_valid)
            warning('Task 7: masked RMSE skipped (no valid samples after gating).');
        else
            pr = res_pos(1:n,:);
            vr = res_vel(1:n,:);
            pr = pr(mask_valid,:);
            vr = vr(mask_valid,:);

        rmse_pos_xyz  = sqrt(mean(pr.^2,1,'omitnan'));
        rmse_pos_all  = sqrt(mean(sum(pr.^2,2),'omitnan'));

        rmse_vel_xyz  = sqrt(mean(vr.^2,1,'omitnan'));
        rmse_vel_all  = sqrt(mean(sum(vr.^2,2),'omitnan'));

            fprintf(['[Task7] Masked RMSE (valid only): ' ...
                     'pos=%.3f m (xyz=[%.3f %.3f %.3f]), ' ...
                     'vel=%.3f m/s (xyz=[%.3f %.3f %.3f])\n'], ...
                    rmse_pos_all, rmse_pos_xyz(1), rmse_pos_xyz(2), rmse_pos_xyz(3), ...
                    rmse_vel_all, rmse_vel_xyz(1), rmse_vel_xyz(2), rmse_vel_xyz(3));

            % Append to metrics file
            fid = fopen(txt_file,'a');
            if fid>0
                fprintf(fid, ['Masked RMSE (valid): pos=%.3f m (xyz=[%.3f %.3f %.3f]), ' ...
                              'vel=%.3f m/s (xyz=[%.3f %.3f %.3f])\n'], ...
                        rmse_pos_all, rmse_pos_xyz(1), rmse_pos_xyz(2), rmse_pos_xyz(3), ...
                        rmse_vel_all, rmse_vel_xyz(1), rmse_vel_xyz(2), rmse_vel_xyz(3));
                fclose(fid);
            end
        end
    end
catch ME
    warning('Task 7: masked RMSE calculation failed: %s', ME.message);
end

end

% -------------------------------------------------------------------------
function [t, pos, vel, acc, lat, lon, r0] = load_est(file)
%LOAD_EST Load estimate containing ECEF or NED position/velocity.
%   The helper accepts NPZ or MAT files produced by either the Python or
%   MATLAB pipelines. When ECEF fields are missing it falls back to NED
%   outputs such as ``pos_ned``, ``fused_pos`` or ``x_log``.
    f = string(file);
    lat = NaN; lon = NaN; r0 = [NaN NaN NaN];
    if endsWith(f,'.npz')
    d = py.numpy.load(f);
    t = double(d{'time_s'});
    if isKey(d, 'pos_ecef_m')
        pos = double(d{'pos_ecef_m'});
    elseif isKey(d, 'pos_ecef')
        pos = double(d{'pos_ecef'});
    else
        error('Task7:BadData', 'NPZ file lacks ECEF position');
    end
    if isKey(d, 'vel_ecef_ms')
        vel = double(d{'vel_ecef_ms'});
    elseif isKey(d, 'vel_ecef')
        vel = double(d{'vel_ecef'});
    else
        error('Task7:BadData', 'NPZ file lacks ECEF velocity');
    end
    if isKey(d,'acc_ecef_ms2')
        acc = double(d{'acc_ecef_ms2'});
    else
        acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
    end
    if isKey(d,'ref_lat_rad'); lat = double(d{'ref_lat_rad'}); elseif isKey(d,'ref_lat'); lat = double(d{'ref_lat'}); end
    if isKey(d,'ref_lon_rad'); lon = double(d{'ref_lon_rad'}); elseif isKey(d,'ref_lon'); lon = double(d{'ref_lon'}); end
    if isKey(d,'ref_r0_m'); r0 = double(d{'ref_r0_m'}); elseif isKey(d,'ref_r0'); r0 = double(d{'ref_r0'}); end
    else
    if endsWith(f,'.txt')
            raw = read_state_file(f);
            % Columns: 1-count, 2-time [s or 0.1s ticks], 3:5-pos ECEF [m], 6:8-vel ECEF [m/s]
            t_raw = raw(:,2);
            dtm = median(diff(t_raw));
            if isfinite(dtm) && dtm > 0.5 && dtm < 1.5
                t = t_raw / 10;  % normalize to seconds (~10 Hz)
            else
                t = t_raw;
            end
            pos = raw(:,3:5);
            vel = raw(:,6:8);
            acc = [zeros(1,3); diff(vel)./diff(t)];
        else
            S = load(f);
            if isfield(S,'time_s');
                t = S.time_s(:);
            elseif isfield(S,'time');
                t = S.time(:);
            elseif isfield(S,'imu_time')
                t = S.imu_time(:);
            else
                % Derive the time vector from any available position field
                if isfield(S,'pos_ecef_m')
                    len = size(S.pos_ecef_m,1);
                elseif isfield(S,'pos_ecef')
                    len = size(S.pos_ecef,1);
                elseif isfield(S,'pos_ned')
                    len = size(S.pos_ned,1);
                else
                    error('Task7:BadData','Estimate lacks position data');
                end
                t = (0:len-1)';
            end
            if isfield(S,'pos_ecef_m')
                pos = S.pos_ecef_m;
                vel = S.vel_ecef_ms;
            elseif isfield(S,'pos_ecef')
                pos = S.pos_ecef;
                vel = S.vel_ecef;
            elseif isfield(S,'pos_ned') && isfield(S,'vel_ned')
                % Derive ECEF using reference parameters
                if isfield(S,'ref_lat'); lat = S.ref_lat; elseif isfield(S,'ref_lat_rad'); lat = S.ref_lat_rad; else; lat = NaN; end
                if isfield(S,'ref_lon'); lon = S.ref_lon; elseif isfield(S,'ref_lon_rad'); lon = S.ref_lon_rad; else; lon = NaN; end
                if isfield(S,'ref_r0'); r0 = S.ref_r0; elseif isfield(S,'ref_r0_m'); r0 = S.ref_r0_m; else; r0 = [NaN NaN NaN]; end
                Ctmp = compute_C_ECEF_to_NED(lat, lon);
                pos = (Ctmp' * S.pos_ned')' + r0(:)';
                vel = (Ctmp' * S.vel_ned')';
            elseif isfield(S,'fused_pos') && isfield(S,'fused_vel')
                % Fallback for GNSS-IMU fusion outputs
                if isfield(S,'ref_lat'); lat = S.ref_lat; elseif isfield(S,'ref_lat_rad'); lat = S.ref_lat_rad; else; lat = NaN; end
                if isfield(S,'ref_lon'); lon = S.ref_lon; elseif isfield(S,'ref_lon_rad'); lon = S.ref_lon_rad; else; lon = NaN; end
                if isfield(S,'ref_r0'); r0 = S.ref_r0; elseif isfield(S,'ref_r0_m'); r0 = S.ref_r0_m; else; r0 = [NaN NaN NaN]; end
                Ctmp = compute_C_ECEF_to_NED(lat, lon);
                pos = (Ctmp' * S.fused_pos')' + r0(:)';
                vel = (Ctmp' * S.fused_vel')';
            elseif isfield(S,'x_log')
                % 15-state log with [pos; vel] in NED
                pos_ned = S.x_log(1:3,:)';
                vel_ned = S.x_log(4:6,:)';
                if isfield(S,'ref_lat'); lat = S.ref_lat; elseif isfield(S,'ref_lat_rad'); lat = S.ref_lat_rad; else; lat = NaN; end
                if isfield(S,'ref_lon'); lon = S.ref_lon; elseif isfield(S,'ref_lon_rad'); lon = S.ref_lon_rad; else; lon = NaN; end
                if isfield(S,'ref_r0'); r0 = S.ref_r0; elseif isfield(S,'ref_r0_m'); r0 = S.ref_r0_m; else; r0 = [NaN NaN NaN]; end
                Ctmp = compute_C_ECEF_to_NED(lat, lon);
                pos = (Ctmp' * pos_ned')' + r0(:)';
                vel = (Ctmp' * vel_ned')';
            else
                error('Task7:BadData','Estimate lacks ECEF or NED position fields');
            end
            if isfield(S,'acc_ecef_ms2')
                acc = S.acc_ecef_ms2;
            else
                if exist('vel','var') && numel(t) > 1
                    acc = [zeros(1,3); diff(vel)./diff(t)];
                else
                    acc = gradient(gradient(pos)) ./ mean(diff(t))^2;
                end
            end
        end
    end
end

% -------------------------------------------------------------------------
function plot_residuals(t, res_pos, res_vel, out_dir)
%PLOT_RESIDUALS Plot position and velocity residuals in NED.
    labels = {'North','East','Down'};
    f = figure('Visible','on','Position',[100 100 900 450]);
    for j = 1:3
        subplot(2,3,j); plot(t, res_pos(:,j));
        title(labels{j}); ylabel('Pos Residual [m]'); grid on;
        subplot(2,3,3+j); plot(t, res_vel(:,j));
        xlabel('Time [s]'); ylabel('Vel Residual [m/s]'); grid on;
    end
    sgtitle('Task 7 - GNSS - Predicted Residuals');
    set(f,'PaperPositionMode','auto');
    out_pdf = fullfile(out_dir,'task7_3_residuals_position_velocity.pdf');
    out_fig = fullfile(out_dir,'task7_3_residuals_position_velocity.fig');
    out_png = fullfile(out_dir,'task7_3_residuals_position_velocity.png');
    print(f,out_pdf,'-dpdf','-bestfit');
    try, savefig(f, out_fig); catch, end
    try, print(f,out_png,'-dpng','-r200'); catch, end
    fprintf('Saved %s and %s\n', out_pdf, out_fig);
end

% -------------------------------------------------------------------------
function plot_norms(t, res_pos, res_vel, out_dir)
%PLOT_NORMS Plot norms of the residual vectors (pos, vel).
    f = figure('Visible','on');
    plot(t, vecnorm(res_pos,2,2), 'DisplayName','|pos error|'); hold on;
    plot(t, vecnorm(res_vel,2,2), 'DisplayName','|vel error|');
    xlabel('Time [s]'); ylabel('Error Norm'); legend; grid on;
    set(f,'PaperPositionMode','auto');
    out_pdf = fullfile(out_dir,'task7_3_error_norms.pdf');
    out_fig = fullfile(out_dir,'task7_3_error_norms.fig');
    out_png = fullfile(out_dir,'task7_3_error_norms.png');
    print(f,out_pdf,'-dpdf','-bestfit');
    try, savefig(f, out_fig); catch, end
    try, print(f,out_png,'-dpng','-r200'); catch, end
    fprintf('Saved %s and %s\n', out_pdf, out_fig);
end

% -------------------------------------------------------------------------
function plot_innovation_chi2(est_file, t, out_dir)
%PLOT_INNOVATION_CHI2 Plot GNSS innovation chi-square vs gate (pos/vel).
    try
        if endsWith(string(est_file), '.mat')
            S = load(est_file);
            if isfield(S,'innov_d2_pos') && isfield(S,'innov_d2_vel')
                d2p = S.innov_d2_pos(:); d2v = S.innov_d2_vel(:);
                gp = 11.345; gv = 11.345; % default 99%% for 3-DoF
                if isfield(S,'gate_chi2_pos'), gp = S.gate_chi2_pos; end
                if isfield(S,'gate_chi2_vel'), gv = S.gate_chi2_vel; end
                % Truncate to common length with time
                n = min([numel(t), numel(d2p), numel(d2v)]);
                t = t(1:n); d2p = d2p(1:n); d2v = d2v(1:n);
                f = figure('Visible','on','Position',[100 100 900 400]);
                subplot(1,2,1); plot(t, d2p, 'b-'); hold on; yline(gp,'r--','Gate'); grid on;
                title('\chi^2 Position Innovation'); xlabel('Time [s]'); ylabel('d^2');
                subplot(1,2,2); plot(t, d2v, 'b-'); hold on; yline(gv,'r--','Gate'); grid on;
                title('\chi^2 Velocity Innovation'); xlabel('Time [s]'); ylabel('d^2');
                print(f, fullfile(out_dir, 'task7_gnss_innovation_chi2.png'), '-dpng','-r200'); close(f);
            end
        end
    catch ME
        warning('Task 7: innovation chi2 plot failed: %s', ME.message);
    end
end

% -------------------------------------------------------------------------
function eul_deg = quat_to_euler_deg_batch(q)
%QUAT_TO_EULER_DEG_BATCH Convert Nx4 [q0 q1 q2 q3] to Nx3 Euler [deg].
    if isempty(q); eul_deg = []; return; end
    N = size(q,1); eul_deg = zeros(N,3);
    for k = 1:N
        q0 = q(k,1); q1 = q(k,2); q2 = q(k,3); q3 = q(k,4);
        R=[1-2*(q2^2+q3^2) 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);...
           2*(q1*q2+q0*q3) 1-2*(q1^2+q3^2) 2*(q2*q3-q0*q1);...
           2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) 1-2*(q1^2+q2^2)];
        roll  = atan2(R(3,2), R(3,3));
        pitch = -asin(max(-1,min(1,R(3,1))));
        yaw   = atan2(R(2,1), R(1,1));
        eul_deg(k,:) = rad2deg([roll, pitch, yaw]);
    end
end

% -------------------------------------------------------------------------
function plot_task7_overlay_pveul(t, pos_est, pos_truth, vel_est, vel_truth, eul_est_deg, eul_truth_deg, out_dir)
%PLOT_TASK7_OVERLAY_PVEUL Overlay pos/vel/Euler (est vs truth) in 3x3 grid.
    labels = {'North','East','Down'};
    f = figure('Visible','on','Position',[100 100 1000 850]);
    % Position
    for i = 1:3
        subplot(3,3,i); hold on; grid on;
        plot(t, pos_truth(:,i), 'k-', 'DisplayName','Truth');
        plot(t, pos_est(:,i), 'b:', 'DisplayName','Fused');
        title(['Position ' labels{i}]); ylabel('[m]');
        if i==1, legend('Location','northeast'); end
    end
    % Velocity
    for i = 1:3
        subplot(3,3,3+i); hold on; grid on;
        plot(t, vel_truth(:,i), 'k-');
        plot(t, vel_est(:,i), 'b:');
        title(['Velocity ' labels{i}]); ylabel('[m/s]');
    end
    % Euler angles (deg)
    names = {'Roll','Pitch','Yaw'};
    for i = 1:3
        subplot(3,3,6+i); hold on; grid on;
        plot(t, eul_truth_deg(:,i), 'k-');
        plot(t, eul_est_deg(:,i), 'b:');
        title(['Attitude ' names{i}]); ylabel('[deg]'); xlabel('Time [s]');
    end
    sgtitle('Task 7 - Overlay: Position, Velocity, Attitude (Euler)');
    set(f,'PaperPositionMode','auto');
    pdf = fullfile(out_dir, 'task7_overlay_pos_vel_euler_NED.pdf');
    fig = fullfile(out_dir, 'task7_overlay_pos_vel_euler_NED.fig');
    png = fullfile(out_dir, 'task7_overlay_pos_vel_euler_NED.png');
    print(f, pdf, '-dpdf', '-bestfit');
    try, savefig(f, fig); catch, end
    try, print(f, png, '-dpng','-r200'); catch, end
    fprintf('Saved %s and %s\n', pdf, fig);
end

% -------------------------------------------------------------------------
function plot_task7_quat_components(t, q_est, q_truth, out_dir)
%PLOT_TASK7_QUAT_COMPONENTS Overlay quaternion components q0..q3.
    f = figure('Visible','on','Position',[100 100 900 600]);
    names = {'q0','q1','q2','q3'};
    for i = 1:4
        subplot(2,2,i); hold on; grid on;
        plot(t, q_truth(:,i), 'k-', 'DisplayName','Truth');
        plot(t, q_est(:,i), 'b:', 'DisplayName','Fused');
        title(['Quaternion ' names{i}]); ylabel(''); xlabel('Time [s]');
        if i==1, legend('Location','best'); end
    end
    sgtitle('Task 7 - Quaternion Components Overlay');
    set(f,'PaperPositionMode','auto');
    pdf = fullfile(out_dir, 'task7_quaternion_components_overlay_NED.pdf');
    fig = fullfile(out_dir, 'task7_quaternion_components_overlay_NED.fig');
    png = fullfile(out_dir, 'task7_quaternion_components_overlay_NED.png');
    print(f, pdf, '-dpdf', '-bestfit');
    try, savefig(f, fig); catch, end
    try, print(f, png, '-dpng','-r200'); catch, end
    fprintf('Saved %s and %s\n', pdf, fig);
end
