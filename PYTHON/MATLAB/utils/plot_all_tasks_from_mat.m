function plot_all_tasks_from_mat(mat_file)
%PLOT_ALL_TASKS_FROM_MAT  Render core task plots from Python fusion output (.mat)
%   PLOT_ALL_TASKS_FROM_MAT('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat')
%   Generates figures for:
%     - Task 3: Attitude angles (Roll/Pitch/Yaw)
%     - Task 4/6: Position and Velocity in NED (overlay truth if present)
%     - Residuals: position/velocity residuals
%     - Body-frame position and velocity
%   Figures are not saved by default; use MATLAB's Save if needed.

% If no file is provided, pick the newest *_kf_output.mat from results/.
if nargin < 1 || isempty(mat_file)
    candidates = {};
    roots = { 'results', fullfile('PYTHON','results') };
    picked = '';
    for r = 1:numel(roots)
        if exist(roots{r}, 'dir')
            d = dir(fullfile(roots{r}, '*_kf_output.mat'));
            if ~isempty(d)
                [~, idx] = max([d.datenum]);
                picked = fullfile(roots{r}, d(idx).name);
                break
            end
        end
    end
    if isempty(picked)
        [f,p] = uigetfile({'*_kf_output.mat','Fusion output (*.mat)'}, 'Select fusion result');
        if isequal(f,0)
            error('No file selected and no default outputs found. Run Python first.');
        end
        mat_file = fullfile(p,f);
    else
        mat_file = picked;
    end
end
fprintf('Using: %s\n', mat_file);
S = load(mat_file);

% --- Harmonise common fields ----------------------------------------------
t = pick_first(S, {'t','time','time_s'});
pos_ned = pick_first(S, {'pos_ned','pos_ned_m','fused_pos'});
vel_ned = pick_first(S, {'vel_ned','vel_ned_ms','fused_vel'});
euler   = pick_first(S, {'euler'});  % [Nx3] deg (roll,pitch,yaw)
quat    = pick_first(S, {'attitude_q','att_quat','quat_log'}); % [Nx4] [w x y z] or [x y z w]
res_pos = pick_first(S, {'residual_pos'});
res_vel = pick_first(S, {'residual_vel'});
t_res   = pick_first(S, {'time_residuals','t_residuals'});
pos_body = pick_first(S, {'pos_body','pos_body_m'});
vel_body = pick_first(S, {'vel_body','vel_body_ms'});

% Optional truth in ECEF
truth_pos_ecef = pick_first(S, {'truth_pos_ecef'});
truth_vel_ecef = pick_first(S, {'truth_vel_ecef'});
truth_time     = pick_first(S, {'truth_time'});
ref_lat = pick_first(S, {'ref_lat','ref_lat_rad'});  % may already be radians
ref_lon = pick_first(S, {'ref_lon','ref_lon_rad'});
ref_r0  = pick_first(S, {'ref_r0','ref_r0_m'});

% Ensure column vectors as appropriate
t = t(:);
if ~isempty(t_res), t_res = t_res(:); end

% Build overview figure with subplots (4x3)
fig = figure('Name','Tasks Overview');
tl = tiledlayout(fig, 4, 3, 'TileSpacing','compact', 'Padding','compact');

% Prepare TRUTH in NED if available
[truth_pos_ned, truth_vel_ned] = deal([]);
if ~isempty(truth_pos_ecef) && ~isempty(ref_lat) && ~isempty(ref_lon) && ~isempty(ref_r0)
    % Convert truth ECEF -> NED using reference
    if max(abs(ref_lat)) > 2*pi  % convert deg to rad if needed
        lat_rad = deg2rad(ref_lat(1)); lon_rad = deg2rad(ref_lon(1));
    else
        lat_rad = ref_lat(1); lon_rad = ref_lon(1);
    end
    Ce2n = C_ECEF_to_NED(lat_rad, lon_rad);
    dp = (truth_pos_ecef - ref_r0(:)');
    truth_pos_ned = (Ce2n * dp')';
    truth_vel_ned = (Ce2n * truth_vel_ecef')';
    % Align truth time if available
    if ~isempty(truth_time) && ~isempty(t)
        ti = min(max(t, truth_time(1)), truth_time(end));
        idx = interp1(truth_time(:), (1:numel(truth_time))', ti, 'nearest','extrap');
        truth_pos_ned = truth_pos_ned(idx,:);
        truth_vel_ned = truth_vel_ned(idx,:);
    end
end

labels = {'North','East','Down'};

% Row 1: Attitude (Euler or from quaternion)
if isempty(euler) && ~isempty(quat)
    q = quat; if size(q,2)==4 && mean(abs(q(:,1)))>=0.3, eul = quat_to_euler_deg(q,'wxyz'); else, eul = quat_to_euler_deg(q,'xyzw'); end
else
    eul = euler;
end
if ~isempty(eul)
    titles_e = {'Roll','Pitch','Yaw'};
    for i=1:3
        ax = nexttile(tl, i);
        plot(t, eul(:,i)); grid on
        ylabel(ax, sprintf('%s [deg]', titles_e{i}));
        if i==1, title(ax,'Task 3 — Attitude'); end
    end
end

% Row 2: Position NED
for i=1:3
    ax = nexttile(tl, 3+i);
    plot(t, pos_ned(:,i), 'b-'); hold on; grid on
    if ~isempty(truth_pos_ned), plot(t, truth_pos_ned(:,i), 'r--'); end
    ylabel(ax, sprintf('%s [m]', labels{i}));
    if i==1, title(ax,'Task 4/6 — Position NED'); end
end

% Row 3: Velocity NED
for i=1:3
    ax = nexttile(tl, 6+i);
    plot(t, vel_ned(:,i), 'b-'); hold on; grid on
    if ~isempty(truth_vel_ned), plot(t, truth_vel_ned(:,i), 'r--'); end
    ylabel(ax, sprintf('V%s [m/s]', labels{i}(1)));
    if i==1, title(ax,'Task 4/6 — Velocity NED'); end
end

% Row 4: Residuals (prefer position; else velocity)
if ~isempty(res_pos)
    for i=1:3
        ax = nexttile(tl, 9+i);
        if ~isempty(t_res), plot(t_res, res_pos(:,i)); else, plot(res_pos(:,i)); end
        grid on
        ylabel(ax, sprintf('Res %s [m]', labels{i}));
        if i==1
            xlabel(ax, 'Time [s]'); title(ax,'Residuals — Position');
        else
            xlabel(ax, '');
        end
    end
elseif ~isempty(res_vel)
    for i=1:3
        ax = nexttile(tl, 9+i);
        if ~isempty(t_res), plot(t_res, res_vel(:,i)); else, plot(res_vel(:,i)); end
        grid on
        ylabel(ax, sprintf('Res V%s [m/s]', labels{i}(1)));
        if i==1
            xlabel(ax, 'Time [s]'); title(ax,'Residuals — Velocity');
        else
            xlabel(ax, '');
        end
    end
else
    % If no residuals, show body-frame if available
    if ~isempty(pos_body)
        for i=1:3
            ax = nexttile(tl, 9+i);
            plot(t, pos_body(:,i)); grid on
            labels_b = {'X','Y','Z'};
            ylabel(ax, sprintf('Body %s [m]', labels_b{i})); if i==1, title(ax,'Body-frame Position'); end
        end
    elseif ~isempty(vel_body)
        for i=1:3
            ax = nexttile(tl, 9+i);
            plot(t, vel_body(:,i)); grid on
            labels_v = {'u','v','w'};
            ylabel(ax, sprintf('Body %s [m/s]', labels_v{i})); if i==1, title(ax,'Body-frame Velocity'); end
        end
    end
end

xlabel(tl,'Time [s]');

end

% ------------------------------ helpers ------------------------------------
function v = pick_first(S, names)
v = [];
for k = 1:numel(names)
    if isfield(S, names{k})
        v = S.(names{k});
        return
    end
end
end

function Ce2n = C_ECEF_to_NED(lat_rad, lon_rad)
sL = sin(lat_rad); cL = cos(lat_rad);
sO = sin(lon_rad); cO = cos(lon_rad);
Ce2n = [ -sL*cO, -sL*sO,  cL; 
         -sO,      cO,    0; 
         -cL*cO, -cL*sO, -sL ];
end

function eul = quat_to_euler_deg(q, order)
% Return [roll,pitch,yaw] in degrees from quaternion time series
% order: 'wxyz' or 'xyzw'
if strcmp(order,'wxyz')
    w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
else
    x=q(:,1); y=q(:,2); z=q(:,3); w=q(:,4);
end
% Using aerospace convention (XYZ intrinsic / roll-pitch-yaw)
ysqr = y.*y;
t0 = +2.0 .* (w.*x + y.*z);
t1 = +1.0 - 2.0.*(x.*x + ysqr);
roll = atan2(t0, t1);
t2 = +2.0 .* (w.*y - z.*x); t2 = max(min(t2,1.0),-1.0);
pitch = asin(t2);
t3 = +2.0 .* (w.*z + x.*y);
t4 = +1.0 - 2.0.*(ysqr + z.*z);
yaw = atan2(t3, t4);
eul = [roll, pitch, yaw] .* (180/pi);
end
