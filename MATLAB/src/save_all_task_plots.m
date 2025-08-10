function demo_save_task5_and_task6()
% DEMO USAGE (edit names/paths as needed)
% Requires in workspace:
%   t_imu, fusedNED, fusedECEF, fusedBODY
%   truth, truthNED, truthECEF, (optional) truthBODY
outdir = fullfile(pwd,'MATLAB','results');
if ~exist(outdir,'dir'); mkdir(outdir); end

% ---- Task 5: fused-only plots
save_task5_fused_plots(t_imu, fusedNED, fusedECEF, fusedBODY, outdir);
drawnow;  % ensure pop-ups render

% ---- Task 6: overlay vs truth (does Δt align + interpolate, saves results)
if exist('truth','var') && ~isempty(truth)
    if ~exist('truthBODY','var'); truthBODY = struct(); end
    save_task6_overlay_plots(t_imu, fusedNED, fusedECEF, fusedBODY, ...
                             truth, truthNED, truthECEF, truthBODY, outdir);
    drawnow;
else
    warning('Task 6 skipped: no truth provided.');
end

    % ---- Optional: Task 7 diff grids (NED/ECEF/Body) if truth provided
    if exist('truth','var') && ~isempty(truth)
        try
            save_task7_plots(t_imu, fusedNED, fusedECEF, fusedBODY, truth, truthNED, truthECEF, truthBODY, outdir);
            drawnow;
        catch ME
            warning('Task 7 plotting failed: %s', ME.message);
        end
    end
    % ---- Optional: Quaternion comparison if quaternions available
    if exist('quatFused','var') && exist('truthQuat','var')
        try
            save_quaternion_comparison(t_imu, quatFused, truth, truthQuat, outdir);
            drawnow;
        catch ME
            warning('Quaternion comparison failed: %s', ME.message);
        end
    end
end

% ========================= Task 5 =========================
function save_task5_fused_plots(t_imu, fusedNED, fusedECEF, fusedBODY, outdir)
trel = t_imu(:) - t_imu(1);  % seconds from 0

% NED
plot_grid3x3_fused_only('Task5_NED_state', 'NED', trel, fusedNED);
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task5_state_NED');

% ECEF
plot_grid3x3_fused_only('Task5_ECEF_state', 'ECEF', trel, fusedECEF);
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task5_state_ECEF');

% Body
plot_grid3x3_fused_only('Task5_BODY_state', 'Body', trel, fusedBODY);
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task5_state_Body');
end

function plot_grid3x3_fused_only(figName, frameName, t, F)
figure('Name',figName,'WindowState','maximized');
tl = tiledlayout(3,3,'TileSpacing','compact','Padding','compact'); %#ok<NASGU>
sgtitle(sprintf('Task 5 – TRIAD – %s (Fused only)', frameName),'FontWeight','bold');

rows = {'Position','Velocity','Acceleration'};
if strcmpi(frameName,'NED'), cols = {'North','East','Down'}; else, cols = {'X','Y','Z'}; end
ylab = {'[m]','[m/s]','[m/s^2]'};
data = {F.pos, F.vel, F.acc};

ax = gobjects(9,1); k = 0;
for r = 1:3
    for c = 1:3
        k = k+1; ax(k)=nexttile; hold on; grid on;
        plot(t, data{r}(:,c), 'b-', 'LineWidth', 1.2);
        title(sprintf('%s %s', rows{r}, cols{c}));
        xlabel('Time [s]'); ylabel(ylab{r});
        axis tight;
        if r==1 && c==1, legend({'Fused GNSS+IMU'},'Location','best'); end
    end
end
linkaxes(ax,'x'); xlim([t(1) t(end)]);
end

% ========================= Task 6 =========================
function save_task6_overlay_plots(t_imu, fusedNED, fusedECEF, fusedBODY, ...
                                  truth, truthNED, truthECEF, truthBODY, outdir)
timu = t_imu(:);
trel = timu - timu(1);

% ---- estimate Δt via ECEF velocities (X & Y), then shift truth ----
vTx = interp1(truth.t, truthECEF.vel(:,1), timu, 'linear','extrap');
vFx = fusedECEF.vel(:,1);
[acfX,lagsX] = xcorr(vFx, vTx, 'coeff'); [mxX,ix]=max(acfX);

vTy = interp1(truth.t, truthECEF.vel(:,2), timu, 'linear','extrap');
vFy = fusedECEF.vel(:,2);
[acfY,lagsY] = xcorr(vFy, vTy, 'coeff'); [mxY,iy]=max(acfY);

lagSamples = round(mean([lagsX(ix), lagsY(iy)]));
dt_imu     = median(diff(timu));
dt_shift   = lagSamples * dt_imu;
fprintf('[Task6] Estimated time shift dt = %+0.3f s (truth shifted forward), corrX=%.3f, corrY=%.3f\n', dt_shift, mxX, mxY);

tTruthAligned = truth.t + dt_shift;

% ---- clip to common span and resample truth onto IMU time grid ----
tmin = max(min(tTruthAligned), min(timu));
tmax = min(max(tTruthAligned), max(timu));
mask = (timu >= tmin) & (timu <= tmax);
t_common = timu(mask);
fprintf('[Task6] Common time span: [%.3f, %.3f] s, n=%d\n', t_common(1), t_common(end), numel(t_common));

T_ECEF.pos = interp1(tTruthAligned, truthECEF.pos, t_common, 'linear');
T_ECEF.vel = interp1(tTruthAligned, truthECEF.vel, t_common, 'linear');
T_ECEF.acc = interp1(tTruthAligned, truthECEF.acc, t_common, 'linear');

T_NED.pos  = interp1(tTruthAligned, truthNED.pos,  t_common, 'linear');
T_NED.vel  = interp1(tTruthAligned, truthNED.vel,  t_common, 'linear');
T_NED.acc  = interp1(tTruthAligned, truthNED.acc,  t_common, 'linear');

if exist('truthBODY','var') && isfield(truthBODY,'pos')
    T_BODY.pos = interp1(tTruthAligned, truthBODY.pos, t_common, 'linear');
    T_BODY.vel = interp1(tTruthAligned, truthBODY.vel, t_common, 'linear');
    T_BODY.acc = interp1(tTruthAligned, truthBODY.acc, t_common, 'linear');
else
    T_BODY = struct('pos',nan(sum(mask),3), ...
                    'vel',nan(sum(mask),3), ...
                    'acc',nan(sum(mask),3));
end

% ---- plot & save (NED/ECEF/Body) ----
plot_overlay_grid3x3('Task6_NED_overlay',  'NED',  t_common - t_common(1), subsetF(fusedNED,mask),  T_NED);
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task6_overlay_state_NED');

plot_overlay_grid3x3('Task6_ECEF_overlay', 'ECEF', t_common - t_common(1), subsetF(fusedECEF,mask), T_ECEF);
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task6_overlay_state_ECEF');

plot_overlay_grid3x3('Task6_BODY_overlay', 'Body', t_common - t_common(1), subsetF(fusedBODY,mask), T_BODY);
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task6_overlay_state_Body');

% ---- compute residuals and save Task 6 results bundle ----
resN.pos = T_NED.pos - subsetF(fusedNED,mask).pos;  resN.vel = T_NED.vel - subsetF(fusedNED,mask).vel;  resN.acc = T_NED.acc - subsetF(fusedNED,mask).acc;
resE.pos = T_ECEF.pos - subsetF(fusedECEF,mask).pos; resE.vel = T_ECEF.vel - subsetF(fusedECEF,mask).vel; resE.acc = T_ECEF.acc - subsetF(fusedECEF,mask).acc;
resB.pos = T_BODY.pos - subsetF(fusedBODY,mask).pos; resB.vel = T_BODY.vel - subsetF(fusedBODY,mask).vel; resB.acc = T_BODY.acc - subsetF(fusedBODY,mask).acc;

[rmseN, finalN] = residual_metrics(resN);
[rmseE, finalE] = residual_metrics(resE);
[rmseB, finalB] = residual_metrics(resB);

results = struct();
results.t = t_common - t_common(1);
results.dt_shift = dt_shift; results.corrX = mxX; results.corrY = mxY;
results.NED = pack_frame(subsetF(fusedNED,mask), T_NED, resN, rmseN, finalN);
results.ECEF = pack_frame(subsetF(fusedECEF,mask), T_ECEF, resE, rmseE, finalE);
results.Body = pack_frame(subsetF(fusedBODY,mask), T_BODY, resB, rmseB, finalB);

res_path = fullfile(outdir,'IMU_X002_GNSS_X002_TRIAD_task6_results.mat');
save(res_path, '-struct', 'results');
fprintf('[Task6] Saved results -> %s\n', res_path);
fprintf('Frame=NED  RMSEpos=%.3f m  FinalPos=%.3f m  RMSEvel=%.3f m/s  FinalVel=%.3f m/s\n', rmseN.mag.pos, finalN.pos, rmseN.mag.vel, finalN.vel);
fprintf('Frame=ECEF RMSEpos=%.3f m  FinalPos=%.3f m  RMSEvel=%.3f m/s  FinalVel=%.3f m/s\n', rmseE.mag.pos, finalE.pos, rmseE.mag.vel, finalE.vel);
fprintf('Frame=Body RMSEpos=%.3f m  FinalPos=%.3f m  RMSEvel=%.3f m/s  FinalVel=%.3f m/s\n', rmseB.mag.pos, finalB.pos, rmseB.mag.vel, finalB.vel);
end

function plot_overlay_grid3x3(figName, frameName, t, F, T)
figure('Name',figName,'WindowState','maximized');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
sgtitle(sprintf('Task 6 – TRIAD – %s Frame (Fused vs. Truth)', frameName),'FontWeight','bold');

rows = {'Position','Velocity','Acceleration'};
if strcmpi(frameName,'NED'), cols = {'North','East','Down'}; else, cols = {'X','Y','Z'}; end
ylab = {'[m]','[m/s]','[m/s^2]'};

dataF = {F.pos, F.vel, F.acc};
dataT = {T.pos, T.vel, T.acc};
ax = gobjects(9,1); k = 0;

for r = 1:3
    for c = 1:3
        k=k+1; ax(k)=nexttile; hold on; grid on;
        plot(t, dataT{r}(:,c), 'k-', 'LineWidth', 1.1);          % truth
        plot(t, dataF{r}(:,c), 'b:', 'LineWidth', 1.4);          % fused
        title(sprintf('%s %s', rows{r}, cols{c}));
        xlabel('Time [s]'); ylabel(ylab{r});
        axis tight;
        if r==1 && c==1
            legend({'Truth','Fused GNSS+IMU (TRIAD)'},'Location','best');
        end
    end
end

function S = subsetF(F, mask)
S = struct('pos', F.pos(mask,:), 'vel', F.vel(mask,:), 'acc', F.acc(mask,:));
end

function [rmse, final] = residual_metrics(R)
% Returns component-wise and magnitude RMSE + final vector norms
rmse.pos = sqrt(mean(R.pos.^2,1));
rmse.vel = sqrt(mean(R.vel.^2,1));
rmse.acc = sqrt(mean(R.acc.^2,1));
rmse.mag.pos = sqrt(mean(sum(R.pos.^2,2)));
rmse.mag.vel = sqrt(mean(sum(R.vel.^2,2)));
rmse.mag.acc = sqrt(mean(sum(R.acc.^2,2)));
final.pos = norm(R.pos(end,:));
final.vel = norm(R.vel(end,:));
final.acc = norm(R.acc(end,:));
end

function P = pack_frame(F, T, R, rmse, final)
P = struct();
P.pos_fused = F.pos; P.vel_fused = F.vel; P.acc_fused = F.acc;
P.pos_truth = T.pos; P.vel_truth = T.vel; P.acc_truth = T.acc;
P.pos_residual = R.pos; P.vel_residual = R.vel; P.acc_residual = R.acc;
P.rmse = rmse; P.final = final;
end
linkaxes(ax,'x'); xlim([t(1) t(end)]);
end

% ========================= Task 7 (3 plots) =========================
function save_task7_plots(t_imu, fusedNED, fusedECEF, fusedBODY, truth, truthNED, truthECEF, truthBODY, outdir)
% Difference (Truth - Fused) over time for NED/ECEF/Body
trel = t_imu(:) - t_imu(1);

make_diff_grid('Task7_NED_diff',  'NED',  trel, fusedNED,  truth.t, truthNED,  outdir, 'IMU_X002_GNSS_X002_TRIAD_task7_diff_NED');
make_diff_grid('Task7_ECEF_diff', 'ECEF', trel, fusedECEF, truth.t, truthECEF, outdir, 'IMU_X002_GNSS_X002_TRIAD_task7_diff_ECEF');
if exist('truthBODY','var') && isfield(truthBODY,'pos')
    make_diff_grid('Task7_BODY_diff', 'Body', trel, fusedBODY, truth.t, truthBODY, outdir, 'IMU_X002_GNSS_X002_TRIAD_task7_diff_Body');
end
end

function make_diff_grid(figName, frameName, t_imu, F, t_truth, T, outdir, base)
% Interpolate truth to IMU time, then plot differences for pos/vel/acc
Tp = interp1(t_truth, T.pos, t_imu, 'linear','extrap');
Tv = interp1(t_truth, T.vel, t_imu, 'linear','extrap');
Ta = interp1(t_truth, T.acc, t_imu, 'linear','extrap');

Epos = Tp - F.pos; Evel = Tv - F.vel; Eacc = Ta - F.acc;

figure('Name',figName,'WindowState','maximized');
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');
sgtitle(sprintf('Task 7 – TRIAD – %s (Truth - Fused)', frameName),'FontWeight','bold');

rows = {'Position','Velocity','Acceleration'};
if strcmpi(frameName,'NED'), cols = {'North','East','Down'}; else, cols = {'X','Y','Z'}; end
ylab = {'[m]','[m/s]','[m/s^2]'};
dat = {Epos, Evel, Eacc};
ax = gobjects(9,1); k = 0;
for r = 1:3
    for c = 1:3
        k=k+1; ax(k)=nexttile; hold on; grid on;
        plot(t_imu, dat{r}(:,c), 'm-', 'LineWidth', 1.1);
        title(sprintf('%s %s', rows{r}, cols{c}));
        xlabel('Time [s]'); ylabel(ylab{r});
        axis tight;
    end
end
linkaxes(ax,'x'); xlim([t_imu(1) t_imu(end)]);
save_fig(outdir, base);
end

% ========================= Quaternion comparison =========================
function save_quaternion_comparison(t_imu, quatFused, truth, truthQuat, outdir)
% Plot Truth vs Computed quaternion components over time
trel = t_imu(:) - t_imu(1);
qt = interp1(truth.t(:), truthQuat, t_imu(:), 'linear','extrap');

figure('Name','Task_Quat_Comparison','WindowState','maximized');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
sgtitle('Quaternion Components – Truth vs. Computed');

labs = {'q_w','q_x','q_y','q_z'};
for i=1:4
    nexttile; hold on; grid on;
    plot(trel, qt(:,i), 'k-', 'DisplayName','Truth');
    plot(trel, quatFused(:,i), 'b:', 'DisplayName','Computed');
    xlabel('Time [s]'); ylabel(labs{i}); title(labs{i});
    if i==1, legend('Location','best'); end
end
save_fig(outdir,'IMU_X002_GNSS_X002_TRIAD_task_quaternion_comparison');
end

% ========================= Save helper =========================
function save_fig(outdir, base)
f = gcf;
pdf = fullfile(outdir, strcat(base,'.pdf'));
png = fullfile(outdir, strcat(base,'.png'));
fig = fullfile(outdir, strcat(base,'.fig'));
exportgraphics(f, pdf, 'ContentType','vector');   % crisp vectors
exportgraphics(f, png, 'Resolution', 200);        % quick bitmap too
savefig(f, fig);                                  % interactive MATLAB figure
fprintf('Saved: %s\nSaved: %s\nSaved: %s\n', pdf, png, fig);
end
