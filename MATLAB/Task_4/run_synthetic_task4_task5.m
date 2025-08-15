% Synthetic demo to validate Task 4 and Task 5 plotting functions.
% Generates synthetic GNSS, IMU, Fused, and Truth data across frames and calls:
%  - task4_plot_comparisons
%  - task5_plot_fusion_results

clear; clc;

% Timelines
T = 1400;                 % seconds
N_imu   = 500000;         % IMU samples
N_gnss  = 1250;           % GNSS samples
N_fused = 20087;          % Fused samples

t_imu   = linspace(0,T,N_imu).';
t_gnss  = linspace(0,T,N_gnss).';
t_fused = linspace(0,T,N_fused).';

% Helper to synthesize a 3-col signal with slight drifts/noise
mk_sig = @(t, f1, f2, amp) [ ...
    amp(1)*sin(2*pi*f1*t)             + 0.001*t + 0.02*randn(size(t)); ...
    amp(2)*cos(2*pi*f2*t + 0.5)       - 0.0005*t+ 0.02*randn(size(t)); ...
    amp(3)*sin(2*pi*(f1+f2/2)*t+0.2)  + 0.0007*t+ 0.02*randn(size(t)) ...
];

% Build frames for GNSS (lower rate, smoother)
gnss.ned.pos  = mk_sig(t_gnss, 0.002, 0.003, [100; 120; 20]);
gnss.ned.vel  = mk_sig(t_gnss, 0.01,  0.02,  [  3;   3;  2]);
gnss.ned.acc  = mk_sig(t_gnss, 0.05,  0.03,  [  0.5; 0.4; 0.3]);
% Derive ECEF/Body by simple linear transforms for demo
R = [0 1 0; -1 0 0; 0 0 1];
gnss.ecef.pos = gnss.ned.pos * R;  gnss.ecef.vel = gnss.ned.vel * R;  gnss.ecef.acc = gnss.ned.acc * R;
gnss.body.pos = gnss.ned.pos * R'; gnss.body.vel = gnss.ned.vel * R'; gnss.body.acc = gnss.ned.acc * R';

% IMU-only (raw/integrated) high rate with more drift/noise
imu.ned.pos  = mk_sig(t_imu, 0.002, 0.003, [100; 120; 20]) + [0.02*t_imu, -0.015*t_imu, 0.01*t_imu];
imu.ned.vel  = mk_sig(t_imu, 0.01,  0.02,  [  4;   4;  3])  + [0.02*ones(size(t_imu)), 0.02*sin(0.001*t_imu), 0*t_imu];
imu.ned.acc  = mk_sig(t_imu, 0.05,  0.03,  [  0.7; 0.6; 0.4]);

% Tailor IMU final integrated NED velocity to match console stats
target_vf_ned = [-1178.306, 8434.896, -1051.365]; % m/s
base_end = imu.ned.vel(end, :);
ramp = (target_vf_ned - base_end) ./ T; % per-second slope
imu.ned.vel = imu.ned.vel + t_imu .* ramp;
imu.ecef.pos = imu.ned.pos * R;  imu.ecef.vel = imu.ned.vel * R;  imu.ecef.acc = imu.ned.acc * R;
imu.body.pos = imu.ned.pos * R'; imu.body.vel = imu.ned.vel * R'; imu.body.acc = imu.ned.acc * R';

% Fused results (medium rate), closer to GNSS but smoother and bounded
fused.ned.pos = mk_sig(t_fused, 0.002, 0.003, [100; 120; 20]) + 0.1*randn(N_fused,3);
fused.ned.vel = mk_sig(t_fused, 0.01,  0.02,  [  2;   2;  1.5]) + 0.05*randn(N_fused,3);
fused.ned.acc = mk_sig(t_fused, 0.05,  0.03,  [  0.4; 0.3; 0.25])+ 0.03*randn(N_fused,3);

% Keep fused velocities reasonable at the end (no blow-up)
target_fused_vf = [-8, 12, -5]; % m/s (arbitrary small values)
base_end_f = fused.ned.vel(end, :);
ramp_f = (target_fused_vf - base_end_f) ./ T;
fused.ned.vel = fused.ned.vel + t_fused .* ramp_f;
fused.ecef.pos = fused.ned.pos * R;  fused.ecef.vel = fused.ned.vel * R;  fused.ecef.acc = fused.ned.acc * R;
fused.body.pos = fused.ned.pos * R'; fused.body.vel = fused.ned.vel * R'; fused.body.acc = fused.ned.acc * R';

% Mixed frame for Task 5 comparisons (arbitrary combo for demo)
fused.mixed.pos = [fused.ned.pos(:,1), fused.ecef.pos(:,2), fused.body.pos(:,3)];
fused.mixed.vel = [fused.ned.vel(:,1), fused.ecef.vel(:,2), fused.body.vel(:,3)];
fused.mixed.acc = [fused.ned.acc(:,1), fused.ecef.acc(:,2), fused.body.acc(:,3)];

% Synthetic truth aligned with fused results
truth = struct();
% Position residual stats from console (mean/std per axis)
mu_pos  = [0.0228, 0.0004, -0.0364];
sig_pos = [0.2677, 0.0273, 0.4019];
res_pos = mu_pos + randn(N_fused,3) .* sig_pos; % fused - truth
truth.ned.pos = fused.ned.pos - res_pos;
% Velocity/acc residuals (arbitrary smaller noise)
res_vel = randn(N_fused,3) .* [0.15, 0.12, 0.18];
res_acc = randn(N_fused,3) .* [0.08, 0.06, 0.07];
truth.ned.vel = fused.ned.vel - res_vel;
truth.ned.acc = fused.ned.acc - res_acc;
% ECEF/Body via transforms for demo
truth.ecef.pos = truth.ned.pos * R;  truth.ecef.vel = truth.ned.vel * R;  truth.ecef.acc = truth.ned.acc * R;
truth.body.pos = truth.ned.pos * R'; truth.body.vel = truth.ned.vel * R'; truth.body.acc = truth.ned.acc * R';

% Provide blow-up times for Task 5 (subset for readability; console shows 9662)
num_blowups = 80; % limit to 80 vertical lines to avoid clutter
fused.blowups = sort(T*rand(1,num_blowups));

% Time containers for both tasks
time_task4 = struct('gnss', t_gnss, 'imu', t_imu, 'fused', t_fused);
time_task5 = struct('truth', t_fused, 'fused', t_fused, 'blowups', fused.blowups);

% Inject IMU velocity spikes near blow-up times to visualize issues
idxs = unique(max(1, min(N_imu, round(fused.blowups/T*(N_imu-1)) + 1))));
for j = 1:numel(idxs)
    ax = randi(3);                         % random axis for spike
    amp = (50 + 150*rand()) * sign(randn); % 50–200 m/s spike
    imu.ned.vel(idxs(j), ax) = imu.ned.vel(idxs(j), ax) + amp;
end
% Update transformed frames after spikes
imu.ecef.vel = imu.ned.vel * R;  imu.body.vel = imu.ned.vel * R';

% Run Task 4 plotting
fprintf('Running Task 4 synthetic plotting...\n');
task4_plot_comparisons(gnss, imu, fused, time_task4);

% Run Task 5 plotting
fprintf('Running Task 5 synthetic plotting...\n');
task5_plot_fusion_results(fused, truth, time_task5.fused, time_task5.blowups);

fprintf('Done. Check PNGs under MATLAB/results.\n');

% Run Task 6 overlay using the synthetic truth
try
    fprintf('Running Task 6 synthetic overlay...\n');
    time_task6 = struct('fused', t_fused, 'truth', t_fused);
    task6_overlay_truth(fused, truth, time_task6);
catch ME
    warning('Task 6 overlay failed in synthetic demo: %s', ME.message);
end
