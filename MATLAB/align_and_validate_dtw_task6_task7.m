function align_and_validate_dtw_task6_task7(est_file, truth_file, output_dir, duration)
%ALIGN_AND_VALIDATE_DTW_TASK6_TASK7  MATLAB equivalent of align_and_validate_dtw_task6_task7.py
%
%   ALIGN_AND_VALIDATE_DTW_TASK6_TASK7(EST_FILE, TRUTH_FILE, OUTPUT_DIR, DURATION)
%   aligns the fused estimator output with ground truth using dynamic time
%   warping, then generates Task 6 overlay plots and basic Task 7 error
%   statistics. The implementation mirrors the Python script of the same
%   name. EST_FILE is a ``*_kf_output.npz`` file. TRUTH_FILE is the
%   corresponding ``STATE_X001.txt``. OUTPUT_DIR defaults to ``results`` and
%   DURATION to 1250 seconds.
%
%   This script requires the Signal Processing Toolbox for the DTW
%   implementation and uses ``py.numpy.load`` to read NPZ files.
%
%   The function saves ``aligned_<est_file>.mat`` and ``aligned_<truth_file>``
%   under OUTPUT_DIR and prints summary error statistics.

if nargin < 3 || isempty(output_dir)
    output_dir = 'MATLAB/results';
end
if nargin < 4 || isempty(duration)
    duration = 1250.0;
end
if ~exist(output_dir, 'dir'); mkdir(output_dir); end

% -------------------------------------------------------------------------
% Load estimator data from NPZ
% -------------------------------------------------------------------------
npz = py.numpy.load(string(est_file));
pos_ned = double(npz{'pos_ned_m'});
vel_ned = double(npz{'vel_ned_ms'});
quat = double(npz{'att_quat'});
time_s = double(npz{'time_s'});
ref_lat = double(npz{'ref_lat_rad'});
ref_lon = double(npz{'ref_lon_rad'});
ref_r0 = double(npz{'ref_r0_m'});

% -------------------------------------------------------------------------
% Load truth trajectory
% -------------------------------------------------------------------------
% Truth logs contain a comment header
truth_raw = read_state_file(truth_file);
truth_time = truth_raw(:,1);
truth_pos_ecef = truth_raw(:,2:4);

% Convert truth ECEF to NED relative to estimator reference
C_e2n = compute_C_ECEF_to_NED(ref_lat, ref_lon);
truth_pos = (C_e2n * (truth_pos_ecef' - ref_r0)).';

% -------------------------------------------------------------------------
% Smooth and normalise for DTW
% -------------------------------------------------------------------------
pos_est_sm = sgolayfilt(pos_ned, 2, 7);
pos_truth_sm = sgolayfilt(truth_pos, 2, 7);

sig_est = vecnorm(pos_est_sm, 2, 2);
sig_truth = vecnorm(pos_truth_sm, 2, 2);

[~,ix,iy] = dtw(sig_est, sig_truth);

aligned_pos_est = pos_ned(ix,:);
aligned_vel_est = vel_ned(ix,:);
aligned_quat = quat(ix,:);
aligned_pos_truth = truth_pos(iy,:);
common_time = linspace(0, duration, numel(ix)).';

% Save aligned data
[~,ename,~] = fileparts(est_file);
[~,tname,ext] = fileparts(truth_file);
aligned_est_file = fullfile(output_dir, ['aligned_' ename '.mat']);
aligned_truth_file = fullfile(output_dir, ['aligned_' tname ext]);

save(aligned_est_file, 'common_time', 'aligned_pos_est', 'aligned_vel_est', ...
    'aligned_quat', 'ref_lat', 'ref_lon', 'ref_r0');
writematrix([common_time aligned_pos_truth], aligned_truth_file);

% -------------------------------------------------------------------------
% Generate plots similar to plot_task6_results.py
% -------------------------------------------------------------------------
plot_task6_results(aligned_pos_est, aligned_vel_est, aligned_quat, common_time, ...
    aligned_pos_truth, common_time, ref_lat, ref_lon, ref_r0, output_dir, ename);

% -------------------------------------------------------------------------
% Basic Task 7 error metrics
% -------------------------------------------------------------------------
err = vecnorm(aligned_pos_est - aligned_pos_truth, 2, 2);
fprintf('Aligned time range: 0 to %.2f seconds\n', common_time(end));
fprintf('Position errors: mean=%.4f m, std=%.4f m\n', mean(err), std(err));
end

% -------------------------------------------------------------------------
function plot_task6_results(est_pos, est_vel, est_quat, est_time, ...
    truth_pos, truth_time, ref_lat, ref_lon, ref_r0, out_dir, method)
%PLOT_TASK6_RESULTS  Simplified Task 6 plots in NED frame.
%
%   PLOT_TASK6_RESULTS(EST_POS, EST_VEL, EST_QUAT, EST_TIME, TRUTH_POS,
%   TRUTH_TIME, REF_LAT, REF_LON, REF_R0, OUT_DIR, METHOD) overlays the
%   fused estimator output with the truth data and saves the figure as
%   ``<METHOD>_task6_results_ned.pdf`` under OUT_DIR. METHOD should
%   encode the dataset and algorithm name, e.g.,
%   ``IMU_X001_GNSS_X001_Davenport``. This mirrors ``plot_task6_results.py``.

truth_pos_i = interp1(truth_time, truth_pos, est_time, 'linear', 'extrap');
dt = mean(diff(est_time));
est_acc = gradient(est_vel, dt);

labels = {'North','East','Down'};
fig = figure('Visible','off','Position',[100 100 900 800]);
for i=1:3
    subplot(3,3,i); plot(est_time, est_pos(:,i), 'b', est_time, truth_pos_i(:,i), '--r');
    title(['Position ' labels{i}]); ylabel('[m]'); grid on;
    subplot(3,3,3+i); plot(est_time, est_vel(:,i), 'b'); title(['Velocity ' labels{i}]); ylabel('[m/s]'); grid on;
    subplot(3,3,6+i); plot(est_time, est_acc(:,i), 'b'); title(['Acceleration ' labels{i}]); ylabel('[m/s^2]'); grid on;
    if i==3; xlabel('Time [s]'); end
end
sgtitle(sprintf('%s Task 6 Results (NED)', method));
set(fig,'PaperPositionMode','auto');
pdf = fullfile(out_dir, sprintf('%s_task6_results_ned.pdf', method));
print(fig, pdf, '-dpdf');
close(fig);
end

function C = compute_C_ECEF_to_NED(lat_rad, lon_rad)
%COMPUTE_C_ECEF_TO_NED  Local rotation matrix from ECEF to NED.
s_lat = sin(lat_rad); c_lat = cos(lat_rad);
s_lon = sin(lon_rad); c_lon = cos(lon_rad);
C = [-s_lat*c_lon, -s_lat*s_lon,  c_lat;
     -s_lon,       c_lon,       0;
     -c_lat*c_lon, -c_lat*s_lon, -s_lat];
end
