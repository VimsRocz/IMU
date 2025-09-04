function plot_from_mat(mat_file)
%PLOT_FROM_MAT  Quick MATLAB plotter for Python fusion outputs (.mat)
%   PLOT_FROM_MAT('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat')
%   Produces basic position and velocity time-series in NED.

if nargin < 1
    error('Usage: plot_from_mat(''results/<tag>_kf_output.mat'')');
end
S = load(mat_file);

% Harmonise time
if ~isfield(S,'t')
    if isfield(S,'time'); S.t = S.time; 
    elseif isfield(S,'time_s'); S.t = S.time_s; 
    else; error('No time vector found (t/time/time_s).'); end
end

% Harmonise position and velocity in NED
if ~isfield(S,'pos_ned')
    if isfield(S,'pos_ned_m'); S.pos_ned = S.pos_ned_m; 
    elseif isfield(S,'fused_pos'); S.pos_ned = S.fused_pos; 
    else; error('No NED position found (pos_ned/pos_ned_m/fused_pos).'); end
end
if ~isfield(S,'vel_ned')
    if isfield(S,'vel_ned_ms'); S.vel_ned = S.vel_ned_ms; 
    elseif isfield(S,'fused_vel'); S.vel_ned = S.fused_vel; 
    else; error('No NED velocity found (vel_ned/vel_ned_ms/fused_vel).'); end
end

t = S.t(:);
pos = S.pos_ned;  % Nx3 [N,E,D]
vel = S.vel_ned;  % Nx3 [N,E,D]

figure('Name','Position NED');
subplot(3,1,1); plot(t, pos(:,1)); ylabel('North [m]'); grid on
subplot(3,1,2); plot(t, pos(:,2)); ylabel('East [m]'); grid on
subplot(3,1,3); plot(t, pos(:,3)); ylabel('Down [m]'); xlabel('Time [s]'); grid on

figure('Name','Velocity NED');
subplot(3,1,1); plot(t, vel(:,1)); ylabel('V_N [m/s]'); grid on
subplot(3,1,2); plot(t, vel(:,2)); ylabel('V_E [m/s]'); grid on
subplot(3,1,3); plot(t, vel(:,3)); ylabel('V_D [m/s]'); xlabel('Time [s]'); grid on

end
