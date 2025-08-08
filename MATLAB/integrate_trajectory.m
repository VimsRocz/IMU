function [pos, vel, acc, pos_ecef, vel_ecef] = integrate_trajectory(acc_body, imu_time, C_B_N, g_NED, varargin)
%INTEGRATE_TRAJECTORY Integrate body-frame accelerations.
%   This is a simplified MATLAB counterpart to the Python function of the
%   same name. Only constant gravity subtraction in the NED frame is
%   implemented. Additional parameters are accepted for API compatibility.
%
%   Inputs:
%     ACC_BODY - Nx3 body-frame specific force measurements.
%     IMU_TIME - Nx1 timestamps in seconds.
%     C_B_N    - 3x3 body-to-NED rotation matrix.
%     G_NED    - 3x1 gravity vector in NED (positive down).
%     'debug'  - logical flag to print intermediate velocities (default false).
%
%   Outputs:
%     POS - Nx3 position in NED (m).
%     VEL - Nx3 velocity in NED (m/s).
%     ACC - Nx3 linear acceleration in NED (m/s^2).
%     POS_ECEF, VEL_ECEF - empty arrays; ECEF support is pending.
%
%   This function is provided for API parity with the Python code.

pos_ecef = [];
vel_ecef = [];

debug = false;
for k = 1:2:numel(varargin)
    if ischar(varargin{k}) || isstring(varargin{k})
        switch lower(string(varargin{k}))
            case 'debug'
                debug = logical(varargin{k+1});
        end
    end
end

n = size(imu_time,1);
pos = zeros(n,3);
vel = zeros(n,3);
acc = zeros(n,3);
mid_idx = floor(n/2);
for i = 2:n
    dt = imu_time(i) - imu_time(i-1);
    f_ned = C_B_N * acc_body(i,:)';
    a_ned = f_ned + g_NED;
    acc(i,:) = a_ned';
    vel(i,:) = vel(i-1,:) + a_ned' * dt;
    pos(i,:) = pos(i-1,:) + vel(i,:) * dt;
    if debug && (i == 2 || i == mid_idx || i == n)
        fprintf('[DEBUG] i=%d dt=%.4f vel=[%.4f %.4f %.4f] acc=[%.4f %.4f %.4f]\n', ...
            i, dt, vel(i,1), vel(i,2), vel(i,3), a_ned(1), a_ned(2), a_ned(3));
    end
end
end

