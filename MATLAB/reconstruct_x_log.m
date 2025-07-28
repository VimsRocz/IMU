function S = reconstruct_x_log(S)
%RECONSTRUCT_X_LOG Construct missing x_log state history.
%   S = RECONSTRUCT_X_LOG(S) ensures the structure S contains an ``x_log``
%   field following the Task 5 convention. When absent, ``x_log`` is
%   reconstructed from available position, velocity and attitude logs. Any
%   missing components are filled with zeros. The output has size 15xN where
%   N is inferred from the existing data.
%
%   This helper mirrors the Python pipeline behaviour so Task_6 can run on
%   MATLAB or Python results.
%
%   Example:
%       S = reconstruct_x_log(S);
%
%   See also TASK_6.

    if isfield(S, 'x_log')
        return
    end

    % Determine number of samples from the available fields
    if isfield(S, 'pos_ned')
        N = size(S.pos_ned, 1);
    elseif isfield(S, 'fused_pos')
        N = size(S.fused_pos, 1);
    elseif isfield(S, 'vel_ned')
        N = size(S.vel_ned, 1);
    elseif isfield(S, 'fused_vel')
        N = size(S.fused_vel, 1);
    elseif isfield(S, 'euler_log')
        N = size(S.euler_log, 2);
    elseif isfield(S, 'quat_log')
        if size(S.quat_log,1) == 4
            N = size(S.quat_log,2);
        else
            N = size(S.quat_log,1);
        end
    else
        error('reconstruct_x_log:InsufficientData', ...
            'Cannot infer state length to rebuild x_log');
    end

    x_log = zeros(15, N);

    if isfield(S, 'pos_ned')
        x_log(1:3, :) = S.pos_ned';
    elseif isfield(S, 'fused_pos')
        x_log(1:3, :) = S.fused_pos';
    end
    if isfield(S, 'vel_ned')
        x_log(4:6, :) = S.vel_ned';
    elseif isfield(S, 'fused_vel')
        x_log(4:6, :) = S.fused_vel';
    end

    if isfield(S, 'euler_log')
        eul = S.euler_log;
    elseif isfield(S, 'quat_log')
        q = S.quat_log;
        if size(q,1) ~= 4
            q = q';
        end
        eul = quat_to_euler_local(q);
    else
        eul = zeros(3, N);
    end
    if size(eul,2) ~= N
        eul = eul';
    end
    x_log(7:9, :) = eul;

    if isfield(S, 'accel_bias_log')
        x_log(10:12, :) = S.accel_bias_log;
    elseif isfield(S, 'accel_bias')
        x_log(10:12, :) = repmat(S.accel_bias(:), 1, N);
    end
    if isfield(S, 'gyro_bias_log')
        x_log(13:15, :) = S.gyro_bias_log;
    elseif isfield(S, 'gyro_bias')
        x_log(13:15, :) = repmat(S.gyro_bias(:), 1, N);
    end

    S.x_log = x_log;
end

function euler = quat_to_euler_local(q)
%QUAT_TO_EULER_LOCAL Convert quaternion to XYZ Euler angles.
    w = q(1,:); x = q(2,:); y = q(3,:); z = q(4,:);
    sinr_cosp = 2 .* (w .* x + y .* z);
    cosr_cosp = 1 - 2 .* (x.^2 + y.^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    sinp = 2 .* (w .* y - z .* x);
    pitch = asin(max(min(sinp, 1), -1));

    siny_cosp = 2 .* (w .* z + x .* y);
    cosy_cosp = 1 - 2 .* (y.^2 + z.^2);
    yaw = atan2(siny_cosp, cosy_cosp);
    euler = [roll; pitch; yaw];
end
