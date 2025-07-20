function vel = derive_velocity(time_s, pos, window_length, polyorder)
%DERIVE_VELOCITY Estimate velocity using Savitzky-Golay smoothing.
%
% Usage:
%   vel = derive_velocity(time_s, pos, window_length, polyorder)
%
% This is a MATLAB stub mirroring ``derive_velocity`` in
% ``task6_plot_fused_trajectory.py``. It should smooth the input position
% with a Savitzky-Golay filter and apply a central difference.
%
% TODO: implement full MATLAB version.

if nargin < 3
    window_length = 11;
end
if nargin < 4
    polyorder = 2;
end

% Placeholder implementation
vel = [zeros(1, size(pos,2)); diff(pos)./diff(time_s)];
vel(1,:) = vel(2,:);
end
