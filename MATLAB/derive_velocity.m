function vel = derive_velocity(time_s, pos, window_length, polyorder)
%DERIVE_VELOCITY  Estimate velocity from a position trajectory.
%
%   vel = DERIVE_VELOCITY(time_s, pos, window_length, polyorder) smooths the
%   Nx3 position array ``pos`` (metres) using a Savitzky--Golay filter and
%   differentiates it with a central difference scheme. ``time_s`` is an Nx1
%   vector of timestamps in seconds. ``window_length`` specifies the filter
%   window length (samples) and must be odd; ``polyorder`` sets the polynomial
%   order. The returned ``vel`` is Nx3 in metres per second. This implementation
%   mirrors ``velocity_utils.derive_velocity`` in the Python codebase.

if nargin < 3 || isempty(window_length)
    window_length = 11;
end
if mod(window_length, 2) == 0
    window_length = window_length + 1; % ensure odd length
end
if nargin < 4 || isempty(polyorder)
    polyorder = 2;
end

pos_sm = sgolayfilt(pos, polyorder, window_length);
vel = zeros(size(pos_sm));

dt = diff(time_s(:));
if isempty(dt)
    return; % single sample
end

% central difference for interior points
vel(2:end-1, :) = (pos_sm(3:end, :) - pos_sm(1:end-2, :)) ./ ...
    (dt(2:end) + dt(1:end-1));

% forward/backward difference at the edges
vel(1, :) = (pos_sm(2, :) - pos_sm(1, :)) ./ dt(1);
vel(end, :) = (pos_sm(end, :) - pos_sm(end-1, :)) ./ dt(end);

end
