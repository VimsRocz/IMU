function t = unwrap_seconds(subsec, dt_hint)
%UNWRAP_SECONDS Reconstruct a monotonic time vector from fractional seconds.
%   t = UNWRAP_SECONDS(subsec, dt_hint) returns a timeline where the
%   fractional-second counter SUBSEC is unwrapped and optionally snapped to a
%   uniform grid using DT_HINT. SUBSEC is assumed to reset to ~0 every second.
%
%   Usage:
%       t = unwrap_seconds(subsec)
%       t = unwrap_seconds(subsec, dt_hint)
%
%   This helper mirrors ``_unwrap_seconds`` in ``src/utils/timeline.py``.
%
%   Inputs:
%       subsec  - fractional seconds that reset to ~0 every second
%       dt_hint - optional expected sample interval (e.g., 0.0025)
%
%   Output:
%       t       - unwrapped time vector starting at zero

if nargin < 2 || isempty(dt_hint)
    dt_hint = median(diff(subsec(subsec>0)));
end

d = diff(subsec(:));
wrap = d < -0.5;              % detect negative jumps (wraps)
step = [0; cumsum(wrap)];
t = double(subsec(:)) + step; % add whole seconds per wrap

t = t - t(1);                 % start at zero

if isfinite(dt_hint) && dt_hint > 0
    n = numel(t);
    t = (0:n-1)' * dt_hint;   % lock to uniform timeline
end
end
