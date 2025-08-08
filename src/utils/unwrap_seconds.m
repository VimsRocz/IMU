function t = unwrap_seconds(subsec, dt_hint)
%UNWRAP_SECONDS Convert fractional seconds with wraps into a uniform timeline.
%   T = UNWRAP_SECONDS(SUBSEC, DT_HINT) mirrors the Python helper
%   ``_unwrap_seconds`` by unwrapping a sub-second clock that resets every
%   second and returning a timeline locked to a uniform grid with spacing
%   DT_HINT (default 0.0025 s).

if nargin < 2 || isempty(dt_hint)
    dt_hint = 0.0025;
end

subsec = double(subsec(:));
d = diff(subsec);
wrap = d < -0.5;
step = [0; cumsum(double(wrap))];
t = subsec + step;
t = t - t(1);
n = numel(t);
t = (0:n-1)' * dt_hint;
end

