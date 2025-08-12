function [t_fix, dt_used, nfix] = ensure_monotonic(t, logger, dt_hint)
%ENSURE_MONOTONIC Return strictly-increasing time, repairing non-monotonicity.
%   [T_FIX, DT_USED, NFIX] = ENSURE_MONOTONIC(T, LOGGER, DT_HINT) repairs
%   non-increasing steps in T using the median positive dt or DT_HINT.
%
%   Usage:
%       [t_fix, dt_used, nfix] = ensure_monotonic(t_raw, @fprintf, 1/400);
%
%   Inputs:
%       t       - time vector
%       logger  - optional logger function (e.g., @fprintf)
%       dt_hint - optional dt hint used when dt cannot be inferred
%
%   Outputs:
%       t_fix   - repaired time vector
%       dt_used - dt applied to repairs (empty if none)
%       nfix    - number of repaired steps
%
%   This mirrors the Python ``ensure_monotonic`` utility.

if nargin < 2, logger = []; end
if nargin < 3, dt_hint = []; end

t = double(t(:));
if numel(t) < 2 || all(diff(t) > 0)
    t_fix = t;
    dt_used = [];
    nfix = 0;
    return;
end

dt = diff(t);
pos = dt(dt > 0);
if isempty(pos) && (isempty(dt_hint) || dt_hint <= 0)
    error('time vector cannot be repaired (no positive dt and no dt_hint).');
end
if ~isempty(dt_hint) && dt_hint > 0
    dt_med = double(dt_hint);
else
    dt_med = median(pos);
end

t_fix = t;
nfix = 0;
for i = 2:numel(t_fix)
    if t_fix(i) <= t_fix(i-1)
        t_fix(i) = t_fix(i-1) + dt_med;
        nfix = nfix + 1;
    end
end

if ~isempty(logger)
    logger('[IMU time repair] fixed %d non-monotonic steps using dt=%0.6fs (orig %0.6f->%0.6f, fixed %0.6f->%0.6f)\n', ...
        nfix, dt_med, t(1), t(end), t_fix(1), t_fix(end));
end
dt_used = dt_med;
end
