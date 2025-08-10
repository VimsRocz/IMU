function [t_u, Y_u, info] = ensure_unique_increasing(label, t, Y)
%ENSURE_UNIQUE_INCREASING  Make time strictly increasing and unique.
%   [t_u, Y_u, info] = ENSURE_UNIQUE_INCREASING(LABEL, t, Y) sanitizes the
%   time vector t and optionally associated array Y so that time stamps are
%   finite, sorted, and strictly increasing. Duplicate times are removed
%   while preserving the first occurrence. A struct INFO summarises the
%   number of removed samples and whether the result is monotonic.
%
%   Usage:
%       [t_u, Y_u, info] = ensure_unique_increasing('IMU t', t, Y);
%       [t_u, ~,  info]  = ensure_unique_increasing('GNSS t', t);
%
%   Notes:
%   - Removes NaN/Inf times
%   - Sorts by time
%   - Deduplicates ties (keeps first)
%   - Returns counts in info
%
%   This utility mirrors the behaviour of the Python counterpart in
%   ``src/utils/ensure_unique_increasing.py``.

if nargin < 3 || isempty(Y), Y = []; end

% enforce column vector for time
t = t(:);
finiteMask = isfinite(t);
if ~all(finiteMask)
    t = t(finiteMask);
    if ~isempty(Y), Y = Y(finiteMask,:); end
end

% sort by time
[ts, ord] = sort(t);
if ~isempty(Y), Y = Y(ord,:); end

% unique by time (stable after sort)
[tu, ia] = unique(ts, 'stable');
Yu = [];
if ~isempty(Y), Yu = Y(ia,:); end

% report and sanity
info = struct();
info.removed_nonfinite = sum(~finiteMask);
info.removed_duplicates = numel(ts) - numel(tu);
info.monotonic = all(diff(tu) > 0);

if ~info.monotonic
    % force strict monotonicity by removing any non-positive steps
    stepMask = [true; diff(tu) > 0];
    tu = tu(stepMask);
    if ~isempty(Yu), Yu = Yu(stepMask,:); end
end

if info.removed_nonfinite || info.removed_duplicates || ~info.monotonic
    fprintf('[time-fix] %s: drop nonfinite=%d, dups=%d, monotonic=%d\n', ...
        label, info.removed_nonfinite, info.removed_duplicates, info.monotonic);
end

t_u = tu; Y_u = Yu;
end
