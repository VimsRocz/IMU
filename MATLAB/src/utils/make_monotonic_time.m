function [t, meta] = make_monotonic_time(t_like, fallback_len, dt, imu_rate_hint)
% MAKE_MONOTONIC_TIME Stub matching Python _make_monotonic_time.
%   Builds a synthetic or unmodified time vector. Currently returns the
%   input unmodified and records minimal metadata.
%
% Usage:
%   [t, meta] = make_monotonic_time(t_like, fallback_len, dt, imu_rate_hint)
%
% Inputs:
%   t_like        - Time-like vector or empty
%   fallback_len  - Length used when synthesising time vector
%   dt            - Sampling interval
%   imu_rate_hint - IMU sample rate hint
%
% Outputs:
%   t    - Time vector
%   meta - Struct with metadata fields

if nargin < 2, fallback_len = []; end
if nargin < 3, dt = []; end
if nargin < 4, imu_rate_hint = []; end

if isempty(t_like)
    if isempty(dt)
        t = [];
    else
        n = fallback_len;
        t = (0:n-1)' * dt;
    end
    meta = struct('source', 'synth', 'wraps', 0, 'dt', dt);
    return;
end

t = t_like(:);
meta = struct('source', 'file', 'wraps', 0);
end
