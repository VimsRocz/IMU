function t0 = zero_base_time(t)
%ZERO_BASE_TIME Make time start at zero, double precision column vector.
%   T0 = ZERO_BASE_TIME(T) returns a column vector of type double where the
%   first element is zero and subsequent elements are relative to the
%   initial time in seconds. If T is empty, the function returns it
%   unchanged.

if isempty(t), t0 = t; return; end

% Ensure column vector and double precision
t = double(t(:));

% Shift so that time starts at zero
t0 = t - t(1);
end
