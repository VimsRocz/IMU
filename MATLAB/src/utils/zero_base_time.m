function t0 = zero_base_time(t)
%ZERO_BASE_TIME Shift time vector to start at zero.
%   t0 = ZERO_BASE_TIME(t) subtracts the first element of t from all entries.

% Ensure column vector and double precision
 t = double(t(:));
 t0 = t - t(1);
end
