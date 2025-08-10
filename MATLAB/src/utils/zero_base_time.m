function t0 = zero_base_time(t)
%ZERO_BASE_TIME Return t - t(1) as double column vector.
%   Handles datetime arrays and leading NaNs. Raises an error if the input
%   is empty or all NaN.
%
%   t0 = ZERO_BASE_TIME(t)
%
if isempty(t)
    error('zero_base_time:Empty','Input time vector is empty');
end

if isdatetime(t)
    t = posixtime(t);
elseif isduration(t)
    t = seconds(t);
end

% Ensure column double
t = double(t(:));
mask = ~isnan(t);
if ~any(mask)
    error('zero_base_time:AllNaN','Input time vector contains only NaNs');
end

% Drop leading NaNs
first = find(mask,1,'first');
t = t(first:end);
mask = ~isnan(t);
t = t(mask);

% Shift to zero
t0 = t - t(1);
end
