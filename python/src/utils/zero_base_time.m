function t0 = zero_base_time(t)
%ZERO_BASE_TIME Shift time vector to start at zero.
%   Handles datetime inputs and leading NaNs. Errors if empty or all NaN.

if isempty(t)
    error('zero_base_time:Empty','Input time vector is empty');
end
if isdatetime(t)
    t = posixtime(t);
elseif isduration(t)
    t = seconds(t);
end
t = double(t(:));
mask = ~isnan(t);
if ~any(mask)
    error('zero_base_time:AllNaN','Input time vector contains only NaNs');
end
first = find(mask,1,'first');
t = t(first:end);
mask = ~isnan(t);
t = t(mask);
t0 = t - t(1);
end
