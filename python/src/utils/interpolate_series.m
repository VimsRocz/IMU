function out = interpolate_series(t_ref, t_data, series)
%INTERPOLATE_SERIES Interpolate data to a reference time vector.
%   OUT = INTERPOLATE_SERIES(T_REF, T_DATA, SERIES) returns SERIES
%   interpolated to the timestamps T_REF using linear interpolation.
%   Values outside the range of T_DATA are extrapolated.

out = interp1(t_data, series, t_ref, 'linear', 'extrap');
end
