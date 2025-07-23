function vel = derive_velocity(time_s, pos, window_length, polyorder)
%DERIVE_VELOCITY Estimate velocity from position measurements.
%
% Usage:
%   vel = derive_velocity(time_s, pos, window_length, polyorder)
%
% Inputs
%   time_s       - Nx1 vector of time stamps [s]
%   pos          - Nx3 matrix of positions [m] (ECEF or NED)
%   window_length - (optional) odd window length for smoothing
%   polyorder     - (optional) polynomial order for smoothing
%
% Output
%   vel          - Nx3 matrix of velocity [m/s]
%
% The implementation mirrors ``src/velocity_utils.py::derive_velocity``.
% The position is smoothed with a Savitzky-Golay filter and then
% differentiated using a central difference scheme.

if nargin < 3
    window_length = 11;
end
if nargin < 4
    polyorder = 2;
end

if mod(window_length, 2) == 0
    window_length = window_length + 1;
end

time_s = time_s(:);
N = numel(time_s);
if size(pos, 1) ~= N
    error('time_s and pos must have the same number of rows');
end

half = floor(window_length / 2);
pos_sm = zeros(size(pos));

coeff = savgol_coeffs(window_length, polyorder);
for col = 1:size(pos, 2)
    pos_sm(:, col) = conv(pos(:, col), coeff, 'same');
    % fit polynomial to edges like scipy.signal.savgol_filter(mode="interp")
    p_left = polyfit(0:window_length-1, pos(1:window_length, col).', polyorder);
    for k = 1:half
        pos_sm(k, col) = polyval(p_left, k - 1);
    end
    p_right = polyfit(0:window_length-1, pos(end-window_length+1:end, col).', polyorder);
    for k = 1:half
        pos_sm(end-half+k, col) = polyval(p_right, window_length - half + k - 1);
    end
end

vel = zeros(size(pos_sm));
dt = diff(time_s);
vel(2:end-1, :) = (pos_sm(3:end, :) - pos_sm(1:end-2, :)) ./ (dt(2:end) + dt(1:end-1));
vel(1, :) = (pos_sm(2, :) - pos_sm(1, :)) / dt(1);
vel(end, :) = (pos_sm(end, :) - pos_sm(end-1, :)) / dt(end);
end

function coeff = savgol_coeffs(window_length, polyorder)
    if polyorder >= window_length
        error('polyorder must be less than window_length');
    end
    half = floor(window_length / 2);
    if mod(window_length, 2) == 0
        pos = half - 0.5;
    else
        pos = half;
    end
    x = (-pos:window_length - pos - 1);
    x = fliplr(x);
    A = zeros(polyorder + 1, window_length);
    for k = 0:polyorder
        A(k + 1, :) = x .^ k;
    end
    y = zeros(polyorder + 1, 1);
    y(1) = 1;
    coeff = pinv(A) * y;
    coeff = coeff(:).';
end
