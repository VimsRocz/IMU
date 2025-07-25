function data_filt = basic_butterworth_filter(data, cutoff, fs, order)
%BASIC_BUTTERWORTH_FILTER Low-pass Butterworth filter without toolboxes.
%   data_filt = BASIC_BUTTERWORTH_FILTER(data, cutoff, fs, order) applies a
%   zero-phase Butterworth low-pass filter to the columns of ``data``. The
%   filter coefficients are computed directly from pole/zero formulas and the
%   data is filtered forwards and backwards using ``filter`` and ``flipud``.
%
%   Parameters
%   ----------
%   data   : NxM array
%       Input signal where each column represents a separate channel.
%   cutoff : scalar, optional
%       Cut-off frequency in Hz (default 5).
%   fs     : scalar, optional
%       Sampling frequency in Hz (default 400).
%   order  : scalar, optional
%       Filter order (default 4).
%
%   Returns
%   -------
%   data_filt : NxM array
%       Filtered signal of the same size as ``data``.
%
%   This function mirrors ``butter_lowpass_filter`` in the Python
%   implementation and serves as a fallback when the Signal Processing
%   Toolbox is unavailable.

if nargin < 4 || isempty(order);   order = 4;   end
if nargin < 3 || isempty(fs);      fs = 400;   end
if nargin < 2 || isempty(cutoff);  cutoff = 5.0; end

% Convert row vector input to column for filtering convenience
transposed = isrow(data);
if transposed
    data = data.';
end

% Prewarp the digital frequency and compute analog poles
warped = 4 * tan(pi * cutoff / fs);
m = -order+1:2:order-1;
p = -exp(1i * pi * m / (2*order));
% Scale poles and gain for desired cutoff
p = warped * p;
k = warped^order;

% Bilinear transform (fs normalized to 2)
fs_norm = 2.0;
fs2 = 2.0 * fs_norm;
z = [];

p_d = (fs2 + p) ./ (fs2 - p);
relative_degree = numel(p_d) - numel(z);
z_d = [-ones(1, relative_degree)];

k_d = k * real(prod(fs2 - z) / prod(fs2 - p));

b = real(poly(z_d)) * k_d;
a = real(poly(p_d));

% Forward-backward filtering to approximate filtfilt
fwd = filter(b, a, data);
rev = flipud(fwd);
rev = filter(b, a, rev);
data_filt = flipud(rev);

if transposed
    data_filt = data_filt.';
end
end
