function data_filt = basic_butterworth_filter(data, cutoff, fs, order)
%BASIC_BUTTERWORTH_FILTER Low-pass Butterworth filtering without toolboxes.
%   data_filt = BASIC_BUTTERWORTH_FILTER(data, cutoff, fs, order) applies a
%   zero-phase Butterworth filter to each column of DATA. Coefficients are
%   computed directly using pole/zero formulas and a bilinear transform. This
%   is intended as a lightweight fallback when the Signal Processing Toolbox
%   functions BUTTER or FILTFILT are unavailable.
%
%   Parameters
%   ----------
%   data   - NxM array of data to filter (samples in rows)
%   cutoff - Cut-off frequency in Hz (default 5.0)
%   fs     - Sampling frequency in Hz (default 400)
%   order  - Filter order (default 4)
%
%   Returns
%   -------
%   data_filt - Filtered data of the same size as DATA.
%
%   This mirrors the butter_lowpass_filter implementation in Python.

    if nargin < 4 || isempty(order);   order = 4;   end
    if nargin < 3 || isempty(fs);      fs = 400;   end
    if nargin < 2 || isempty(cutoff);  cutoff = 5.0; end

    % Pre-warp for bilinear transform
    warped = 2*fs * tan(pi * cutoff / fs);
    k = 0:(order-1);
    angles = pi/2 + (2*k+1) * pi / (2*order);
    poles_analog = warped * exp(1i*angles);
    poles_digital = (2*fs + poles_analog) ./ (2*fs - poles_analog);
    zeros_digital = -ones(1, order);
    gain_analog = warped^order;
    gain_digital = real(gain_analog ./ prod(2*fs - poles_analog));
    b = real(poly(zeros_digital)) * gain_digital;
    a = real(poly(poles_digital));

    % Zero-phase filtering via forward and reverse filtering
    y = filter(b, a, data);
    data_filt = flipud(filter(b, a, flipud(y)));
end
