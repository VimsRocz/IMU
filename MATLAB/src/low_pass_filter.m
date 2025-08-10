function data_out = low_pass_filter(data_in, cutoff_freq, fs)
%LOW_PASS_FILTER Zero-phase Butterworth low-pass filter.
%   DATA_OUT = LOW_PASS_FILTER(DATA_IN, CUTOFF_FREQ, FS) applies a 4th order
%   Butterworth filter with cut-off frequency CUTOFF_FREQ (Hz) to DATA_IN
%   sampled at FS (Hz). ``filtfilt`` is used when available so that the
%   output has zero phase distortion.
%
%   Example:
%       data_f = low_pass_filter(acc, 10, 100);
%
%   This is a lightweight helper mirroring ``butter_lowpass_filter`` in the
%   Python code base.

    if nargin < 3 || isempty(fs)
        fs = 100; % default sampling rate (Hz)
    end
    if nargin < 2 || isempty(cutoff_freq)
        cutoff_freq = 10; % default cut-off frequency (Hz)
    end
    order = 4;
    nyq = 0.5 * fs;
    normal_cutoff = cutoff_freq / nyq;

    if exist('butter','file') == 2 && exist('filtfilt','file') == 2
        [b,a] = butter(order, normal_cutoff, 'low');
        data_out = filtfilt(b, a, data_in);
    elseif exist('basic_butterworth_filter','file') == 2
        data_out = basic_butterworth_filter(data_in, cutoff_freq, fs, order);
    else
        % Simple moving average as last resort
        win = max(1, round(fs * (1/cutoff_freq)));
        data_out = movmean(data_in, win, 1, 'Endpoints','shrink');
    end
end
