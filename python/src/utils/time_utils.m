function varargout = time_utils(varargin)
%TIME_UTILS Dispatch wrapper for time-related utilities.
%   This file exists for parity; external callers should use
%   ``ensure_unique_increasing`` directly.  Calling TIME_UTILS(...) will
%   forward to ``ensure_unique_increasing``.
[varargout{1:nargout}] = ensure_unique_increasing(varargin{:});
end
