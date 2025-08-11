function log_msg(varargin)
%LOG_MSG Print a formatted debug message when DEBUG is enabled.

DEBUG_FLAG = false;
if evalin('base','exist(''DEBUG_FLAG'',''var'')')
    DEBUG_FLAG = evalin('base','DEBUG_FLAG');
end
if DEBUG_FLAG
    fprintf('[DEBUG] %s\n', sprintf(varargin{:}));
end
end
