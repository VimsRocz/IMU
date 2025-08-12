function set_debug(val)
%SET_DEBUG Enable or disable debug logging.
%   SET_DEBUG(true) enables debug messages. SET_DEBUG(false) disables them.
%   Calling without arguments toggles the current state.

persistent DEBUG_FLAG

if nargin > 0
    DEBUG_FLAG = logical(val);
elseif isempty(DEBUG_FLAG)
    DEBUG_FLAG = false;
else
    DEBUG_FLAG = ~DEBUG_FLAG;
end

assignin('base','DEBUG_FLAG',DEBUG_FLAG);

if DEBUG_FLAG
    fprintf('[DEBUG] Debug logging enabled\n');
else
    fprintf('[DEBUG] Debug logging disabled\n');
end
end
