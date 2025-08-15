function varargout = Task_3(varargin)
% Unified Task 3 entry point. Delegates to original implementation.
if nargout
    [varargout{1:nargout}] = MATLAB.Task_3.Task_3(varargin{:});
else
    MATLAB.Task_3.Task_3(varargin{:});
end
end

