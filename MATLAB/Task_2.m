function varargout = Task_2(varargin)
% Unified Task 2 entry point. Delegates to original implementation.
if nargout
    [varargout{1:nargout}] = MATLAB.Task_2.Task_2(varargin{:});
else
    MATLAB.Task_2.Task_2(varargin{:});
end
end

