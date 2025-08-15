function varargout = Task_1(varargin)
% Unified Task 1 entry point. Delegates to original implementation.
% Keeps compatibility while we consolidate files.
if nargout
    [varargout{1:nargout}] = MATLAB.Task_1.Task_1(varargin{:});
else
    MATLAB.Task_1.Task_1(varargin{:});
end
end

