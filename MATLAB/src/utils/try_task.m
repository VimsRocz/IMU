function try_task(taskFunc, taskName, varargin)
%TRY_TASK Execute a task and report errors without stopping execution.

fprintf('\u25B6 Running %s...\n', taskName);
try
    feval(taskFunc, varargin{:});
    fprintf('\u2713 %s completed successfully.\n', taskName);
catch ME
    fprintf('\u274C Error in %s: %s\n', taskName, ME.message);
    if evalin('base','exist(''DEBUG_FLAG'',''var'') && DEBUG_FLAG')
        fprintf('Stack trace:\n');
        for s = 1:numel(ME.stack)
            fprintf('  %s:%d\n', ME.stack(s).file, ME.stack(s).line);
        end
    end
end
end
