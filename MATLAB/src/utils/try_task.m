function try_task(taskFunc, taskName, varargin)
%TRY_TASK Execute a task and report errors without stopping execution.
%   Usage:
%       try_task(@Task_1, 'Task 1', arg1, arg2, ...)
%       or
%       try_task('Task 1', @Task_1, arg1, arg2, ...)
%   Automatically detects swapped arguments and, when debugging is enabled,
%   saves the caller workspace to ``<MATLAB/results>/<taskName>_workspace.mat``
%   upon failure.

% If first two args are swapped
if isa(taskFunc, 'char') && isa(taskName, 'function_handle')
    tmp = taskFunc;
    taskFunc = taskName;
    taskName = tmp;
end

print_task_start(taskName);
try
    feval(taskFunc, varargin{:});
    fprintf('%s %s completed successfully.\n', char(0x2713), taskName);
catch ME
    fprintf('%s Error in %s: %s\n', char(0x274C), taskName, ME.message);
    if evalin('base','exist(''DEBUG_FLAG'',''var'') && DEBUG_FLAG')
        fprintf('Stack trace:\n');
        for s = 1:numel(ME.stack)
            fprintf('  %s:%d\n', ME.stack(s).file, ME.stack(s).line);
        end

        % Dump caller workspace for post-mortem analysis
        vars = evalin('caller', 'whos');
        snap = struct();
        for k = 1:numel(vars)
            name = vars(k).name;
            try
                snap.(name) = evalin('caller', name);
            catch
            end
        end
        outdir = get_matlab_results_dir();
        outfile = fullfile(outdir, sprintf('%s_workspace.mat', taskName));
        save(outfile, '-struct', 'snap');
        fprintf('[DEBUG] Workspace saved to %s\n', outfile);
    end
end
end
