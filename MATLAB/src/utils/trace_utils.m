function try_task(taskName, taskFunc, varargin)
%TRY_TASK Execute a task with logging and error capture.
%   Usage:
%       try_task('Task3', @Task_3, arg1, arg2)

    log_msg(sprintf('▶ START %s', taskName));
    try
        feval(taskFunc, varargin{:});
        log_msg(sprintf('✓ DONE %s', taskName));
    catch ME
        log_msg(sprintf('[ERROR] %s failed: %s', taskName, ME.message));
        log_msg(getReport(ME, 'extended', 'hyperlinks', 'off'));
        dump_vars_matlab(sprintf('%s_locals.mat', taskName));
        % Continue execution
    end
end

function dump_vars_matlab(filename)
%DUMP_VARS_MATLAB Save caller workspace variables for debugging.
    vars = evalin('caller', 'whos');
    S = struct();
    for k = 1:length(vars)
        try
            S.(vars(k).name) = evalin('caller', vars(k).name);
        catch
            % skip variables that cannot be read
        end
    end
    save(fullfile('MATLAB','results',filename), '-struct', 'S');
    log_msg(sprintf('[DUMP] Saved variables to %s', filename));
end

function log_msg(msg)
%LOG_MSG Write a debug message when DEBUG is enabled.
    persistent debug_mode log_path
    if isempty(debug_mode)
        env = getenv('DEBUG');
        debug_mode = any(strcmpi(env, {'1','true','yes'}));
        log_dir = fullfile('MATLAB','results');
        if exist(log_dir, 'dir') ~= 7
            mkdir(log_dir);
        end
        log_path = fullfile(log_dir, 'debug_log.txt');
    end
    if ~debug_mode
        return
    end
    ts = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    line = sprintf('[%s] %s', ts, msg);
    fprintf('%s\n', line);
    fid = fopen(log_path, 'a');
    if fid ~= -1
        fprintf(fid, '%s\n', line);
        fclose(fid);
    end
end
