function trace_utils_boot()
%TRACE_UTILS_BOOT Auto-bootstrap tracing infrastructure once per session.
%   Called automatically when any logging function executes.  Ensures the
%   ``MATLAB/results`` directory exists and prints a message when debugging
%   is enabled via the ``DEBUG`` environment variable.

    persistent BOOTED
    if ~isempty(BOOTED), return; end
    BOOTED = true;
    logdir = fullfile('MATLAB','results');
    if ~exist(logdir,'dir'), mkdir(logdir); end
    if is_debug()
        log_msg('[DEBUG] MATLAB tracing enabled');
    end
end

function set_debug(val)
%SET_DEBUG Enable or disable debug logging.
%   SET_DEBUG(TRUE) turns on logging; SET_DEBUG(FALSE) turns it off.  When
%   called without arguments it simply initialises internal state.

    persistent DEBUG_FLAG;
    if nargin > 0
        DEBUG_FLAG = logical(val);
    elseif isempty(DEBUG_FLAG)
        DEBUG_FLAG = false;
    end
    assignin('base','DEBUG_FLAG',DEBUG_FLAG);
    trace_utils_boot();
end

function out = is_debug()
%IS_DEBUG True when debug logging is enabled.
    persistent DEBUG_FLAG;
    if isempty(DEBUG_FLAG), DEBUG_FLAG = false; end
    out = DEBUG_FLAG;
end

function log_msg(msg)
%LOG_MSG Write a timestamped message to console and log file when DEBUG.
    if ~is_debug(), return; end
    trace_utils_boot();
    logfile = fullfile('MATLAB','results','debug_log.txt');
    ts = datestr(now,'yyyy-mm-dd HH:MM:SS');
    line = sprintf('[%s] %s', ts, msg);
    fprintf('%s\n', line);
    fid = fopen(logfile,'a'); if fid>0, fprintf(fid,'%s\n',line); fclose(fid); end
end

function try_task(taskName, fhandle, varargin)
%TRY_TASK Run a task with logging, structure dump and error resilience.

    trace_utils_boot();
    log_msg(repmat('=',1,80));
    log_msg(sprintf('%s START %s', char(0x25B6), taskName));
    try
        feval(fhandle, varargin{:});
        log_msg(sprintf('%s DONE %s', char(0x2713), taskName));
    catch ME
        log_msg(sprintf('[ERROR] %s failed: %s', taskName, ME.message));
        log_msg(getReport(ME, 'extended', 'hyperlinks','off'));
        dump_caller_locals(sprintf('%s_locals.mat', taskName));
    end
    log_msg(repmat('=',1,80));
end

function dump_caller_locals(filename)
%DUMP_CALLER_LOCALS Save caller workspace variables and log their structure.

    trace_utils_boot();
    vars = evalin('caller','whos');
    S = struct();
    for k=1:numel(vars)
        name = vars(k).name;
        try
            S.(name) = evalin('caller', name);
        catch
            % skip
        end
    end
    save(fullfile('MATLAB','results', filename), '-struct','S');
    log_msg(sprintf('[DUMP] saved %s', filename));
    for k=1:numel(vars)
        name = vars(k).name;
        try
            obj = evalin('caller', name);
            dump_structure(name, obj, 0, 4);
        catch
        end
    end
end

function dump_structure(name, obj, depth, maxdepth)
%DUMP_STRUCTURE Recursively log the structure of ``obj``.

    if nargin<3, depth=0; end
    if nargin<4, maxdepth=5; end
    if depth>maxdepth, return; end
    indent = repmat('  ',1,depth);
    try
        if isnumeric(obj)
            sz = size(obj);
            log_msg(sprintf('%s%s: numeric size=%s', indent, name, mat2str(sz)));
        elseif islogical(obj)
            sz = size(obj);
            log_msg(sprintf('%s%s: logical size=%s', indent, name, mat2str(sz)));
        elseif ischar(obj) || isstring(obj)
            log_msg(sprintf('%s%s: char/string value="%s"', indent, name, string(obj)));
        elseif istable(obj)
            log_msg(sprintf('%s%s: table %dx%d', indent, name, height(obj), width(obj)));
        elseif isstruct(obj)
            f = fieldnames(obj);
            log_msg(sprintf('%s%s: struct fields=%s', indent, name, strjoin(f,', ')));
            fn = f(1:min(numel(f),20));
            for i=1:numel(fn)
                try
                    dump_structure(sprintf('%s.%s',name,fn{i}), obj.(fn{i}), depth+1, maxdepth);
                catch
                end
            end
        elseif iscell(obj)
            s = size(obj);
            log_msg(sprintf('%s%s: cell size=%s', indent, name, mat2str(s)));
            n = min(numel(obj), 10);
            for i=1:n
                try
                    dump_structure(sprintf('%s{%d}', name, i), obj{i}, depth+1, maxdepth);
                catch
                end
            end
        else
            log_msg(sprintf('%s%s: class=%s', indent, name, class(obj)));
        end
    catch ME
        log_msg(sprintf('%s%s: <dump failed: %s>', indent, name, ME.message));
    end
end

function save_plot_interactive(fig, outbase, formats)
%SAVE_PLOT_INTERACTIVE Save ``fig`` in multiple formats without closing it.

    trace_utils_boot();
    if nargin<1 || isempty(fig) || ~ishandle(fig)
        fig = gcf;
    end
    if nargin<3 || isempty(formats)
        formats = {'.png','.fig'};
    end
    outdir = fileparts(outbase);
    if ~isempty(outdir) && ~exist(outdir,'dir'), mkdir(outdir); end
    if ~ishandle(fig)
        log_msg(sprintf('[WARN] no valid figure for %s', outbase));
        return;
    end
    for i=1:numel(formats)
        ext = lower(formats{i});
        switch ext
            case '.fig', savefig(fig, [outbase '.fig']);
            otherwise,  exportgraphics(fig, [outbase ext], 'Resolution', 300);
        end
    end
    figure(fig);
end

