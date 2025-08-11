classdef TaskIO
    %TASKIO Simple helper for saving/loading task structs.
    %   Each task stores its results in a single MAT-file containing a
    %   struct named after the task (e.g. ``Task1``). Subsequent tasks load
    %   these files using the same field name. This mirrors the Python
    %   placeholder ``TaskIO``.
    methods (Static)
        function path = save(taskName, S, outpath)
            if nargin < 3 || isempty(outpath)
                outdir = fullfile(pwd,'MATLAB','results');
                filename = sprintf('%s_results.mat', taskName);
            else
                if isfolder(outpath)
                    outdir = outpath;
                    filename = sprintf('%s_results.mat', taskName);
                else
                    [outdir,filename,ext] = fileparts(outpath);
                    if isempty(ext), ext = '.mat'; end
                    if isempty(outdir), outdir = pwd; end
                    filename = [filename, ext];
                end
            end
            if ~exist(outdir,'dir'), mkdir(outdir); end
            path = fullfile(outdir, filename);
            P = struct(); P.(taskName) = S;
            save(path,'-struct','P','-v7.3');
            fprintf('[TaskIO] Saved %s -> %s (fields: %s)\n', ...
                taskName, path, strjoin(fieldnames(S),', '));
        end
        function S = load(taskName, path)
            L = load(path);
            if isfield(L, taskName)
                S = L.(taskName);
            else
                fn = fieldnames(L);
                if numel(fn)==1
                    S = L.(fn{1});
                else
                    error('TaskIO:missingField','%s not found in %s. Has: %s', ...
                        taskName, path, strjoin(fn,', '));
                end
            end
        end
    end
end
