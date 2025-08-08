function p = project_paths()
%PROJECT_PATHS Adds src/utils once; ensures MATLAB/results exists.
    here = fileparts(mfilename('fullpath'));
    cands = { fullfile(here,'src','utils'), ...
              fullfile(fileparts(here),'src','utils'), ...
              fullfile(here,'utils') };
    added = false;
    for k = 1:numel(cands)
        if exist(cands{k},'dir')
            if ~contains(path, cands{k})
                addpath(cands{k});
            end
            added = true; break;
        end
    end
    if ~added
        warning('utils folder not found near %s', here);
    end
    p.root = fileparts(here);
    p.matlab_results = fullfile(p.root,'MATLAB','results');
    if ~exist(p.matlab_results,'dir'), mkdir(p.matlab_results); end
end

