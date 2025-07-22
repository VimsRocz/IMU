%% RUN_METHOD_ONLY  Run all datasets with a chosen initialisation method
% Mirrors run_method_only.py. The script sets up Python using pyenv and
% invokes the Python helper with the desired method.
%
% Usage:
%   run_method_only                % defaults to 'TRIAD'
%   run_method_only('SVD')
%
% All results are written to the 'results' folder.

function run_method_only(method)
if nargin < 1 || isempty(method)
    method = 'TRIAD';
end

here = fileparts(mfilename('fullpath'));
root = fileparts(here);

py = pyenv;
if py.Status == "NotLoaded"
    try
        py = pyenv("Version", "python3");
    catch
        try
            py = pyenv("Version", "python");
        catch
            fprintf(['No usable Python interpreter found. Install Python 3 and ' ...
                'configure pyenv to point to your installation.\n']);
            return;
        end
    end
end

py_script = fullfile(root, 'src', 'run_method_only.py');
cmd = sprintf('"%s" "%s" --method %s', py.Executable, py_script, method);
status = system(cmd);
if status ~= 0
    error('run_method_only.py failed');
end
end
