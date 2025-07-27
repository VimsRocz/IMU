function run_all_datasets_with_python()
%RUN_ALL_DATASETS_WITH_PYTHON  Run the Python batch pipeline from MATLAB
%   Checks for a Python interpreter via pyenv and executes
%   src/run_all_datasets.py. After completion each generated
%   *_kf_output.mat file is loaded and passed to plot_results to
%   reproduce the standard figures in MATLAB.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);

py = pyenv;
if py.Status == "NotLoaded"
    try
        py = pyenv("Version","python3");
    catch
        try
            py = pyenv("Version","python");
        catch
            error(["No usable Python interpreter found. " ...
                   "Install Python 3 and configure pyenv."]);
        end
    end
end

pyScript = fullfile(root,'src','run_all_datasets.py');
cmd = sprintf('"%s" "%s"', py.Executable, pyScript);
status = system(cmd);
if status ~= 0
    error('run_all_datasets.py failed');
end

resultsDir = fullfile(root,'MATLAB/results');
matFiles = dir(fullfile(resultsDir,'*_kf_output.mat'));
for k = 1:numel(matFiles)
    try
        plot_results(fullfile(resultsDir, matFiles(k).name));
    catch ME
        warning('Plotting failed for %s: %s', matFiles(k).name, ME.message);
    end
end

fprintf('Python pipeline complete.\n');
end
