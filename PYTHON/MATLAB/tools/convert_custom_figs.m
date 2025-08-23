function convert_custom_figs(rootDir)
%CONVERT_CUSTOM_FIGS Batch-convert custom MAT-based .fig files to MATLAB .fig
%  convert_custom_figs() scans the default results folder and writes
%  sibling files with the suffix _matlab.fig that open natively in MATLAB.
%
%  convert_custom_figs(rootDir) converts all .fig files under rootDir.
%
%  The input .fig files are MAT-files produced by Python code that saved
%  line data via utils.save_plot_fig/save_plot_mat. This script reconstructs
%  the plots and saves true MATLAB figures alongside them.

if nargin < 1 || isempty(rootDir)
    % Default to the repo's PYTHON/results directory if present; otherwise, CWD/results
    here = fileparts(mfilename('fullpath'));
    cand = fullfile(here, '..', '..', 'PYTHON', 'results');
    if exist(cand, 'dir')
        rootDir = cand;
    else
        rootDir = fullfile(pwd, 'results');
    end
end

files = dir(fullfile(rootDir, '*.fig'));
if isempty(files)
    fprintf('No .fig files found in %s\n', rootDir);
    return;
end

fprintf('Converting %d figure(s) under %s\n', numel(files), rootDir);
for i = 1:numel(files)
    in = fullfile(files(i).folder, files(i).name);
    [~, base, ~] = fileparts(in);
    out = fullfile(files(i).folder, [base, '_matlab.fig']);
    try
        rebuild_custom_fig(in, out);
        fprintf('  OK  %s -> %s\n', files(i).name, [base, '_matlab.fig']);
    catch ME
        fprintf(2, '  ERR %s: %s\n', files(i).name, ME.message);
    end
end

end

