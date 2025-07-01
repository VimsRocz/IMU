function run_triad_only(truthFile)
%RUN_TRIAD_ONLY  Run all datasets using the TRIAD method and validate results
% This MATLAB script mirrors run_triad_only.py. It calls the Python batch
% processor with the TRIAD method and then validates the generated MAT files
% against the reference STATE_X*.txt logs when available.
%
% Usage:
%   run_triad_only
%   run_triad_only(TRUTHFILE)
%
% When TRUTHFILE is provided it is used for all datasets. If omitted the
% script searches for STATE_<id>.txt next to this
% file.
%
% All .mat files are written to the 'results' folder in the repository root.
% When a matching STATE_<id>.txt exists the script invokes
% validate_with_truth.py for the dataset.

if nargin < 1
    truthFile = '';
end

here = fileparts(mfilename('fullpath'));
py_dir = fullfile(here, '..', 'Python');

%% Determine which Python executable to use
py = pyenv;
pyexe = py.Executable;
if isempty(pyexe)
    pyexe = 'python3';  % fallback
end

%% -- Run the Python batch processor ---------------------------------------
py_script = fullfile(py_dir, 'run_all_datasets.py');
cmd = sprintf('"%s" "%s" --method TRIAD', pyexe, py_script);
status = system(cmd);
if status ~= 0
    error('run_all_datasets.py failed');
end

%% -- Validate each result when ground truth is available ------------------
results_dir = fullfile(py_dir, 'results');
mat_files = dir(fullfile(results_dir, '*_TRIAD_kf_output.mat'));

for k = 1:numel(mat_files)
    name = mat_files(k).name;
    tokens = regexp(name, '^IMU_(X\d+)_.*_TRIAD_kf_output\.mat$', 'tokens');
    if isempty(tokens)
        continue
    end
    ds = tokens{1}{1};
    if ~isempty(truthFile)
        candidates = {truthFile};
    else
        data_dir = fullfile(here, '..', 'Data');
        candidates = {fullfile(data_dir, ['STATE_' ds '.txt'])};
    end
    truth_file = '';
    for c = candidates
        if isfile(c{1})
            truth_file = c{1};
            break
        end
    end
    if isempty(truth_file)
        if ~isempty(truthFile)
            fprintf('Warning: reference file %s not found, skipping validation\n', truthFile);
        else
            fprintf('Warning: no truth file for %s, skipping validation\n', ds);
        end
        continue
    end
    validate_py = fullfile(here, 'validate_with_truth.py');
    vcmd = sprintf('"%s" "%s" --est-file "%s" --truth-file "%s" --output "%s"', ...
        pyexe, validate_py, fullfile(results_dir, name), truth_file, results_dir);
    vstatus = system(vcmd);
    if vstatus ~= 0
        warning('Validation failed for %s', name);
    end
end
end
