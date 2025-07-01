%% RUN_TRIAD_ONLY  Run all datasets using the TRIAD method and validate results
% This MATLAB script mirrors run_triad_only.py. It calls the Python batch
% processor with the TRIAD method and then validates the generated MAT files
% against the reference STATE_X*.txt logs when available.
%
% Usage:
%   run_triad_only
%
% All .mat files are written to the 'results' folder in the repository root.
% When a matching STATE_<id>.txt exists the script invokes
% validate_with_truth.py for the dataset.

here = fileparts(mfilename('fullpath'));

%% -- Run the Python batch processor ---------------------------------------
py_script = fullfile(here, 'run_all_datasets.py');
cmd = sprintf('python "%s" --method TRIAD', py_script);
status = system(cmd);
if status ~= 0
    error('run_all_datasets.py failed');
end

%% -- Validate each result when ground truth is available ------------------
results_dir = fullfile(here, 'results');
mat_files = dir(fullfile(results_dir, '*_TRIAD_kf_output.mat'));

for k = 1:numel(mat_files)
    name = mat_files(k).name;
    tokens = regexp(name, '^IMU_(X\d+)_.*_TRIAD_kf_output\.mat$', 'tokens');
    if isempty(tokens)
        continue
    end
    ds = tokens{1}{1};
    truth_file = fullfile(here, ['STATE_' ds '.txt']);
    if ~isfile(truth_file)
        continue
    end
    validate_py = fullfile(here, 'validate_with_truth.py');
    vcmd = sprintf('python "%s" --est-file "%s" --truth-file "%s" --output "%s"', ...
        validate_py, fullfile(results_dir, name), truth_file, results_dir);
    vstatus = system(vcmd);
    if vstatus ~= 0
        warning('Validation failed for %s', name);
    end
end
