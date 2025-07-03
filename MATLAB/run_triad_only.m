%% RUN_TRIAD_ONLY  Run all datasets using the TRIAD method and validate results
% This MATLAB script mirrors run_triad_only.py. It calls the Python batch
% processor with the TRIAD method and then validates the generated MAT files
% against the reference STATE_X*.txt logs when available.
%
% The script attempts to locate a Python interpreter using ``pyenv``.  If
% Python is not loaded it tries ``python3`` then ``python``.  When no
% interpreter can be found a helpful message is printed.
%
% Usage:
%   run_triad_only
%
% All .mat files are written to the 'results' folder in the repository root.
% When a matching STATE_<id>.txt exists the script invokes
% validate_with_truth.py for the dataset.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);

% Initialise the Python environment. Try python3 then python if not loaded.
py = pyenv;
if py.Status == "NotLoaded"
    try
        py = pyenv("Version", "python3");
    catch
        try
            py = pyenv("Version", "python");
        catch
            fprintf('No usable Python interpreter found. Install Python 3 and configure pyenv to point to your installation.\n');
            return;
        end
    end
end

%% -- Run the Python batch processor ---------------------------------------
py_script = fullfile(root, 'src', 'run_all_datasets.py');
cmd = sprintf('"%s" "%s" --method TRIAD', py.Executable, py_script);
status = system(cmd);
if status ~= 0
    error('run_all_datasets.py failed');
end

%% -- Validate each result when ground truth is available ------------------
results_dir = fullfile(pwd, 'results');
mat_files = dir(fullfile(results_dir, '*_TRIAD_kf_output.mat'));

for k = 1:numel(mat_files)
    name = mat_files(k).name;
    tokens = regexp(name, '^IMU_(X\d+)_.*_TRIAD_kf_output\.mat$', 'tokens');
    if isempty(tokens)
        continue
    end
    ds = tokens{1}{1};
    truth_file = fullfile(root, ['STATE_' ds '.txt']);
    if ~isfile(truth_file)
        continue
    end
    validate_py = fullfile(root, 'src', 'validate_with_truth.py');
    first = readmatrix(truth_file, 'CommentStyle', '#');
    r0 = first(1,3:5);
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(r0(1), r0(2), r0(3));
    vcmd = sprintf(['"%s" "%s" --est-file "%s" --truth-file "%s" --output "%s" ' ...
        '--ref-lat %.8f --ref-lon %.8f --ref-r0 %.3f %.3f %.3f'], ...
        py.Executable, validate_py, fullfile(results_dir, name), truth_file, results_dir, ...
        lat_deg, lon_deg, r0(1), r0(2), r0(3));
    vstatus = system(vcmd);
    if vstatus ~= 0
        warning('Validation failed for %s', name);
    end
end
