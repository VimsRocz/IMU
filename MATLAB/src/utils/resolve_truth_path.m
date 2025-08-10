function truth_path = resolve_truth_path()
%RESOLVE_TRUTH_PATH Canonicalize the truth file path.
%   truth_path = resolve_truth_path()
%       Returns the path string to the truth file if found, or an empty char
%       if it does not exist.

    root = fileparts(fileparts(mfilename('fullpath')));
    candidates = {
        fullfile(root, 'DATA', 'TRUTH', 'STATE_X001.txt'),
        fullfile(root, 'DATA', 'TRUTH', 'STATE_X001_small.txt')
    };
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            truth_path = candidates{i};
            fprintf('Using TRUTH: %s\n', truth_path);
            return;
        end
    end
    listing = dir(fullfile(root, 'DATA', 'TRUTH', 'STATE_*.txt'));
    if ~isempty(listing)
        truth_path = fullfile(listing(1).folder, listing(1).name);
        fprintf('Using TRUTH: %s\n', truth_path);
    else
        truth_path = '';
    end
end
