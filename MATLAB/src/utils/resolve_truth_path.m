function truth_path = resolve_truth_path()
%RESOLVE_TRUTH_PATH Canonicalize the truth file path.
%   truth_path = resolve_truth_path()
%       Returns the path string to the truth file if found, or an empty char
%       if it does not exist.

    preferred = '/Users/vimalchawda/Desktop/IMU/STATE_X001.txt';

    if isfile(preferred)
        truth_path = preferred;
        fprintf('Using TRUTH: %s\n', truth_path);
        return;
    end

    truth_path = '';
end
