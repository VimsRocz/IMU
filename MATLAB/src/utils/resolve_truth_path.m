function truth_path = resolve_truth_path()
%RESOLVE_TRUTH_PATH Canonicalize the truth file path.
%   Picks a preferred path; if missing but an alternate exists, copies the
%   alternate into the preferred location so future runs are stable.
%
%   truth_path = resolve_truth_path()
%       Returns the path string to the truth file if found or copied, or an
%       empty char if neither exists.

    preferred = '/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt';
    alternate = '/Users/vimalchawda/Desktop/IMU/STATE_X001.txt';

    if isfile(preferred)
        truth_path = preferred;
        fprintf('Using TRUTH: %s\n', truth_path);
        return;
    end

    if isfile(alternate)
        try
            fprintf('Truth missing at preferred path; copying %s -> %s\n', alternate, preferred);
            copyfile(alternate, preferred, 'f');   % force overwrite
            truth_path = preferred;
            fprintf('Using TRUTH: %s\n', truth_path);
            return;
        catch ME
            warning('Copy to preferred failed (%s). Falling back to alternate.', ME.message);
            truth_path = alternate;
            return;
        end
    end

    truth_path = '';
end
