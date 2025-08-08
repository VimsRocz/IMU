function truth_path = resolve_truth(preferred_path)
% Always use /Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt
% If missing but /Users/vimalchawda/Desktop/IMU/STATE_X001.txt exists,
% copy it and use the preferred path.

    if nargin==0 || isempty(preferred_path)
        preferred_path = '/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt';
    end
    fallback = '/Users/vimalchawda/Desktop/IMU/STATE_X001.txt';

    if isfile(preferred_path)
        fprintf('Using TRUTH: %s\n', preferred_path);
        truth_path = preferred_path;
        return;
    end

    if isfile(fallback)
        try
            fprintf('Truth missing at preferred path; copying %s -> %s\n', fallback, preferred_path);
            copyfile(fallback, preferred_path);
            truth_path = preferred_path;
            fprintf('Using TRUTH: %s\n', truth_path);
            return;
        catch ME
            warning('Failed to copy truth file: %s', ME.message);
        end
    end

    warning('Truth file not found at preferred or fallback paths.');
    truth_path = '';  % allow pipeline to continue; Task 6/7 will no-op gracefully
end
