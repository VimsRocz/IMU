function truth_path = resolve_truth(preferred_path)
%RESOLVE_TRUTH Locate or create the canonical truth file.
%   TRUTH_PATH = RESOLVE_TRUTH(PREFERRED_PATH) ensures that the truth file
%   used by MATLAB tasks is located at the canonical path
%   ``/Users/vimalchawda/Desktop/IMU/STATE_IMU_X001.txt``. If the file is
%   missing but ``STATE_X001.txt`` exists, it is copied into place. The
%   function returns the path to the truth file or an empty string if the
%   file could not be found or created.

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
    truth_path = '';
end

