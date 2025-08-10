function truth_path = resolve_truth(preferred_path)
%RESOLVE_TRUTH Locate the canonical truth file.
%   TRUTH_PATH = RESOLVE_TRUTH(PREFERRED_PATH) ensures that the truth file
%   used by MATLAB tasks is located at the canonical path
%   ``/Users/vimalchawda/Desktop/IMU/STATE_X001.txt``. The function returns
%   the path to the truth file or an empty string if the file could not be
%   found.

    if nargin==0 || isempty(preferred_path)
        preferred_path = '/Users/vimalchawda/Desktop/IMU/STATE_X001.txt';
    end

    if isfile(preferred_path)
        fprintf('Using TRUTH: %s\n', preferred_path);
        truth_path = preferred_path;
        return;
    end

    warning('Truth file not found at preferred path.');
    truth_path = '';
end

