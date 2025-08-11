function t = read_truth_state(truth_path)
%READ_TRUTH_STATE Returns time vector from STATE_X001.txt
% Handles lines starting with '#', multiple spaces/tabs, and headers.
% The second numeric column is treated as time in seconds.
    opts = detectImportOptions(truth_path, 'FileType','text', ...
        'Delimiter', {'\t',' ',';','|',','}, ...
        'CommentStyle','#', 'ConsecutiveDelimitersRule','join');
    T = readtable(truth_path, opts);
    numCols = find(varfun(@isnumeric,T,'OutputFormat','uniform'));
    if numel(numCols) < 2
        error('Not enough numeric columns in truth file to extract time.');
    end
    % Column 1 is the sample count; column 2 is time in seconds.
    t = T{:, numCols(2)};
    t = t(~isnan(t));
    t = t(:);
end

