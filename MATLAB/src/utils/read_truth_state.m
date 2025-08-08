function t = read_truth_state(truth_path)
%READ_TRUTH_STATE Returns time vector from STATE_IMU_X001.txt
% Handles lines starting with '#', multiple spaces/tabs, and headers.
    opts = detectImportOptions(truth_path, 'FileType','text', ...
        'Delimiter', {'\t',' ',';','|',','}, ...
        'CommentStyle','#', 'ConsecutiveDelimitersRule','join');
    T = readtable(truth_path, opts);
    firstNumCol = find(varfun(@isnumeric,T,'OutputFormat','uniform'), 1, 'first');
    if isempty(firstNumCol)
        error('No numeric column in truth file to use as time.');
    end
    t = T{:, firstNumCol};
    t = t(~isnan(t));
    t = t(:);
end

