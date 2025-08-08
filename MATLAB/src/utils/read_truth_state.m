function truth = read_truth_state(truth_path)
%READ_TRUTH_STATE Robust parser for STATE_* truth files.
%   TRUTH = READ_TRUTH_STATE(PATH) reads a whitespace-delimited text file
%   containing time [s] followed by position NED [m] and velocity NED [m/s].
%   Lines beginning with '#' and blank lines are ignored.

fid = fopen(truth_path,'r');
if fid < 0
    error('Truth state file not found: %s', truth_path);
end
C = textscan(fid, '%s', 'Delimiter','\n', 'Whitespace','');
fclose(fid);
lines = C{1};
keep = true(size(lines));
for i = 1:numel(lines)
    s = strtrim(lines{i});
    if isempty(s) || startsWith(s, '#')
        keep(i) = false;
    end
end
lines = lines(keep);
if isempty(lines)
    error('Truth file has no data rows after removing comments: %s', truth_path);
end
tmp = cellfun(@(s) str2num(s), lines, 'UniformOutput', false); %#ok<ST2NM>
M = cell2mat(tmp);
if size(M,2) < 7
    error('Truth file must have at least 7 columns.');
end
t = M(:,1);
pos_ned = M(:,2:4);
vel_ned = M(:,5:7);
truth = struct('t', t, 'pos_ned', pos_ned, 'vel_ned', vel_ned);
end
