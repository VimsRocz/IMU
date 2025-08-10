function p = get_data_file(name)
%GET_DATA_FILE Return absolute path for a dataset, searching standard dirs.
% Search order: DATA/IMU, DATA/GNSS, DATA/TRUTH, then repo root.

if isstring(name); name = char(name); end
if isfile(name) && startsWith(name, filesep), p = name; return; end

repo = fileparts(mfilename('fullpath')); repo = fileparts(repo); % repo root
cands = { fullfile(repo,'DATA','IMU',  name), ...
          fullfile(repo,'DATA','GNSS', name), ...
          fullfile(repo,'DATA','TRUTH',name), ...
          fullfile(repo, name) };
for i=1:numel(cands), if isfile(cands{i}), p = cands{i}; return; end, end
error('get_data_file:FileNotFound','Dataset not found: %s', name);
end
