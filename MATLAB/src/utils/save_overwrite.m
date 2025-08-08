function save_overwrite(fname, varargin)
%SAVE_OVERWRITE Save variables ensuring the file is freshly written.
%   SAVE_OVERWRITE(FNAME, VAR1, VAR2, ...) deletes any existing file at
%   FNAME before saving. Intermediate directories are created if needed.

folder = fileparts(fname);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end
if exist(fname, 'file')
    delete(fname);
end
save(fname, varargin{:});
end
