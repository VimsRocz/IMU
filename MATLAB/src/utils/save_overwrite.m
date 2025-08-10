function save_overwrite(fname, varargin)
%SAVE_OVERWRITE Save variables from caller workspace after fresh write.
%   SAVE_OVERWRITE(FNAME, VAR1, VAR2, ...) deletes any existing file at
%   FNAME before saving and then saves the variables named VAR1, VAR2, ...
%   from the CALLER workspace. Intermediate directories are created.

folder = fileparts(fname);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end
if exist(fname, 'file')
    delete(fname);
end

% Collect variables from caller into a struct for safe saving
S = struct();
for i = 1:numel(varargin)
    varname = varargin{i};
    try
        S.(varname) = evalin('caller', varname);
    catch
        error('save_overwrite:VarNotFound', 'Variable ''%s'' not found in caller workspace.', varname);
    end
end
save(fname, '-struct', 'S');
end
