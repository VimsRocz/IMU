function Truth = TruthLoader(truthPath, opts)
%TRUTHLOADER Load ground-truth trajectory from various formats.
%   Truth = TruthLoader(truthPath, opts) attempts to load ground truth
%   information from *truthPath*. The function supports ``.mat``,
%   ``.csv``/``.txt``, ``.json`` and gzipped variants of these formats.
%   The returned structure contains:
%       .source_path
%       .format
%       .t_posix             % Nx1 POSIX seconds
%       .t0                  % zero-based time
%       .has_pose            % logical
%       .pos_ecef            % Nx3 or []
%       .vel_ecef            % Nx3 or []
%       .latlonh_deg         % Nx3 or []
%       .att_quat_wxyz       % Nx4 or []
%       .R_b2n               % 3x3xN or []
%       .notes               % cellstr diagnostics
%       .n                   % number of valid rows
%
%   ``opts`` is reserved for future options (currently unused).
%   Diagnostics and decisions taken during loading are appended to
%   ``Truth.notes``.

if nargin < 2 || isempty(opts), opts = struct(); end

Truth = empty_truth();
Truth.source_path = truthPath;

if nargin < 1 || isempty(truthPath)
    error('TruthLoader:MissingPath', 'truthPath not set or empty');
end

% If directory supplied, search for likely truth file
if isfolder(truthPath)
    truthPath = find_truth_file(truthPath);
    Truth.source_path = truthPath;
end

if ~isfile(truthPath)
    error('TruthLoader:MissingFile', 'File not found: %s', truthPath);
end

% Handle gz files
[basePath, ext] = strip_gz(truthPath);
cleanupObj = onCleanup(@() cleanup_temp(basePath, truthPath)); %#ok<NASGU>

Truth.format = lower(strrep(ext, '.', ''));

switch ext
    case '.mat'
        Truth = load_mat(basePath, Truth);
    case {'.csv', '.txt'}
        Truth = load_table_file(basePath, Truth);
    case '.json'
        Truth = load_json_file(basePath, Truth);
    otherwise
        error('TruthLoader:UnknownFormat', 'Unsupported file extension: %s', ext);
end

Truth.n = numel(Truth.t_posix);
if Truth.n > 0
    Truth.t0 = zero_base_time(Truth.t_posix);
end
Truth.has_pose = ~isempty(Truth.pos_ecef) || ~isempty(Truth.latlonh_deg);

end

%% ------------------------------------------------------------------------
function Truth = empty_truth()
Truth = struct('source_path','', 'format','', 't_posix',[], 't0',[], ...
    'has_pose',false, 'pos_ecef',[], 'vel_ecef',[], 'latlonh_deg',[], ...
    'att_quat_wxyz',[], 'R_b2n',[], 'notes',{{}}, 'n',0);
end

function pathOut = find_truth_file(dirPath)
patterns = {'truth*', '*ground*truth*', '*ref*', '*gt*'};
exts = {'.mat','.csv','.txt','.json','.csv.gz','.json.gz','.mat.gz'};
for i = 1:numel(patterns)
    for j = 1:numel(exts)
        d = dir(fullfile(dirPath, [patterns{i} exts{j}]));
        if ~isempty(d)
            pathOut = fullfile(dirPath, d(1).name);
            return;
        end
    end
end
error('TruthLoader:NoFileInDir', 'No truth file found in directory %s', dirPath);
end

function [basePath, ext] = strip_gz(p)
[filepath, name, ext] = fileparts(p);
if strcmpi(ext,'.gz')
    [~, name2, ext2] = fileparts(name);
    tmpDir = tempname; mkdir(tmpDir);
    gunzip(p, tmpDir);
    basePath = fullfile(tmpDir, [name2 ext2]);
    ext = ext2;
else
    basePath = p;
end
end

function cleanup_temp(basePath, original)
[~,~,ext] = fileparts(original);
if strcmpi(ext,'.gz')
    tmpDir = fileparts(basePath);
    if exist(tmpDir,'dir'), rmdir(tmpDir,'s'); end
end
end

%% ---- loaders -----------------------------------------------------------
function Truth = load_mat(path, Truth)
try
    vars = whos('-file', path);
catch ME
    if contains(ME.message,'v7.3')
        mf = matfile(path);
        vars = whos(mf);
    else
        rethrow(ME);
    end
end
var_names = {vars.name};
Truth.notes{end+1} = sprintf('Loaded .mat; variables: %s', strjoin(var_names, ', '));
candidates = {'Truth','truth','Ttruth','T','gt','reference','pose','traj'};
idx = find(ismember(var_names, candidates), 1);
if isempty(idx)
    error('TruthLoader:NoVar', 'No expected variables found in mat file');
end
vname = var_names{idx};
S = load(path, vname);
obj = S.(vname);
Truth.notes{end+1} = sprintf('Using variable ''%s''.', vname);

if istable(obj)
    Truth = parse_table(obj, Truth);
elseif isstruct(obj)
    Truth = parse_struct(obj, Truth);
elseif isnumeric(obj)
    Truth = parse_numeric(obj, Truth);
else
    error('TruthLoader:UnsupportedVarType', 'Variable %s of type %s not supported', vname, class(obj));
end
end

function Truth = load_table_file(path, Truth)
opts = detectImportOptions(path, 'NumHeaderLines',0, 'PreserveVariableNames',true);
T = readtable(path, opts);
Truth.notes{end+1} = sprintf('Loaded table from %s with %d rows.', path, height(T));
Truth = parse_table(T, Truth);
end

function Truth = load_json_file(path, Truth)
raw = fileread(path);
data = jsondecode(raw);
if isstruct(data)
    if isscalar(data)
        fns = fieldnames(data);
        for k=1:numel(fns)
            val = data.(fns{k});
            if istable(val)
                Truth.notes{end+1} = sprintf('Found table field ''%s'' in JSON.', fns{k});
                Truth = parse_table(val, Truth); return;
            elseif isstruct(val)
                Truth = parse_struct(val, Truth); return;
            end
        end
        Truth = parse_struct(data, Truth);
    else
        T = struct2table(data);
        Truth = parse_table(T, Truth);
    end
elseif iscell(data)
    T = struct2table([data{:}]);
    Truth = parse_table(T, Truth);
else
    error('TruthLoader:InvalidJSON','Unsupported JSON content.');
end
end

%% ---- parse helpers ----------------------------------------------------
function Truth = parse_table(T, Truth)
[time_col, time_name] = match_var(T.Properties.VariableNames, ...
    {'Posix_Time','posix_time','time','t','time_s'});
if isempty(time_col)
    error('TruthLoader:NoTime', 'Time column not found; tried Posix_Time,posix_time,time,t,time_s');
end
Truth.notes{end+1} = sprintf('Mapped time from ''%s''.', time_name);
t = T{:, time_col};
Truth.t_posix = coerce_time_to_posix(t, Truth);

x_names = {'X_ECEF_m','x_ecef','ecef_x','x'};
y_names = {'Y_ECEF_m','y_ecef','ecef_y','y'};
z_names = {'Z_ECEF_m','z_ecef','ecef_z','z'};
[x_col, x_nm] = match_var(T.Properties.VariableNames, x_names);
[y_col, y_nm] = match_var(T.Properties.VariableNames, y_names);
[z_col, z_nm] = match_var(T.Properties.VariableNames, z_names);
if ~isempty(x_col) && ~isempty(y_col) && ~isempty(z_col)
    Truth.pos_ecef = [T{:,x_col}, T{:,y_col}, T{:,z_col}];
    Truth.notes{end+1} = sprintf('Found ECEF position columns: %s,%s,%s.', x_nm, y_nm, z_nm);
end

[lat_col, lat_nm] = match_var(T.Properties.VariableNames, {'lat','latitude','Lat'});
[lon_col, lon_nm] = match_var(T.Properties.VariableNames, {'lon','longitude','Lon'});
[h_col, h_nm]     = match_var(T.Properties.VariableNames, {'height','h','alt','Alt'});
if ~isempty(lat_col) && ~isempty(lon_col) && ~isempty(h_col)
    Truth.latlonh_deg = [T{:,lat_col}, T{:,lon_col}, T{:,h_col}];
    Truth.notes{end+1} = sprintf('Found lat/lon/height columns: %s,%s,%s.', lat_nm, lon_nm, h_nm);
end

vx_names = {'VX_ECEF_mps','Vx_ECEF_mps','vx_ecef','vx'};
vy_names = {'VY_ECEF_mps','Vy_ECEF_mps','vy_ecef','vy'};
vz_names = {'VZ_ECEF_mps','Vz_ECEF_mps','vz_ecef','vz'};
[vx_col, vx_nm] = match_var(T.Properties.VariableNames, vx_names);
[vy_col, vy_nm] = match_var(T.Properties.VariableNames, vy_names);
[vz_col, vz_nm] = match_var(T.Properties.VariableNames, vz_names);
if ~isempty(vx_col) && ~isempty(vy_col) && ~isempty(vz_col)
    Truth.vel_ecef = [T{:,vx_col}, T{:,vy_col}, T{:,vz_col}];
    Truth.notes{end+1} = sprintf('Found ECEF velocity columns: %s,%s,%s.', vx_nm, vy_nm, vz_nm);
end

q_names = {'quat_w','qw','q_w','w','quat_x','qx','q_x','x','quat_y','qy','q_y','y','quat_z','qz','q_z','z'};
q_idx = match_vars(T.Properties.VariableNames, q_names, 4);
if ~isempty(q_idx)
    q = T{:, q_idx};
    q = normalize_quat(q);
    Truth.att_quat_wxyz = q;
    Truth.notes{end+1} = sprintf('Found quaternion columns: %s.', strjoin(T.Properties.VariableNames(q_idx), ','));
end
end

function Truth = parse_struct(S, Truth)
fns = fieldnames(S);
T = struct2table(S);
Truth.notes{end+1} = sprintf('Converted struct to table with fields: %s', strjoin(fns, ', '));
Truth = parse_table(T, Truth);
end

function Truth = parse_numeric(A, Truth)
if size(A,2) < 2
    error('TruthLoader:NumericTooFewCols','Numeric array must have at least 2 columns');
end
Truth.notes{end+1} = sprintf('Parsed numeric array with %d rows.', size(A,1));
Truth.t_posix = coerce_time_to_posix(A(:,2), Truth);
if size(A,2) >=5
    Truth.pos_ecef = A(:,3:5);
    Truth.notes{end+1} = 'Assumed columns 3-5 are ECEF position.';
end
if size(A,2) >=8
    Truth.vel_ecef = A(:,6:8);
    Truth.notes{end+1} = 'Assumed columns 6-8 are ECEF velocity.';
end
end

%% ---- utilities --------------------------------------------------------
function [idx, name] = match_var(varnames, candidates)
lower = lower(varnames);
cand = lower(candidates);
idx = [];
name = '';
for k = 1:numel(cand)
    idx = find(strcmp(lower, cand{k}),1);
    if ~isempty(idx)
        name = varnames{idx};
        return;
    end
end
end

function idx = match_vars(varnames, candidates, n)
lower = lower(varnames);
idx = [];
cand_lower = lower(candidates);
cols = zeros(1,n);
for k=1:n
    cols(k) = find(strcmp(lower, cand_lower{k}),1);
end
if all(cols)
    idx = cols;
end
end

function q = normalize_quat(q)
if size(q,2) ~= 4, return; end
norms = sqrt(sum(q.^2,2));
q = q ./ norms;
end

function t = coerce_time_to_posix(t, Truth)
if isdatetime(t)
    t = posixtime(t);
elseif isduration(t)
    t = seconds(t);
else
    t = double(t);
end
t = t(:);
mask = ~isnan(t);
if ~all(mask)
    Truth.notes{end+1} = sprintf('Removed %d NaN time entries.', sum(~mask));
    t = t(mask);
end
[t, ia] = unique(t); %#ok<ASGLU>
if ~issorted(t)
    t = sort(t);
    Truth.notes{end+1} = 'Sorted time to ensure monotonicity.';
end
if max(t) < 1e8
    Truth.notes{end+1} = 'Time appears relative; using zero-based seconds.';
    t = t - t(1);
end
end

