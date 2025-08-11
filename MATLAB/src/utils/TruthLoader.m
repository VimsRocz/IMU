function Truth = TruthLoader(path)
%TRUTHLOADER Robustly read whitespace-delimited truth files.
%   TRUTH = TRUTHLOADER(PATH) parses a text or CSV file containing truth
%   data. The loader handles comment lines starting with '#', '%' or '//',
%   normalises variable names, maps standard columns (time, ECEF position,
%   velocity and quaternion), and enforces a monotonic time base.
%
%   The returned struct TRUTH has fields:
%       t_posix   - raw time vector as doubles (seconds)
%       t0        - zero-based time vector (seconds)
%       n         - number of samples
%       pos_ecef  - [N x 3] ECEF positions (m) or []
%       vel_ecef  - [N x 3] ECEF velocities (m/s) or []
%       att_quat_wxyz - [N x 4] quaternions in wxyz order or []
%       notes     - cell array of diagnostic strings
%
%   This mirrors the Python ``truth_loader`` placeholder.

    notes = {};
    opts = detectImportOptions(path, 'FileType','text', ...
        'Delimiter',{' ','\t'}, 'ConsecutiveDelimitersRule','join');
    opts.CommentStyle = {'#','%','//'};
    opts.PreserveVariableNames = true;
    T = readtable(path, opts);
    vn = normalize_varnames(T.Properties.VariableNames);
    T.Properties.VariableNames = cellstr(vn);

    % Map columns
    timeNames = {'time','t','time_s','posix_time','timestamp'};
    tcol = '';
    for k = 1:numel(timeNames)
        if ismember(timeNames{k}, vn)
            tcol = timeNames{k};
            break;
        end
    end
    if isempty(tcol)
        error('TruthLoader:noTime','No time column found');
    end
    t = double(T.(tcol));
    t = t(:);

    % Drop non-finite times
    mask = isfinite(t);
    dropped = sum(~mask);
    if dropped > 0
        notes{end+1} = sprintf('Dropped %d non-finite time rows', dropped);
    end
    t = t(mask); T = T(mask,:);

    % Enforce unique/monotonic time
    [tu, iu] = unique(t,'stable');
    dup = numel(t) - numel(tu);
    if dup > 0
        notes{end+1} = sprintf('Dropped %d duplicate time rows', dup);
    end
    t = tu; T = T(iu,:);

    t0 = zero_base_time(t);
    dt = diff(t0); dt = dt(isfinite(dt) & dt>0);
    if isempty(dt)
        dt_med = NaN; hz_est = NaN;
        notes{end+1} = 'No valid positive dt found';
    else
        dt_med = median(dt); hz_est = 1/dt_med;
        notes{end+1} = sprintf('dt_med=%.6f hz=%.6f', dt_med, hz_est);
    end

    % Position columns
    posCols = {'x_ecef_m','y_ecef_m','z_ecef_m'};
    if all(ismember(posCols, vn))
        pos_ecef = table2array(T(:,posCols));
    else
        pos_ecef = [];
    end

    % Velocity columns
    velCols = {'vx_ecef_mps','vy_ecef_mps','vz_ecef_mps'};
    if all(ismember(velCols, vn))
        vel_ecef = table2array(T(:,velCols));
    else
        vel_ecef = [];
    end

    % Quaternion columns
    quatCols = {'q0','q1','q2','q3'};
    if all(ismember(quatCols, vn))
        q = table2array(T(:,quatCols));
        if max(abs(q(:,1))) < max(abs(q(:,end)))
            % assume xyzw -> reorder to wxyz
            q = q(:, [4 1 2 3]);
        end
        nq = sqrt(sum(q.^2,2));
        q = q ./ nq;
        att_quat_wxyz = q;
    else
        att_quat_wxyz = [];
    end

    Truth = struct('t_posix', t, 't0', t0, 'n', numel(t0), ...
                   'pos_ecef', pos_ecef, 'vel_ecef', vel_ecef, ...
                   'att_quat_wxyz', att_quat_wxyz, 'notes', {notes});
end
