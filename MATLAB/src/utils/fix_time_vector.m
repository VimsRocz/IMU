function [t_fix, notes] = fix_time_vector(t_raw, dt_hint)
%FIX_TIME_VECTOR Enforce finite, strictly-increasing time vector.
%   [T_FIX, NOTES] = FIX_TIME_VECTOR(T_RAW, DT_HINT) drops nonfinite and
%   duplicate entries from T_RAW, reconstructing the vector if necessary to
%   ensure a monotonically increasing sequence. DT_HINT is an approximate
%   sample interval used as a fallback when reconstruction is required.
%
%   Notes describing any cleanup actions are returned in the string array
%   NOTES.
%
%   This utility mirrors the logic of ``fix_time_vector`` in the Python
%   timeline utilities.
%
%   Inputs:
%       t_raw   - raw time vector (column or row)
%       dt_hint - approximate sample interval
%
%   Outputs:
%       t_fix   - cleaned time vector
%       notes   - string array of notes describing corrections
%
%   Example:
%       [t, notes] = fix_time_vector(t_raw, 1/400);
%
%   See also: PRINT_TIMELINE_MATLAB, TIMELINE_MATLAB

    notes = strings(0,1);
    t = double(t_raw(:));

    % Drop nonfinite entries
    mask = isfinite(t);
    drop_nf = nnz(~mask);
    if drop_nf > 0
        notes(end+1) = sprintf('dropped nonfinite=%d', drop_nf); %#ok<AGROW>
    end
    t = t(mask);

    % Zero-base if values appear absolute
    if ~isempty(t) && max(t) > 1000
        t = t - t(1);
    end

    % Sort and de-duplicate
    [t, idx] = sort(t);
    dups = [false; diff(t) == 0];
    n_dups = nnz(dups);
    if n_dups > 0
        t = t(~dups);
        notes(end+1) = sprintf('dropped duplicates=%d', n_dups); %#ok<AGROW>
    end

    % Reconstruct if non-increasing
    if any(diff(t) <= 0)
        if ~isempty(t)
            dt = median(diff(t));
            if ~isfinite(dt) || dt <= 0
                dt = dt_hint;
            end
            t = (0:numel(t)-1)' * dt;
            notes(end+1) = sprintf('reconstructed time with dt=%g', dt); %#ok<AGROW>
        end
    end

    % Fallback if still too short
    if numel(t) < 3
        N = numel(t_raw);
        if N > 0
            t = (0:N-1)' * dt_hint;
            notes(end+1) = sprintf('reconstructed from length only, dt=%g', dt_hint); %#ok<AGROW>
        else
            t = zeros(0,1);
        end
    end

    % Ensure column vector starting at zero
    if ~isempty(t)
        t = t - t(1);
    end
    t_fix = t;
end
