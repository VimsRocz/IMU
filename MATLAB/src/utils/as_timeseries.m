function C = as_timeseries(C, N)
%AS_TIMESERIES Broadcast a 3x3 attitude to N-by-3-by-3.
%   C = AS_TIMESERIES(C, N) ensures C has size N-by-3-by-3 by broadcasting
%   a single 3x3 matrix to all time steps. Throws an error for unsupported
%   shapes.

    if ndims(C) == 3 && size(C,1) == N
        return;
    elseif isequal(size(C), [3 3])
        C = repmat(C, 1, 1, N);
        C = permute(C, [3 1 2]);
    else
        error('Unexpected attitude shape for timeseries');
    end
end
