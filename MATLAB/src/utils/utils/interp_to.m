function Yt = interp_to(t_src, Y_src, t_tgt)
%INTERP_TO 1-D interp of vectors or NxM arrays along rows over time.
%   YT = INTERP_TO(T_SRC, Y_SRC, T_TGT) interpolates the data Y_SRC sampled
%   at times T_SRC to the target times T_TGT. Linear interpolation with
%   extrapolation is used. T_SRC and T_TGT are column vectors in seconds.
%   Y_SRC can be a vector or an N-by-M matrix where N = length(T_SRC).

% Ensure column vectors of type double
t_src = double(t_src(:));
t_tgt = double(t_tgt(:));

% Convert Y_src to 2-D matrix with rows corresponding to time samples
if isvector(Y_src)
    Y_src = Y_src(:);
end

% Preallocate output
Yt = zeros(numel(t_tgt), size(Y_src,2));

% Interpolate each column individually
for j = 1:size(Y_src,2)
    Yt(:,j) = interp1(t_src, Y_src(:,j), t_tgt, 'linear', 'extrap');
end
end
