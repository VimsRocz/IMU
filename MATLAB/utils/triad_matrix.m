function T = triad_matrix(v1, v2)
% TRIAD_MATRIX  Build 3x3 orthonormal basis given two non-collinear 3x1 vectors.
%   T = TRIAD_MATRIX(v1, v2) returns a matrix whose columns are an
%   orthonormal basis built from primary vector v1 and secondary vector v2.
%   v1 and v2 must be 3-element vectors.
%
%   Usage:
%       T = triad_matrix(v1, v2)
%
%   The basis vectors are:
%       t1 = v1 / norm(v1)
%       t2 = normalized component of v2 orthogonal to t1
%       t3 = t1 x t2 (right-handed)
%
%   Errors if inputs are near zero or collinear.

    arguments
        v1 (1,3) double
        v2 (1,3) double
    end

    v1 = v1(:); v2 = v2(:);
    assert(numel(v1)==3 && numel(v2)==3, 'triad_matrix: inputs must be 3-vectors');

    n1 = norm(v1);
    n2 = norm(v2);
    if n1 < 1e-12
        error('triad_matrix: primary vector near zero');
    end
    if n2 < 1e-12
        error('triad_matrix: secondary vector near zero');
    end

    t1 = v1 / n1;
    v2p = v2 - (t1.' * v2) * t1;
    n2p = norm(v2p);
    if n2p < 1e-12
        error('triad_matrix: vectors are collinear; cannot build basis');
    end
    t2 = v2p / n2p;
    t3 = cross(t1, t2);
    t3 = t3 / norm(t3);
    if abs(det([t1 t2 t3])) < 1e-6
        error('triad_matrix: degenerate basis (det ~ 0)');
    end
    T = [t1 t2 t3];
end

