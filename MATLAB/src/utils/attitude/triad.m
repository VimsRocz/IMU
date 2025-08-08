function C_bn = triad(n1, n2, b1, b2)
%TRIAD Body->NED DCM from two non-collinear vector pairs.
%   C_bn = TRIAD(n1, n2, b1, b2) returns the direction cosine matrix
%   rotating vectors from the body frame to NED given reference vectors
%   ``n1``, ``n2`` in NED and their measurements ``b1``, ``b2`` in the body
%   frame.
%
%   All inputs are 3x1 vectors and must be non-collinear.

tn1 = n1 / norm(n1);
tb1 = b1 / norm(b1);

tn2 = n2 - (tn1.'*n2)*tn1; tn2 = tn2 / norm(tn2);
tb2 = b2 - (tb1.'*b2)*tb1; tb2 = tb2 / norm(tb2);

tn3 = cross(tn1, tn2);
tb3 = cross(tb1, tb2);

M_n = [tn1 tn2 tn3];
M_b = [tb1 tb2 tb3];

C_bn = M_n * M_b.';  % body->NED
end
