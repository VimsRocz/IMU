function C_bn = svd_wahba(n, b, w)
%SVD_WAHBA Wahba solution via SVD, body->NED.
%   C_bn = SVD_WAHBA(n, b) returns the direction cosine matrix using a
%   singular value decomposition approach given 3xK reference vectors ``n``
%   in NED and corresponding body-frame measurements ``b``.  Optional weights
%   ``w`` may be supplied as a 1xK vector.

if nargin < 3 || isempty(w)
    w = ones(1,size(n,2));
end

H = zeros(3,3);
for k = 1:size(n,2)
    H = H + w(k) * (b(:,k) * n(:,k).');  % body->ref (NED)
end
[U,~,V] = svd(H);
D = diag([1,1,det(U*V')]);
C_bn = U * D * V.';
end
