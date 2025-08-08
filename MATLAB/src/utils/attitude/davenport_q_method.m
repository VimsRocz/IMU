function C_bn = davenport_q_method(n, b, w)
%DAVENPORT_Q_METHOD Wahba via Davenport's q-method, body->NED.
%   C_bn = DAVENPORT_Q_METHOD(n, b) solves Wahba's problem for the rotation
%   matrix ``C_bn`` given 3xK reference vectors ``n`` in NED and their
%   corresponding body-frame measurements ``b``. Optional weights ``w`` can
%   be supplied as a 1xK vector; if omitted each pair is weighted equally.

if nargin < 3 || isempty(w)
    w = ones(1,size(n,2));
end

B = zeros(3,3);
for k = 1:size(n,2)
    B = B + w(k) * (b(:,k) * n(:,k).');  % body->ref (NED)
end
S = B + B.';
sigma = trace(B);
Z = [ B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1) ];

K = [ sigma,    Z.';
      Z,   S - sigma*eye(3) ];

[vec,~] = eigs(K,1,'largestreal');
q0 = vec(1); qv = vec(2:4);

% quaternion to DCM (body->NED), q = [q0; qx;qy;qz]
qx = qv(1); qy = qv(2); qz = qv(3);
C_bn = [ q0^2+qx^2-qy^2-qz^2,   2*(qx*qy - q0*qz),     2*(qx*qz + q0*qy);
         2*(qy*qx + q0*qz),     q0^2-qx^2+qy^2-qz^2,   2*(qy*qz - q0*qx);
         2*(qz*qx - q0*qy),     2*(qz*qy + q0*qx),     q0^2-qx^2-qy^2+qz^2 ];
end
