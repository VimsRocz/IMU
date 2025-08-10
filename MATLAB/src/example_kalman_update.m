% Example usage of kalman_update with dummy dimensions
N = 15;  % state dimension (e.g. INS states)
M = 6;   % GNSS position + velocity measurement
x = zeros(N,1);
P = eye(N);
z = zeros(M,1);
H = randn(M,N);
R = eye(M);
Q = eye(N);
[x, P] = kalman_update(x, P, z, H, R, Q);
