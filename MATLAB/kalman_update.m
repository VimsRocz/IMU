function [x, P] = kalman_update(x, P, z, H, R, Q)
%KALMAN_UPDATE  Numerically stable EKF measurement update.
%
%   [x, P] = KALMAN_UPDATE(x, P, z, H, R, Q) performs a measurement update
%   using innovation z - H*x and measurement covariance R. The optional
%   process noise Q is checked for positive-definiteness. The function
%   preserves symmetry of P, regularises ill-conditioned innovation
%   covariance S, and uses a Cholesky-based gain computation. The Joseph
%   form is used for the covariance update.
%
%   Inputs:
%       x  - prior state estimate
%       P  - prior state covariance (NxN)
%       z  - measurement vector
%       H  - measurement Jacobian (MxN)
%       R  - measurement noise covariance (MxM)
%       Q  - process noise covariance (NxN), optional
%
%   Example:
%       N = 15;          % state dimension
%       M = 6;           % measurement dimension
%       x = zeros(N,1);
%       P = eye(N);
%       z = zeros(M,1);
%       H = randn(M,N);
%       R = eye(M);
%       Q = eye(N);      % process noise
%       [x,P] = kalman_update(x,P,z,H,R,Q);
%
%   Author: Auto-generated

    if nargin < 6
        Q = [];
    end

    %--- sanity checks -----------------------------------------------------
    if ~isequal(R, R') || any(eig(R) <= 0)
        error('Measurement noise R must be symmetric positive-definite');
    end
    if ~isempty(Q) && (~isequal(Q, Q') || any(eig(Q) <= 0))
        error('Process noise Q must be symmetric positive-definite');
    end

    %--- innovation --------------------------------------------------------
    y = z - H*x;
    S = H*P*H' + R;
    S = (S+S')/2;                % enforce symmetry

    %--- regularise if S is ill-conditioned -------------------------------
    if isnan(rcond(S)) || rcond(S) < 1e-12
        S = S + eps*eye(size(S));
    end

    %--- Innovation gating (99% chi-square) -------------------------------
    M = size(S,1);
    do_update = true;
    try
        d2 = y' * (S \ y);
    catch
        d2 = y' * pinv(S) * y;
    end
    try
        gate = chi2inv(0.99, M);
    catch
        % Fallback common values (approximate)
        if M >= 6
            gate = 16.812;  % 6-DoF @99%
        elseif M >= 3
            gate = 11.345;  % 3-DoF @99%
        else
            gate = 6.635;   % 1-2 DoF conservative fallback
        end
    end
    if isfinite(d2) && d2 > gate
        do_update = false;
    end

    %--- Cholesky-based gain computation ---------------------------------
    [L,p] = chol(S,'lower');
    if p > 0
        % S not PD even after regularisation: use pseudo-inverse
        K = P*H'/pinv(S);
    else
        % Solve K = (P*H')/S using two triangular solves
        K = (P*H')/L'/L;
    end

    %--- state update -----------------------------------------------------
    if do_update
        x = x + K*y;
    end

    %--- Joseph-form covariance update -----------------------------------
    I_KH = eye(size(P)) - K*H;
    if do_update
        P = I_KH*P*I_KH' + K*R*K';
    end
    P = (P+P')/2;   % enforce symmetry
end
