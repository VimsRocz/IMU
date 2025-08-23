function [rb, rms_err, inlier_frac] = estimate_lever_arm_rb(Rwb, v_gnss_w, v_state_w, omega_b, varargin)
%ESTIMATE_LEVER_ARM_RB Estimate IMU->GNSS lever arm r_b (body frame) from velocities.
%   [rb, rms, inliers] = ESTIMATE_LEVER_ARM_RB(Rwb, v_gnss_w, v_state_w, omega_b, ...)
%     Rwb: (3,3,N) rotation body->world at each epoch
%     v_gnss_w, v_state_w: (N,3) velocities (world)
%     omega_b: (N,3) body angular rate [rad/s]
%   Name/Value:
%     'SpeedMin', 2.0        minimum speed [m/s]
%     'OmegaMinDps', 2.0     minimum body rate [deg/s]
%     'HuberDelta', 0.5      robust scale [m/s]
%     'L2Reg', 1e-6          Tikhonov regularizer
%     'MaxIter', 5           IRLS iterations

    p = inputParser;
    addParameter(p, 'SpeedMin', 2.0);
    addParameter(p, 'OmegaMinDps', 2.0);
    addParameter(p, 'HuberDelta', 0.5);
    addParameter(p, 'L2Reg', 1e-6);
    addParameter(p, 'MaxIter', 5);
    parse(p, varargin{:});
    spdMin = p.Results.SpeedMin;
    omgMin = p.Results.OmegaMinDps;
    hub = p.Results.HuberDelta;
    l2 = p.Results.L2Reg;
    iters = p.Results.MaxIter;

    N = size(v_gnss_w,1);
    if ndims(Rwb) == 3
        assert(size(Rwb,1)==3 && size(Rwb,2)==3 && size(Rwb,3)==N, 'Rwb must be 3x3xN');
    else
        error('Rwb must be 3x3xN');
    end

    speed = vecnorm(v_gnss_w,2,2);
    omega_dps = vecnorm(omega_b,2,2) * 180/pi;
    sel = (speed >= spdMin) & (omega_dps >= omgMin);
    if ~any(sel)
        rb = zeros(3,1); rms_err = NaN; inlier_frac = 0.0; return;
    end

    M = nnz(sel);
    A = zeros(3*M,3); b = zeros(3*M,1);
    ii = 1;
    for k = find(sel).'
        R = Rwb(:,:,k);
        w = omega_b(k,:).';
        Skew = [  0   -w(3)  w(2);
                 w(3)   0   -w(1);
                -w(2)  w(1)   0  ];
        dv_b = R' * (v_gnss_w(k,:).' - v_state_w(k,:).');
        A(3*(ii-1)+1:3*ii, :) = Skew;
        b(3*(ii-1)+1:3*ii) = dv_b;
        ii = ii + 1;
    end

    % Robust IRLS with Huber weights
    W = ones(size(b)); rb = zeros(3,1);
    for t = 1:iters
        Aw = A .* W; % each row weighted
        H = Aw.'*A + l2*eye(3);
        g = Aw.'*b;
        rb_new = H \ g;
        res = b - A*rb_new;
        absr = abs(res);
        W = ones(size(b));
        idx = absr > hub;
        W(idx) = hub ./ absr(idx);
        rb = rb_new;
    end
    res = b - A*rb;
    rms_err = sqrt(mean(res.^2));
    inlier_frac = mean(abs(res) <= 3*hub);
end

