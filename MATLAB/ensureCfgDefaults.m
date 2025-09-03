function cfg = ensureCfgDefaults(cfg)
%ENSURECFGDEFAULTS Fill Task_5 configuration with safe defaults.
%   cfg = ENSURECFGDEFAULTS(cfgIn) ensures required nested fields exist
%   with conservative defaults so Task_5 does not error on missing fields.

    if nargin == 0 || isempty(cfg)
        cfg = struct();
    end

    % ZUPT defaults expected by Task_5:
    %   - speed_thresh_mps
    %   - acc_movstd_thresh
    %   - min_pre_lift_s
    if ~isfield(cfg, 'zupt') || ~isstruct(cfg.zupt)
        cfg.zupt = struct();
    end
    if ~isfield(cfg.zupt, 'speed_thresh_mps') || isempty(cfg.zupt.speed_thresh_mps)
        cfg.zupt.speed_thresh_mps = 0.30;  % consistent with run_triad_only defaults
    end
    if ~isfield(cfg.zupt, 'acc_movstd_thresh') || isempty(cfg.zupt.acc_movstd_thresh)
        cfg.zupt.acc_movstd_thresh = 0.15; % [m/s^2] moving-std accel threshold
    end
    if ~isfield(cfg.zupt, 'min_pre_lift_s') || isempty(cfg.zupt.min_pre_lift_s)
        cfg.zupt.min_pre_lift_s = 5.0;     % [s] minimum pre-liftoff window
    end

    % Provide a GNSS gating default container if absent (used for chi^2 gate)
    if ~isfield(cfg, 'gnss') || ~isstruct(cfg.gnss)
        cfg.gnss = struct();
    end
    if ~isfield(cfg.gnss, 'reject_maha_prob') || isempty(cfg.gnss.reject_maha_prob)
        cfg.gnss.reject_maha_prob = 0.999;
    end
end

