function fix_triad_task5(cfg)
% =======================  START OF “CODEX PROMPT”  =======================
% You are generating MATLAB R2023b+ code ONLY (no Python, no classes, no Live
% Scripts). The environment may be missing some toolboxes. Respect these rules:
%   • Use only built-in MATLAB & Mapping/Stats/NAV functions WHEN available.
%   • If a function is missing (e.g., chi2inv, ecef2ned), fall back to simple,
%     local replacements provided in this file (do not require toolboxes).
%   • Do NOT use external packages or System objects.
% Task: Patch an INS/GNSS TRIAD pipeline’s Task 5 to fix large position/velocity
% errors by addressing 3 issues:
%   (1) Frame/Gravity: Attitude is logged as Body->NED (C_nb). Use quaternion
%       q (wxyz) and rotmat(q,'frame') to get C_bn, then C_nb = C_bn.'.
%       Propagate with specific force f_b via  vdot_n = C_nb*f_b + g_n,
%       where g_n = [0 0 +g] (NED has +Z down).  (NO sign mistakes.)
%   (2) ZUPT: Detect static correctly with ( ||a|| - g ) < epsAcc AND
%       ||gyro|| < epsGyro, with a minimum dwell time. Do NOT compare ||a||
%       directly to a tiny threshold. Expose thresholds in cfg.
%   (3) GNSS usage & gating: Use only GNSS position/velocity (disable 1 Hz
%       “GNSS acceleration”). Build measurement covariance from DOPs if present,
%       else defaults. Gate innovations with chi-square (3 dof); if Stats
%       Toolbox unavailable, use the constant gate value 27.879 for 0.997 prob.
% Additional requirements:
%   • Robust time alignment: convert IMU & GNSS to timetables and use
%     synchronize(...,'regular','linear',TimeStep=dt).
%   • No UI. Save results to cfg.save. Print a tiny summary.
% Inputs (cfg struct with defaults below):
%   cfg.task1: Task 1 MAT (gravity, lat, lon)
%   cfg.t5in : Task 5 MAT (att_quat [Nx4 wxyz], t_est)
%   cfg.imu  : IMU CSV (t, ax,ay,az,gx,gy,gz) or timetable already in MAT
%   cfg.gnss : GNSS CSV or table with either NED pos/vel or ECEF pos/vel+DOP
%   cfg.save : output MAT path
%   ZUPT params: cfg.zupt_acc_eps (0.15), cfg.zupt_gyro_eps (deg2rad(1)),
%                 cfg.zupt_min_dur_s (0.25)
%   Gate prob: cfg.chi2_prob (0.997) and fall back value if chi2inv missing.
% Output MAT must contain t_est, p_n [Nx3], v_n [Nx3], zupt [Nx1 logical].
% ========================  END OF “CODEX PROMPT”  ========================

arguments
    cfg.task1  (1,1) string = "results/Task1_init_IMU_X002_GNSS_X002_TRIAD.mat"
    cfg.t5in   (1,1) string = "results/IMU_X002_GNSS_X002_TRIAD_task5_results.mat"
    cfg.imu    (1,1) string = "DATA/IMU/IMU_X002.csv"
    cfg.gnss   (1,1) string = "DATA/GNSS/GNSS_X002.csv"
    cfg.save   (1,1) string = "results/IMU_X002_GNSS_X002_TRIAD_task5_FIXED.mat"
    cfg.zupt_acc_eps   (1,1) double = 0.15
    cfg.zupt_gyro_eps  (1,1) double = deg2rad(1.0)
    cfg.zupt_min_dur_s (1,1) double = 0.25
    cfg.chi2_prob      (1,1) double = 0.997
end

% --- Load essentials ---
S1  = load(cfg.task1);  % expects: gravity_ned_mps2, lat_deg, lon_deg
S5  = load(cfg.t5in);   % expects: att_quat (Nx4, wxyz), t_est
if isfield(S1,'gravity_ned_mps2')
    g_n = [0 0 S1.gravity_ned_mps2]; % NED +Z down
elseif isfield(S1,'g_NED')
    g_n = S1.g_NED(:).';
else
    g_n = [0 0 9.81];
end
lat0 = S1.lat_deg; lon0 = S1.lon_deg;

% --- IMU timetable ---
imuTT = tryLoadIMU(cfg.imu);                            % timetable: Time, ax..gz
Fs    = 1/seconds(median(diff(imuTT.Time)));            % ~400 Hz
dt    = 1/Fs;

% --- GNSS timetable (NED if present; else convert from ECEF) ---
gnssTT = tryLoadGNSS(cfg.gnss, lat0, lon0);

% --- Time-align IMU & GNSS on uniform IMU grid ---
allTT = synchronize(imuTT, gnssTT, 'regular','linear','TimeStep',seconds(dt)); % [4][5]
% Drop/ignore any GNSS acceleration fields (too noisy at 1 Hz)
dropIfExists(allTT, ["GnssAccN","GnssAccE","GnssAccD"]);

% --- Attitude (Body->NED) from quaternion, fix frames ---
q = quaternion(S5.att_quat,'wxyz');                     % quaternion (wxyz)
C_bn = rotmat(q,"frame");                               % N->B
C_nb = pagetranspose(C_bn);                             % Body->NED

% --- Specific force & propagate with CORRECT gravity sign ---
N   = height(allTT);
f_b = allTT{:, ["ax","ay","az"]};                    % specific force (m/s^2)
v_n = zeros(N,3); p_n = zeros(N,3);

% --- Robust ZUPT (||a||-g and gyro), with dwell ---
zupt = detectZUPT(imuTT, Fs, cfg.zupt_acc_eps, cfg.zupt_gyro_eps, cfg.zupt_min_dur_s, abs(g_n(3)));

% --- GNSS measurement noise from DOPs or defaults ---
if all(ismember(["HDOP","VDOP"], allTT.Properties.VariableNames))
    posSigma = max(3.0, 0.8*allTT.HDOP);               % crude mapping from DOP→σ [m]
else
    posSigma = 3.0*ones(N,1);
end
velSigma = 0.35*ones(N,1);                              % ~0.3–0.5 m/s

% --- Chi-square gate (3 dof) with fallback if Stats Toolbox missing ---
gate3 = chi2gate(3, cfg.chi2_prob);                     % default ~27.879 for 0.997

% --- Main loop (simple INS with ZUPT + GNSS pos/vel updates) ---
zuptCount=0; gated=0;
hasPos = all(ismember(["GnssPosN","GnssPosE","GnssPosD"], allTT.Properties.VariableNames));
hasVel = all(ismember(["GnssVelN","GnssVelE","GnssVelD"], allTT.Properties.VariableNames));
for k = 2:N
    Cnb = C_nb(:,:,min(k,size(C_nb,3)));                % guard if sizes differ
    vdot = (Cnb * f_b(k,:).' + g_n.').';                % key fix
    v_n(k,:) = v_n(k-1,:) + vdot*dt;
    p_n(k,:) = p_n(k-1,:) + v_n(k,:)*dt;

    % ZUPT: clamp velocity during detected rest
    if zupt(k)
        v_n(k,:) = [0 0 0];
        zuptCount = zuptCount + 1;
    end

    % GNSS position update with χ² gate
    if hasPos && all(~ismissing(allTT{k,["GnssPosN","GnssPosE","GnssPosD"]}))
        z  = allTT{k,["GnssPosN","GnssPosE","GnssPosD"]}.';
        h  = p_n(k,:).';
        Rk = diag((posSigma(k)*[1 1 1]).^2);
        nu = z - h;
        d2 = nu.'*(Rk\nu);
        if d2 <= gate3, p_n(k,:) = (h+nu).'; else, gated=gated+1; end
    end

    % GNSS velocity update with χ² gate
    if hasVel && all(~ismissing(allTT{k,["GnssVelN","GnssVelE","GnssVelD"]}))
        z  = allTT{k,["GnssVelN","GnssVelE","GnssVelD"]}.';
        h  = v_n(k,:).';
        Rk = diag((velSigma(k)*[1 1 1]).^2);
        nu = z - h;
        d2 = nu.'*(Rk\nu);
        if d2 <= gate3, v_n(k,:) = (h+nu).'; else, gated=gated+1; end
    end
end

t_est = seconds(allTT.Time - allTT.Time(1));
save(cfg.save, 't_est','p_n','v_n','zupt','gate3','cfg');
fprintf('[fix_triad_task5] DONE  N=%d  Fs=%.1f Hz  ZUPT=%d  gated=%d  -> %s\n', ...
    N, Fs, zuptCount, gated, cfg.save);

% ============================== helpers ==============================
function TT = tryLoadIMU(f)
    if endsWith(f,".mat",IgnoreCase=true)
        S = load(f);
        if isfield(S,'imuTT'); TT = S.imuTT; return; end
        if isfield(S,'imu');   TT = S.imu;   return; end
    end
    M = readmatrix(f);                          % [t ax ay az gx gy gz]
    T = seconds(M(:,1)-M(1,1));
    TT = timetable(T, M(:,2),M(:,3),M(:,4), M(:,5),M(:,6),M(:,7), ...
        'VariableNames', {'ax','ay','az','gx','gy','gz'});
end

function TT = tryLoadGNSS(f, lat0, lon0)
    if endsWith(f,".mat",IgnoreCase=true)
        S = load(f);
        TT = firstGnssTimetable(S);
        if ~isempty(TT), TT = ensureNED(TT, lat0, lon0); return; end
    end
    T = readtable(f);
    t = pickTime(T);
    vars = T.Properties.VariableNames;

    % If NED available, use it
    if all(ismember({'GnssPosN','GnssPosE','GnssPosD','GnssVelN','GnssVelE','GnssVelD'},vars))
        TT = timetable(t, T.GnssPosN,T.GnssPosE,T.GnssPosD, T.GnssVelN,T.GnssVelE,T.GnssVelD, ...
            pickVar(T,'HDOP'), pickVar(T,'VDOP'), ...
            'VariableNames', {'GnssPosN','GnssPosE','GnssPosD','GnssVelN','GnssVelE','GnssVelD','HDOP','VDOP'});
        return
    end
    % Else convert from ECEF (fallback implementation if Mapping TBX missing)
    if all(ismember({'X_ECEF_m','Y_ECEF_m','Z_ECEF_m'},vars))
        [n,e,d] = ecef2ned_local(T.X_ECEF_m, T.Y_ECEF_m, T.Z_ECEF_m, lat0, lon0, 0);
        % If ECEF velocity present, rotate to NED
        if all(ismember({'VX_ECEF_mps','VY_ECEF_mps','VZ_ECEF_mps'},vars))
            [vn,ve,vd] = ecef2nedv_local(T.VX_ECEF_mps,T.VY_ECEF_mps,T.VZ_ECEF_mps, lat0, lon0);
        else
            vn=nan(size(n)); ve=vn; vd=vn;
        end
        TT = timetable(t, n,e,d, vn,ve,vd, pickVar(T,'HDOP'), pickVar(T,'VDOP'), ...
            'VariableNames', {'GnssPosN','GnssPosE','GnssPosD','GnssVelN','GnssVelE','GnssVelD','HDOP','VDOP'});
        return
    end
    error('tryLoadGNSS: Unsupported GNSS table format.');
end

function v = pickVar(T,name)
    v = []; if ismember(name, T.Properties.VariableNames), v = T.(name); end
end

function TT = firstGnssTimetable(S)
    TT = [];
    fns = fieldnames(S);
    for i=1:numel(fns)
        if istimetable(S.(fns{i})), TT = S.(fns{i}); return; end
    end
end

function t = pickTime(T)
    if ismember('Posix_Time',T.Properties.VariableNames)
        t = seconds(T.Posix_Time - T.Posix_Time(1));
    elseif ismember('t',T.Properties.VariableNames)
        t = seconds(T.t - T.t(1));
    else
        error('No time column found in GNSS table.');
    end
end

function TT = ensureNED(TT, lat0, lon0)
    hasPos = all(ismember({'GnssPosN','GnssPosE','GnssPosD'},TT.Properties.VariableNames));
    hasECEF = all(ismember({'X_ECEF_m','Y_ECEF_m','Z_ECEF_m'},TT.Properties.VariableNames));
    if ~hasPos && hasECEF
        [n,e,d] = ecef2ned_local(TT.X_ECEF_m, TT.Y_ECEF_m, TT.Z_ECEF_m, lat0, lon0, 0);
        TT.GnssPosN=n; TT.GnssPosE=e; TT.GnssPosD=d;
    end
    hasVel = all(ismember({'GnssVelN','GnssVelE','GnssVelD'},TT.Properties.VariableNames));
    hasECEFv = all(ismember({'VX_ECEF_mps','VY_ECEF_mps','VZ_ECEF_mps'},TT.Properties.VariableNames));
    if ~hasVel && hasECEFv
        [vn,ve,vd] = ecef2nedv_local(TT.VX_ECEF_mps, TT.VY_ECEF_mps, TT.VZ_ECEF_mps, lat0, lon0);
        TT.GnssVelN=vn; TT.GnssVelE=ve; TT.GnssVelD=vd;
    end
end

function dropIfExists(TT, names)
    for k = 1:numel(names)
        if ismember(names(k), TT.Properties.VariableNames)
            TT.(names(k)) = [];
        end
    end
end

function zupt = detectZUPT(imuTT, Fs, epsAcc, epsGyro, minDur, gScalar)
% Robust ZUPT: (||a||-g) < epsAcc AND ||gyro|| < epsGyro with dwell
    a = imuTT{:, ["ax","ay","az"]};
    w = imuTT{:, ["gx","gy","gz"]};
    aerr = abs(vecnorm(a,2,2) - abs(gScalar));
    raw  = (aerr < epsAcc) & (vecnorm(w,2,2) < epsGyro);
    K = max(1, round(minDur*Fs));
    zupt = movsum(raw, K) == K;
end

function gate = chi2gate(dof, prob)
% Try chi2inv if present; else use constants for dof=3
    gate = 27.879; % default for dof=3, prob≈0.997 (fallback)
    if exist('chi2inv','file') == 2
        gate = chi2inv(prob, dof);
    end
end

% ---------------------- ECEF<->NED fallbacks ----------------------
function [n,e,d] = ecef2ned_local(X,Y,Z,lat0,lon0,h0)
% Prefer Mapping TBX if available
    if exist('ecef2ned','file')==2
        [n,e,d] = ecef2ned(X,Y,Z,lat0,lon0,h0,wgs84ellipsoid_local());
        return
    end
    % Minimal local version using rotation at origin (no ellipsoid scaling)
    [x0,y0,z0] = geodetic2ecef_local(lat0,lon0,h0);
    dx=X-x0; dy=Y-y0; dz=Z-z0;
    R = nedCfromEcef(lat0,lon0);
    ned = R*[dx(:) dy(:) dz(:)].';
    n=reshape(ned(1,:),size(X)); e=reshape(ned(2,:),size(X)); d=reshape(ned(3,:),size(X));
end

function [vn,ve,vd] = ecef2nedv_local(U,V,W,lat0,lon0)
    if exist('ecef2nedv','file')==2
        [vn,ve,vd] = ecef2nedv(U,V,W,lat0,lon0); return
    end
    R = nedCfromEcef(lat0,lon0);
    ned = R*[U(:) V(:) W(:)].';
    vn=reshape(ned(1,:),size(U)); ve=reshape(ned(2,:),size(U)); vd=reshape(ned(3,:),size(U));
end

function E = wgs84ellipsoid_local()
    if exist('wgs84Ellipsoid','file')==2
        E = wgs84Ellipsoid('meter');
    else
        E = struct('SemiMajorAxis', 6378137, 'Flattening', 1/298.257223563);
    end
end

function [x,y,z] = geodetic2ecef_local(lat,lon,h)
    a=6378137; f=1/298.257223563; e2=f*(2-f);
    lat=deg2rad(lat); lon=deg2rad(lon);
    N = a./sqrt(1 - e2.*sin(lat).^2);
    x = (N+h).*cos(lat).*cos(lon);
    y = (N+h).*cos(lat).*sin(lon);
    z = (N.*(1-e2)+h).*sin(lat);
end

function R = nedCfromEcef(lat,lon)
% Rotation: ECEF -> NED at (lat,lon)  (NED axes: +N, +E, +D)
    lat=deg2rad(lat); lon=deg2rad(lon);
    sL=sin(lat); cL=cos(lat); sA=sin(lon); cA=cos(lon);
    R = [-sL*cA, -sL*sA,  cL;
          -sA,     cA,    0;
         -cL*cA, -cL*sA, -sL];
end

% ========================== end helpers ==========================
end

