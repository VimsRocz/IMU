import numpy as np
from scipy.signal import butter, filtfilt, correlate


def _butter_bandpass(x, fs, f1=0.02, f2=2.0, order=3):
    b, a = butter(order, [f1/(fs/2), f2/(fs/2)], btype='band')
    return filtfilt(b, a, x)


def estimate_dt_seconds(
    t_imu, wz_imu,                      # IMU time, yaw-rate (rad/s)
    t_gnss, vx_gnss, vy_gnss,           # GNSS time, horizontal vel in world (ENU/NED)
    windows_s=((0, None),),             # list of (t0,t1) windows to analyze
    fs_ref=20.0,                        # resample freq for correlation
    lag_range_s=2.0,                    # +/- seconds
):
    """Estimate time shift between IMU and GNSS via yaw-rate correlation."""
    # Build GNSS heading rate
    psi = np.unwrap(np.arctan2(vy_gnss, vx_gnss))
    dpsi_dt = np.gradient(psi, t_gnss)

    # Resample both onto common grid
    t0 = max(t_imu[0], t_gnss[0])
    t1 = min(t_imu[-1], t_gnss[-1])
    if t1 <= t0:
        return 0.0
    tt = np.arange(t0, t1, 1.0/fs_ref)
    wz_i = np.interp(tt, t_imu, wz_imu)
    dpsi = np.interp(tt, t_gnss, dpsi_dt)

    # Band-limit
    wz_i = _butter_bandpass(wz_i, fs_ref)
    dpsi = _butter_bandpass(dpsi, fs_ref)

    # Evaluate per-window lags, take median
    max_lag = int(lag_range_s * fs_ref)
    lags_found = []
    for w0, w1 in windows_s:
        i0 = 0 if w0 in (None, 0) else np.searchsorted(tt, tt[0] + w0)
        i1 = len(tt) if (w1 is None) else np.searchsorted(tt, tt[0] + w1)
        a = wz_i[i0:i1] - np.mean(wz_i[i0:i1])
        b = dpsi[i0:i1] - np.mean(dpsi[i0:i1])
        if len(a) < 10 or len(b) < 10:
            continue
        xcorr = correlate(a, b, mode='full')
        lags = np.arange(-len(a)+1, len(b))
        # restrict lag window
        sel = (lags >= -max_lag) & (lags <= max_lag)
        lags = lags[sel]
        xcorr = xcorr[sel]
        lag_samples = lags[np.argmax(np.abs(xcorr))]
        lags_found.append(lag_samples / fs_ref)

    if not lags_found:
        return 0.0
    return float(np.median(lags_found))


def estimate_lever_arm_rb(
    t, R_wb, v_gnss_w, v_state_w, omega_b,     # arrays aligned in time (post Δt)
    speed_min=2.0,                             # m/s; ignore near-standstill
    omega_min_deg_s=2.0,                       # deg/s; excite turns
    huber_delta=0.5,                           # m/s robust scale
    l2_reg=1e-6,
    max_iter=5,
):
    """
    R_wb: (N,3,3) body->world
    v_gnss_w, v_state_w: (N,3)
    omega_b: (N,3) rad/s
    Returns r_b (3,), residual RMS (m/s), inliers frac.
    """
    N = len(t)
    assert R_wb.shape == (N, 3, 3)
    assert v_gnss_w.shape == (N, 3) and v_state_w.shape == (N, 3) and omega_b.shape == (N, 3)

    # Build A r = b over selected samples
    speed = np.linalg.norm(v_gnss_w, axis=1)
    omega_deg = np.linalg.norm(omega_b, axis=1) * 180/np.pi
    sel = (speed >= speed_min) & (omega_deg >= omega_min_deg_s)

    if not np.any(sel):
        return np.zeros(3), float('nan'), 0.0

    Rwbs = R_wb[sel]                     # (M,3,3)
    vdiff_w = (v_gnss_w - v_state_w)[sel]
    omegas = omega_b[sel]

    # Rotate velocity difference to body: b = R^T (v_a - v_c)
    b = np.einsum('nij,nj->ni', np.transpose(Rwbs, (0, 2, 1)), vdiff_w)  # (M,3)

    # A_k = [omega]_x
    def skew(w):
        wx, wy, wz = w
        return np.array([[0, -wz, wy], [wz, 0, -wx], [-wy, wx, 0]])

    A = np.stack([skew(w) for w in omegas], axis=0)   # (M,3,3)
    A2 = A.reshape(-1, 3)
    b2 = b.reshape(-1)

    # Robust IRLS with Huber loss
    W = np.ones_like(b2)
    r_b = np.zeros(3)
    for _ in range(max_iter):
        Aw = A2 * W[:, None]
        H = Aw.T @ A2 + l2_reg*np.eye(3)
        g = Aw.T @ (W * b2)
        r_b_new = np.linalg.solve(H, g)
        res = b2 - A2 @ r_b_new
        absr = np.abs(res)
        W = np.where(absr <= huber_delta, 1.0, huber_delta/absr)
        r_b = r_b_new

    res = b2 - A2 @ r_b
    rms = float(np.sqrt(np.mean(res**2)))
    inliers = float(np.mean(np.abs(res) <= 3*huber_delta))
    return r_b, rms, inliers

