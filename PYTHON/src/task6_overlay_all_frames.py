import os, json, math, numpy as np
import scipy.io as sio
from pathlib import Path
import matplotlib.pyplot as plt
from typing import Tuple, Dict


def _ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)


def _to_np(a):
    return np.asarray(a)


def load_estimates(est_file: str) -> Dict[str, np.ndarray]:
    """
    Load estimator output from .mat or .npz
    Expected keys (if present):
      t, time, pos_ned, vel_ned, acc_ned, pos_ecef, vel_ecef, acc_ecef,
      pos_body, vel_body, acc_body
    Return dict with as many of these as found. Arrays shape (N,3); time (N,).
    """
    p = Path(est_file)
    if p.suffix == '.npz':
        d = dict(np.load(p))
    else:
        d = {k: v for k, v in sio.loadmat(p, squeeze_me=True).items() if not k.startswith('__')}
    def pick(*names):
        for n in names:
            if n in d:
                x = _to_np(d[n])
                return x
        return None
    time = pick('t','time','Time','TIME','time_s','t_est')
    out = {
        'time': time,
        'pos_ned': pick('pos_ned','posNED','position_ned','pos_ned_m'),
        'vel_ned': pick('vel_ned','velNED','velocity_ned','vel_ned_ms'),
        'acc_ned': pick('acc_ned','accNED','acceleration_ned','acc_ned_ms'),
        'pos_ecef': pick('pos_ecef','posECEF','position_ecef','pos_ecef_m'),
        'vel_ecef': pick('vel_ecef','velECEF','velocity_ecef','vel_ecef_ms'),
        'acc_ecef': pick('acc_ecef','accECEF','acceleration_ecef','acc_ecef_ms'),
        'pos_body': pick('pos_body','posBODY','pos_body_m'),
        'vel_body': pick('vel_body','velBODY','vel_body_ms'),
        'acc_body': pick('acc_body','accBODY','acc_body_ms'),
    }
    return out


def load_truth_truthfile(truth_file: str) -> Dict[str, np.ndarray]:
    """
    Parse STATE_Xxxx.txt; support whitespace/csv with headers containing:
      time, pos_ecef_X/Y/Z, vel_ecef_X/Y/Z  (case-insensitive)
      optionally *_ned_* columns.
    Return dict with 'time', and as many of pos/vel in ECEF/NED as present.
    """
    import pandas as pd, numpy as np
    try:
        df = pd.read_csv(truth_file, sep=None, engine='python')
        cols = {c.lower(): c for c in df.columns}
        def grab(prefix, frame):
            key = {}
            for k in ['x','y','z','n','e','d']:
                for name in [f'{prefix}_{k}', f'{frame}_{k}', f'{prefix}_{frame}_{k}']:
                    if name in cols:
                        key[k] = cols[name]
                        break
            return key
        out = {}
        tcol = None
        for cand in ['time','t','posix_time','sec','seconds']:
            if cand in cols: tcol = cols[cand]; break
        out['time'] = df[tcol].to_numpy() if tcol else np.arange(len(df))
        ecef_map = {'x':None,'y':None,'z':None}
        for k, pat in {'x':['pos_ecef_x','ecef_x','x_ecef','x_ecef_m','x'],'y':['pos_ecef_y','ecef_y','y_ecef','y_ecef_m','y'],'z':['pos_ecef_z','ecef_z','z_ecef','z_ecef_m','z']}.items():
            for c in pat:
                if c in cols: ecef_map[k]=cols[c]; break
        if all(ecef_map.values()):
            out['pos_ecef'] = df[[ecef_map['x'],ecef_map['y'],ecef_map['z']]].to_numpy()
        vel_map = {'x':None,'y':None,'z':None}
        for k, pat in {'x':['vel_ecef_x','vx_ecef','ecef_vx','vx','vx_ecef_mps'],'y':['vel_ecef_y','vy_ecef','ecef_vy','vy','vy_ecef_mps'],'z':['vel_ecef_z','vz_ecef','ecef_vz','vz','vz_ecef_mps']}.items():
            for c in pat:
                if c in cols: vel_map[k]=cols[c]; break
        if all(vel_map.values()):
            out['vel_ecef'] = df[[vel_map['x'],vel_map['y'],vel_map['z']]].to_numpy()
        ned_map = {'n':None,'e':None,'d':None}
        for k, pat in {'n':['pos_n','ned_n','north'],'e':['pos_e','ned_e','east'],'d':['pos_d','ned_d','down']}.items():
            for c in pat:
                if c in cols: ned_map[k]=cols[c]; break
        if all(ned_map.values()):
            out['pos_ned'] = df[[ned_map['n'],ned_map['e'],ned_map['d']]].to_numpy()
        return out
    except Exception:
        pass
    arr = np.loadtxt(truth_file, comments='#')
    out = {'time': arr[:,1]}
    if arr.shape[1] >= 8:
        out['pos_ecef'] = arr[:,2:5]
        out['vel_ecef'] = arr[:,5:8]
    return out

# --- Frame helpers ---
def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    sL, cL = np.sin(lat_rad), np.cos(lat_rad)
    sO, cO = np.sin(lon_rad), np.cos(lon_rad)
    # NED-from-ECEF (3x3)
    return np.array([[-sL*cO, -sL*sO,  cL],
                     [   -sO,     cO, 0.0],
                     [-cL*cO, -cL*sO, -sL]])


def quat_to_dcm(qw,qx,qy,qz):
    # body->NED DCM from quaternion (qw,qx,qy,qz)
    q0,q1,q2,q3 = qw,qx,qy,qz
    R = np.array([
        [1-2*(q2*q2+q3*q3), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1-2*(q1*q1+q2*q2)]
    ])
    return R


def ned_to_body(v_ned: np.ndarray, q_b2n: Tuple[float,float,float,float]) -> np.ndarray:
    Rb2n = quat_to_dcm(*q_b2n)
    Rn2b = Rb2n.T
    return (Rn2b @ v_ned.T).T


def ecef_to_ned_vec(v_ecef: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R @ v_ecef.T).T


def ned_to_ecef_vec(v_ned: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R.T @ v_ned.T).T


def interp_to(t_src, X_src, t_dst):
    # piecewise linear per column
    X_src = np.asarray(X_src)
    out = np.zeros((len(t_dst), X_src.shape[1]))
    for i in range(X_src.shape[1]):
        out[:,i] = np.interp(t_dst, t_src, X_src[:,i])
    return out


def plot_overlay_3x3(time, est_xyz, truth_xyz, title, ylabels, outfile):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3,3, figsize=(16,10), sharex=True)
    comps = ['X','Y','Z']
    for r,yl in enumerate(ylabels):
        est_block = est_xyz[r] if est_xyz[r] is not None else np.zeros((len(time),3))
        truth_block = truth_xyz[r] if truth_xyz[r] is not None else np.zeros((len(time),3))
        for c in range(3):
            ax = axes[r,c]
            ax.plot(time, est_block[:,c], label='Estimated', linewidth=1.2)
            ax.plot(time, truth_block[:,c], linestyle='--', label='Truth', linewidth=1.0)
            ax.set_ylabel(f'{yl} {comps[c]}')
            if r==0: ax.set_title(['Position','Velocity','Acceleration'][c] if False else comps[c])
            if r==2: ax.set_xlabel('Time [s]')
            ax.grid(True, alpha=0.3)
    # Better titles
    axes[0,0].set_title('Position')
    axes[0,1].set_title('Velocity')
    axes[0,2].set_title('Acceleration')
    handles, labels = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=2, loc='upper center')
    fig.suptitle(title)
    fig.tight_layout(rect=[0,0,1,0.95])
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


def run_task6_overlay_all_frames(est_file: str, truth_file: str, output_dir: str,
                                 lat_deg: float = None, lon_deg: float = None,
                                 q_b2n: Tuple[float,float,float,float] = None):
    """
    - est_file: kf output .mat/.npz with at least NED series
    - truth_file: STATE file with ECEF and/or NED
    - lat/lon: if None, read from GNSS_X*.csv later (caller provides); otherwise use.
    - q_b2n: constant body->NED quaternion from Task 3 (if None, assume [1,0,0,0]).
    Saves three PNGs:
      {run}/task6/{run}_task6_overlay_NED.png
      {run}/task6/{run}_task6_overlay_ECEF.png
      {run}/task6/{run}_task6_overlay_BODY.png
    """
    from pathlib import Path
    outp = Path(output_dir); _ensure_dir(outp)

    est = load_estimates(est_file)
    tru = load_truth_truthfile(truth_file)

    t = est.get('time')
    if t is None: raise ValueError('Estimator output lacks time array.')
    # Build NED for both
    # Estimated: prefer provided; else convert from ECEF if present
    pos_ned_est = est.get('pos_ned')
    vel_ned_est = est.get('vel_ned')
    acc_ned_est = est.get('acc_ned')

    # If NED missing but ECEF present and lat/lon given, convert:
    if (pos_ned_est is None or vel_ned_est is None) and est.get('pos_ecef') is not None and lat_deg is not None:
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        if pos_ned_est is None: pos_ned_est = ecef_to_ned_vec(est['pos_ecef'], lat, lon)
        if vel_ned_est is None and est.get('vel_ecef') is not None:
            vel_ned_est = ecef_to_ned_vec(est['vel_ecef'], lat, lon)
        if acc_ned_est is None and est.get('acc_ecef') is not None:
            acc_ned_est = ecef_to_ned_vec(est['acc_ecef'], lat, lon)

    # Truth: prefer NED; else from ECEF with lat/lon
    pos_ned_tru = tru.get('pos_ned')
    vel_ned_tru = tru.get('vel_ned')
    acc_ned_tru = tru.get('acc_ned')  # optional; often missing
    if pos_ned_tru is None and tru.get('pos_ecef') is not None and lat_deg is not None:
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        pos_ned_tru = ecef_to_ned_vec(tru['pos_ecef'], lat, lon)
        if tru.get('vel_ecef') is not None:
            vel_ned_tru = ecef_to_ned_vec(tru['vel_ecef'], lat, lon)

    # Interpolate truth to estimator timebase
    t_tru = tru.get('time')
    if t_tru is None: t_tru = t
    if pos_ned_tru is not None:
        pos_ned_tru = interp_to(t_tru, pos_ned_tru, t)
    if vel_ned_tru is not None:
        vel_ned_tru = interp_to(t_tru, vel_ned_tru, t)
    if acc_ned_tru is not None:
        acc_ned_tru = interp_to(t_tru, acc_ned_tru, t)
    pos_ecef_tru = tru.get('pos_ecef')
    vel_ecef_tru = tru.get('vel_ecef')
    acc_ecef_tru = tru.get('acc_ecef')
    if pos_ecef_tru is not None:
        pos_ecef_tru = interp_to(t_tru, pos_ecef_tru, t)
    if vel_ecef_tru is not None:
        vel_ecef_tru = interp_to(t_tru, vel_ecef_tru, t)
    if acc_ecef_tru is not None:
        acc_ecef_tru = interp_to(t_tru, acc_ecef_tru, t)

    # BODY frame using constant q_b2n (from Task 3 TRIAD by default)
    if q_b2n is None: q_b2n = (1.0,0.0,0.0,0.0)
    def to_body(Pn, Vn, An):
        Pb = ned_to_body(Pn, q_b2n) if Pn is not None else None
        Vb = ned_to_body(Vn, q_b2n) if Vn is not None else None
        Ab = ned_to_body(An, q_b2n) if An is not None else None
        return Pb, Vb, Ab

    # ECEF frame: if not provided, convert from NED using lat/lon
    def ensure_ecef(Pn, Vn, An):
        if Pn is None: return None, None, None
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        Pe = ned_to_ecef_vec(Pn, lat, lon)
        Ve = ned_to_ecef_vec(Vn, lat, lon) if Vn is not None else None
        Ae = ned_to_ecef_vec(An, lat, lon) if An is not None else None
        return Pe, Ve, Ae

    # --- Build per-frame triplets (pos, vel, acc) for est and truth ---
    # NED
    NED_est = (pos_ned_est, vel_ned_est, acc_ned_est)
    NED_tru = (pos_ned_tru, vel_ned_tru, acc_ned_tru)

    # BODY
    BODY_est = to_body(*NED_est)
    BODY_tru = to_body(*NED_tru)

    # ECEF
    if lat_deg is None or lon_deg is None:
        raise ValueError('lat_deg/lon_deg required to produce ECEF overlays when only NED is available.')
    ECEF_est = (est.get('pos_ecef'), est.get('vel_ecef'), est.get('acc_ecef'))
    ECEF_tru = (pos_ecef_tru, vel_ecef_tru, acc_ecef_tru)
    if ECEF_est[0] is None and NED_est[0] is not None:
        ECEF_est = ensure_ecef(*NED_est)
    if ECEF_tru[0] is None and NED_tru[0] is not None:
        ECEF_tru = ensure_ecef(*NED_tru)

    # --- Plot ---
    # Wrap in lists for plotting helper: [pos, vel, acc]
    def pack(trip):
        return [trip[0], trip[1], trip[2]]

    # NED
    if NED_est[0] is not None and NED_tru[0] is not None:
        plot_overlay_3x3(
            t,
            [NED_est[0], NED_est[1], NED_est[2]],
            [NED_tru[0], NED_tru[1], NED_tru[2]],
            title='Task 6: Overlay (NED)',
            ylabels=['Pos [m]','Vel [m/s]','Acc [m/s²]'],
            outfile=str(Path(output_dir)/'task6_overlay_NED.png')
        )
    # ECEF
    if ECEF_est[0] is not None and ECEF_tru[0] is not None:
        plot_overlay_3x3(
            t,
            [ECEF_est[0], ECEF_est[1], ECEF_est[2]],
            [ECEF_tru[0], ECEF_tru[1], ECEF_tru[2]],
            title='Task 6: Overlay (ECEF)',
            ylabels=['Pos [m]','Vel [m/s]','Acc [m/s²]'],
            outfile=str(Path(output_dir)/'task6_overlay_ECEF.png')
        )
    # BODY
    if BODY_est[0] is not None and BODY_tru[0] is not None:
        plot_overlay_3x3(
            t,
            [BODY_est[0], BODY_est[1], BODY_est[2]],
            [BODY_tru[0], BODY_tru[1], BODY_tru[2]],
            title='Task 6: Overlay (BODY)',
            ylabels=['Pos [m]','Vel [m/s]','Acc [m/s²]'],
            outfile=str(Path(output_dir)/'task6_overlay_BODY.png')
        )

    return True
