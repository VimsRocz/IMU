#!/usr/bin/env python3
"""Figure out the exact SciPy formulation"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def reverse_engineer_scipy():
    """Let's see exactly what scipy does"""
    
    # Test with a simple case
    q_xyzw = np.array([0.1, 0.2, 0.3, 0.9])
    q_xyzw = q_xyzw / np.linalg.norm(q_xyzw)  # normalize
    
    print("Quaternion (xyzw):", q_xyzw)
    print("Quaternion (wxyz):", [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    
    r = R.from_quat(q_xyzw)
    dcm = r.as_matrix()
    eul = r.as_euler('zyx', degrees=True)
    
    print("\nRotation matrix:")
    print(dcm)
    
    print(f"\nSciPy Euler (zyx): {eul}")
    
    # Now let's manually extract Euler angles from the rotation matrix
    # Using the ZYX convention (yaw-pitch-roll)
    r11, r12, r13 = dcm[0, 0], dcm[0, 1], dcm[0, 2]
    r21, r22, r23 = dcm[1, 0], dcm[1, 1], dcm[1, 2]
    r31, r32, r33 = dcm[2, 0], dcm[2, 1], dcm[2, 2]
    
    # Standard ZYX Euler angle extraction
    yaw = np.degrees(np.arctan2(r21, r11))
    pitch = np.degrees(np.arcsin(-r31))
    roll = np.degrees(np.arctan2(r32, r33))
    
    print(f"Manual from DCM:   [{yaw:.6f}, {pitch:.6f}, {roll:.6f}]")
    
    # Now let's compute directly from quaternion
    w, x, y, z = q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]
    
    # Build rotation matrix from quaternion (standard formula)
    r11_q = 1 - 2*(y*y + z*z)
    r12_q = 2*(x*y - w*z)
    r13_q = 2*(x*z + w*y)
    r21_q = 2*(x*y + w*z)
    r22_q = 1 - 2*(x*x + z*z)
    r23_q = 2*(y*z - w*x)
    r31_q = 2*(x*z - w*y)
    r32_q = 2*(y*z + w*x)
    r33_q = 1 - 2*(x*x + y*y)
    
    # Extract Euler from our quaternion-derived DCM
    yaw_q = np.degrees(np.arctan2(r21_q, r11_q))
    pitch_q = np.degrees(np.arcsin(-r31_q))
    roll_q = np.degrees(np.arctan2(r32_q, r33_q))
    
    print(f"From quat->DCM:    [{yaw_q:.6f}, {pitch_q:.6f}, {roll_q:.6f}]")
    
    # The difference should be zero if our math is right
    diff_dcm = np.max(np.abs([yaw - yaw_q, pitch - pitch_q, roll - roll_q]))
    print(f"DCM difference: {diff_dcm:.10f}")
    
    diff_scipy = np.max(np.abs([eul[0] - yaw_q, eul[1] - pitch_q, eul[2] - roll_q]))
    print(f"SciPy difference: {diff_scipy:.10f}")

if __name__ == "__main__":
    reverse_engineer_scipy()