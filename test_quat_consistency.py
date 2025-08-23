#!/usr/bin/env python3
"""Test script to demonstrate quaternion and Euler angle consistency issues between MATLAB and Python"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def as_xyzw_from_wxyz(qwxyz):
    """Convert wxyz to xyzw format"""
    qwxyz = np.asarray(qwxyz, float)
    if qwxyz.ndim == 1:
        return np.array([qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]])
    return np.column_stack([qwxyz[:, 1], qwxyz[:, 2], qwxyz[:, 3], qwxyz[:, 0]])

def as_wxyz_from_xyzw(qxyzw):
    """Convert xyzw to wxyz format"""
    qxyzw = np.asarray(qxyzw, float)
    if qxyzw.ndim == 1:
        return np.array([qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]])
    return np.column_stack([qxyzw[:, 3], qxyzw[:, 0], qxyzw[:, 1], qxyzw[:, 2]])

def quat_to_euler_matlab_style(q_wxyz):
    """
    Convert quaternion wxyz to Euler ZYX (yaw, pitch, roll) in degrees
    Using EXACT MATLAB formula from quat2rotm_local + rotm2eul_batch_local
    """
    q = np.asarray(q_wxyz, float)
    if q.ndim == 1:
        q = q.reshape(1, -1)
    
    euler_angles = []
    for i in range(len(q)):
        w, x, y, z = q[i, 0], q[i, 1], q[i, 2], q[i, 3]
        
        # Build rotation matrix exactly as in MATLAB quat2rotm_local
        R = np.array([
            [1-2*(y**2+z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1-2*(x**2+z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x**2+y**2)]
        ])
        
        # Extract Euler angles exactly as in MATLAB rotm2eul_batch_local
        yaw = np.degrees(np.arctan2(R[1,0], R[0,0]))
        pitch = np.degrees(np.arcsin(np.clip(-R[2,0], -1.0, 1.0)))
        roll = np.degrees(np.arctan2(R[2,1], R[2,2]))
        
        euler_angles.append([yaw, pitch, roll])
    
    return np.array(euler_angles)

def quat_to_euler_scipy_style(q_wxyz):
    """
    Convert quaternion wxyz to Euler ZYX using SciPy
    """
    q_xyzw = as_xyzw_from_wxyz(q_wxyz)
    eul = R.from_quat(q_xyzw).as_euler("zyx", degrees=True)
    return eul

def normalize_quat(q):
    """Normalize quaternion"""
    q = np.asarray(q, float)
    if q.ndim == 1:
        return q / np.linalg.norm(q)
    return q / np.linalg.norm(q, axis=1, keepdims=True)

def enforce_continuity(q):
    """Enforce quaternion continuity (hemisphere alignment)"""
    q = np.asarray(q, float)
    if q.ndim == 1:
        return q
    
    q_out = q.copy()
    for i in range(1, len(q)):
        if np.dot(q_out[i], q_out[i-1]) < 0:
            q_out[i] = -q_out[i]
    return q_out

def hemisphere_align(q_truth, q_est):
    """Align estimate quaternions to truth hemisphere"""
    q_truth = np.asarray(q_truth, float)
    q_est = np.asarray(q_est, float)
    
    if q_truth.ndim == 1:
        return q_est if np.dot(q_truth, q_est) >= 0 else -q_est
    
    dots = np.sum(q_truth * q_est, axis=1)
    flip_mask = dots < 0
    q_est_aligned = q_est.copy()
    q_est_aligned[flip_mask] = -q_est_aligned[flip_mask]
    return q_est_aligned

def test_quaternion_consistency():
    """Test quaternion and Euler angle consistency"""
    print("Testing Quaternion and Euler Angle Consistency")
    print("=" * 50)
    
    # Let's test with a very simple case first: identity quaternion
    print("Test 1: Identity quaternion")
    q_identity_wxyz = np.array([1, 0, 0, 0])
    eul_matlab = quat_to_euler_matlab_style(q_identity_wxyz)
    eul_scipy = quat_to_euler_scipy_style(q_identity_wxyz)
    print(f"Identity (wxyz): [{q_identity_wxyz[0]:.1f}, {q_identity_wxyz[1]:.1f}, {q_identity_wxyz[2]:.1f}, {q_identity_wxyz[3]:.1f}]")
    print(f"MATLAB->Euler: [{eul_matlab[0,0]:.1f}, {eul_matlab[0,1]:.1f}, {eul_matlab[0,2]:.1f}]")
    print(f"SciPy->Euler:  [{eul_scipy[0]:.1f}, {eul_scipy[1]:.1f}, {eul_scipy[2]:.1f}]")
    
    # Test 2: Pure yaw rotation
    print("\nTest 2: Pure yaw rotation (90 degrees)")
    yaw_90_rad = np.pi/2
    q_yaw90_wxyz = np.array([np.cos(yaw_90_rad/2), 0, 0, np.sin(yaw_90_rad/2)])
    eul_matlab = quat_to_euler_matlab_style(q_yaw90_wxyz)
    eul_scipy = quat_to_euler_scipy_style(q_yaw90_wxyz)
    print(f"Yaw 90° (wxyz): [{q_yaw90_wxyz[0]:.4f}, {q_yaw90_wxyz[1]:.4f}, {q_yaw90_wxyz[2]:.4f}, {q_yaw90_wxyz[3]:.4f}]")
    print(f"MATLAB->Euler: [{eul_matlab[0,0]:.1f}, {eul_matlab[0,1]:.1f}, {eul_matlab[0,2]:.1f}]")
    print(f"SciPy->Euler:  [{eul_scipy[0]:.1f}, {eul_scipy[1]:.1f}, {eul_scipy[2]:.1f}]")
    
    # Test 3: Pure roll rotation
    print("\nTest 3: Pure roll rotation (30 degrees)")
    roll_30_rad = np.pi/6
    q_roll30_wxyz = np.array([np.cos(roll_30_rad/2), np.sin(roll_30_rad/2), 0, 0])
    eul_matlab = quat_to_euler_matlab_style(q_roll30_wxyz)
    eul_scipy = quat_to_euler_scipy_style(q_roll30_wxyz)
    print(f"Roll 30° (wxyz): [{q_roll30_wxyz[0]:.4f}, {q_roll30_wxyz[1]:.4f}, {q_roll30_wxyz[2]:.4f}, {q_roll30_wxyz[3]:.4f}]")
    print(f"MATLAB->Euler: [{eul_matlab[0,0]:.1f}, {eul_matlab[0,1]:.1f}, {eul_matlab[0,2]:.1f}]")
    print(f"SciPy->Euler:  [{eul_scipy[0]:.1f}, {eul_scipy[1]:.1f}, {eul_scipy[2]:.1f}]")
    
    # Test 4: Check if scipy uses different convention
    print("\nTest 4: Direct verification with SciPy")
    # Create rotation using SciPy and see what quaternion it produces
    eul_test = [10, 5, 2]  # yaw, pitch, roll in degrees
    r = R.from_euler('zyx', eul_test, degrees=True)
    q_scipy_xyzw = r.as_quat()
    q_scipy_wxyz = as_wxyz_from_xyzw(q_scipy_xyzw)
    
    print(f"Input Euler: [{eul_test[0]:.1f}, {eul_test[1]:.1f}, {eul_test[2]:.1f}] (yaw, pitch, roll)")
    print(f"SciPy->Quat (xyzw): [{q_scipy_xyzw[0]:.4f}, {q_scipy_xyzw[1]:.4f}, {q_scipy_xyzw[2]:.4f}, {q_scipy_xyzw[3]:.4f}]")
    print(f"SciPy->Quat (wxyz): [{q_scipy_wxyz[0]:.4f}, {q_scipy_wxyz[1]:.4f}, {q_scipy_wxyz[2]:.4f}, {q_scipy_wxyz[3]:.4f}]")
    
    # Now convert back using both methods
    eul_matlab = quat_to_euler_matlab_style(q_scipy_wxyz)
    eul_scipy = quat_to_euler_scipy_style(q_scipy_wxyz)
    print(f"MATLAB->Euler: [{eul_matlab[0,0]:.1f}, {eul_matlab[0,1]:.1f}, {eul_matlab[0,2]:.1f}]")
    print(f"SciPy->Euler:  [{eul_scipy[0]:.1f}, {eul_scipy[1]:.1f}, {eul_scipy[2]:.1f}]")
    
    diff = eul_matlab[0] - eul_scipy
    print(f"Difference:     [{diff[0]:.3f}, {diff[1]:.3f}, {diff[2]:.3f}]")
    
    return np.max(np.abs(diff)) < 0.1

if __name__ == "__main__":
    test_quaternion_consistency()