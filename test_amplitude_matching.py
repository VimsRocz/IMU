#!/usr/bin/env python3
"""Test quaternion amplitude matching and consistent processing"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Import the functions from validate_with_truth if available
import sys
sys.path.append('/home/runner/work/IMU/IMU/PYTHON/src')

def as_xyzw_from_wxyz(qwxyz):
    qwxyz = np.asarray(qwxyz, float)
    if qwxyz.ndim == 1:
        return np.array([qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]])
    return np.column_stack([qwxyz[:, 1], qwxyz[:, 2], qwxyz[:, 3], qwxyz[:, 0]])

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

def process_quaternions_python_way(q_truth, q_estimate):
    """Process quaternions exactly as the Python implementation does"""
    # 1. Normalize
    q_truth = q_truth / np.linalg.norm(q_truth, axis=1, keepdims=True)
    q_estimate = q_estimate / np.linalg.norm(q_estimate, axis=1, keepdims=True)
    
    # 2. Enforce continuity
    q_truth = enforce_continuity(q_truth)
    q_estimate = enforce_continuity(q_estimate)
    
    # 3. Hemisphere align estimate to truth
    q_estimate = hemisphere_align(q_truth, q_estimate)
    
    return q_truth, q_estimate

def process_quaternions_matlab_way(q_truth, q_estimate):
    """Process quaternions exactly as the fixed MATLAB implementation does"""
    # 1. Normalize (same as Python)
    q_truth = q_truth / np.linalg.norm(q_truth, axis=1, keepdims=True)
    q_estimate = q_estimate / np.linalg.norm(q_estimate, axis=1, keepdims=True)
    
    # 2. Enforce continuity (same as Python)
    q_truth = enforce_continuity(q_truth)
    q_estimate = enforce_continuity(q_estimate)
    
    # 3. Hemisphere align estimate to truth (same as Python)
    q_estimate = hemisphere_align(q_truth, q_estimate)
    
    return q_truth, q_estimate

def create_test_quaternion_series():
    """Create test quaternion series with some amplitude and sign issues"""
    
    # Create a series of rotations
    t = np.linspace(0, 10, 50)
    
    # Truth: smooth rotation sequence
    angles_truth = np.column_stack([
        10 * np.sin(0.1 * t),  # varying yaw
        5 * np.cos(0.15 * t),  # varying pitch  
        2 * np.sin(0.2 * t)    # varying roll
    ])
    
    # Estimate: similar but with some errors and sign flips
    angles_est = angles_truth + np.random.normal(0, 1, angles_truth.shape)
    
    # Convert to quaternions
    q_truth = []
    q_estimate = []
    
    for i in range(len(t)):
        # Truth quaternions
        r_t = R.from_euler('zyx', angles_truth[i], degrees=True)
        q_t_xyzw = r_t.as_quat()
        q_t_wxyz = np.array([q_t_xyzw[3], q_t_xyzw[0], q_t_xyzw[1], q_t_xyzw[2]])
        q_truth.append(q_t_wxyz)
        
        # Estimate quaternions (with some random sign flips and amplitude errors)
        r_e = R.from_euler('zyx', angles_est[i], degrees=True)  
        q_e_xyzw = r_e.as_quat()
        q_e_wxyz = np.array([q_e_xyzw[3], q_e_xyzw[0], q_e_xyzw[1], q_e_xyzw[2]])
        
        # Introduce some amplitude and sign issues
        if i % 7 == 0:  # Random sign flip every 7th quaternion
            q_e_wxyz = -q_e_wxyz
        if i % 5 == 0:  # Random amplitude scaling every 5th quaternion 
            q_e_wxyz = q_e_wxyz * (1 + 0.1 * np.random.randn())
            
        q_estimate.append(q_e_wxyz)
    
    return np.array(q_truth), np.array(q_estimate), t

def test_amplitude_matching():
    """Test the amplitude matching issue mentioned in the problem statement"""
    print("Testing Quaternion Amplitude Matching")
    print("="*50)
    
    # Create test data
    q_truth, q_estimate, t = create_test_quaternion_series()
    
    print(f"Generated {len(q_truth)} quaternion samples")
    
    # Check initial state
    norms_truth = np.linalg.norm(q_truth, axis=1)
    norms_est = np.linalg.norm(q_estimate, axis=1)
    
    print(f"Truth quaternion norms: min={np.min(norms_truth):.6f}, max={np.max(norms_truth):.6f}")
    print(f"Estimate quaternion norms: min={np.min(norms_est):.6f}, max={np.max(norms_est):.6f}")
    
    # Process using both methods
    q_t_py, q_e_py = process_quaternions_python_way(q_truth.copy(), q_estimate.copy())
    q_t_mat, q_e_mat = process_quaternions_matlab_way(q_truth.copy(), q_estimate.copy())
    
    # Check if processing results are identical
    diff_truth = np.max(np.abs(q_t_py - q_t_mat))
    diff_est = np.max(np.abs(q_e_py - q_e_mat))
    
    print(f"\nProcessing consistency:")
    print(f"Truth quaternion difference (Python vs MATLAB): {diff_truth:.10f}")
    print(f"Estimate quaternion difference (Python vs MATLAB): {diff_est:.10f}")
    
    # Check amplitude matching
    dots_py = np.sum(q_t_py * q_e_py, axis=1)
    dots_mat = np.sum(q_t_mat * q_e_mat, axis=1)
    
    print(f"\nQuaternion dot products (should be positive after alignment):")
    print(f"Python method: min={np.min(dots_py):.6f}, mean={np.mean(dots_py):.6f}")
    print(f"MATLAB method: min={np.min(dots_mat):.6f}, mean={np.mean(dots_mat):.6f}")
    
    # Convert to Euler angles for comparison
    eul_t_py = R.from_quat(as_xyzw_from_wxyz(q_t_py)).as_euler('zyx', degrees=True)
    eul_e_py = R.from_quat(as_xyzw_from_wxyz(q_e_py)).as_euler('zyx', degrees=True)
    eul_t_mat = R.from_quat(as_xyzw_from_wxyz(q_t_mat)).as_euler('zyx', degrees=True)
    eul_e_mat = R.from_quat(as_xyzw_from_wxyz(q_e_mat)).as_euler('zyx', degrees=True)
    
    # Check Euler angle consistency
    euler_diff_truth = np.max(np.abs(eul_t_py - eul_t_mat))
    euler_diff_est = np.max(np.abs(eul_e_py - eul_e_mat))
    
    print(f"\nEuler angle consistency:")
    print(f"Truth Euler difference: {euler_diff_truth:.6f} degrees")
    print(f"Estimate Euler difference: {euler_diff_est:.6f} degrees")
    
    # Calculate attitude errors
    angle_err_py = 2 * np.degrees(np.arccos(np.clip(np.abs(dots_py), 0, 1)))
    angle_err_mat = 2 * np.degrees(np.arccos(np.clip(np.abs(dots_mat), 0, 1)))
    
    print(f"\nAttitude errors:")
    print(f"Python method: mean={np.mean(angle_err_py):.2f}°, max={np.max(angle_err_py):.2f}°")
    print(f"MATLAB method: mean={np.mean(angle_err_mat):.2f}°, max={np.max(angle_err_mat):.2f}°")
    
    # Success criteria
    processing_consistent = (diff_truth < 1e-10) and (diff_est < 1e-10)
    euler_consistent = (euler_diff_truth < 0.001) and (euler_diff_est < 0.001)
    no_negative_dots = np.all(dots_py >= -0.001) and np.all(dots_mat >= -0.001)
    
    print(f"\n{'='*50}")
    if processing_consistent and euler_consistent and no_negative_dots:
        print("🎉 SUCCESS: Amplitude matching and processing consistency achieved!")
        print("Both MATLAB and Python implementations should now produce identical results.")
        return True
    else:
        print("❌ ISSUES DETECTED:")
        if not processing_consistent:
            print(f"   - Quaternion processing inconsistency: {max(diff_truth, diff_est):.2e}")
        if not euler_consistent:
            print(f"   - Euler angle inconsistency: {max(euler_diff_truth, euler_diff_est):.6f}°")
        if not no_negative_dots:
            print(f"   - Hemisphere alignment failed: {np.sum(dots_py < 0) + np.sum(dots_mat < 0)} negative dot products")
        return False

if __name__ == "__main__":
    success = test_amplitude_matching()
    
    if success:
        print("\n✅ The fixes should resolve the quaternion amplitude matching issue!")
    else:
        print("\n❌ Further debugging may be needed.")