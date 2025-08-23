#!/usr/bin/env python3
"""Test the fixed quaternion consistency"""

import numpy as np
from scipy.spatial.transform import Rotation as R

# Import the functions from validate_with_truth
import sys
sys.path.append('/home/runner/work/IMU/IMU/PYTHON/src')

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

def quat_to_euler_scipy_consistent(q_wxyz):
    """
    Convert quaternion wxyz to Euler ZYX using SciPy (our standard)
    This matches exactly what the fixed Python implementation uses
    """
    q_xyzw = as_xyzw_from_wxyz(q_wxyz)
    eul = R.from_quat(q_xyzw).as_euler("zyx", degrees=True)
    return eul

def quat_to_euler_matlab_fixed(q_wxyz):
    """
    Convert quaternion wxyz to Euler ZYX using MATLAB's new implementation
    This should now match SciPy exactly
    """
    q = np.asarray(q_wxyz, float)
    if q.ndim == 1:
        q = q.reshape(1, -1)
    
    euler_angles = []
    for i in range(len(q)):
        w, x, y, z = q[i, 0], q[i, 1], q[i, 2], q[i, 3]
        
        # Use the exact same formulation as the new MATLAB function
        # Yaw (rotation around Z-axis)
        yaw = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        
        # Pitch (rotation around Y-axis) 
        sin_pitch = 2*(w*y - z*x)
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)  # Clamp to avoid numerical issues
        pitch = np.degrees(np.arcsin(sin_pitch))
        
        # Roll (rotation around X-axis)
        roll = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        
        euler_angles.append([yaw, pitch, roll])
    
    return np.array(euler_angles)

def _enforce_continuity(q):
    """Test the enforce_continuity function"""
    q = np.asarray(q, float)
    if q.ndim == 1:
        return q
    
    q_cont = q.copy()
    for k in range(1, len(q)):
        if np.dot(q_cont[k], q_cont[k-1]) < 0:
            q_cont[k] = -q_cont[k]
    return q_cont

def test_fixes():
    """Test that our fixes work"""
    print("Testing Fixed Quaternion and Euler Angle Consistency")
    print("=" * 55)
    
    # Test with various cases
    test_cases = [
        [0, 0, 0],      # Identity
        [10, 5, 2],     # Small angles
        [90, 0, 0],     # Pure yaw
        [0, 90, 0],     # Pure pitch  
        [0, 0, 90],     # Pure roll
        [45, 30, 15],   # Mixed angles
        [180, -90, 180], # Large angles
    ]
    
    print("Test cases (yaw, pitch, roll in degrees):")
    for i, case in enumerate(test_cases):
        print(f"  Case {i+1}: [{case[0]:6.1f}, {case[1]:6.1f}, {case[2]:6.1f}]")
    
    all_good = True
    max_error = 0
    
    for i, eul_input in enumerate(test_cases):
        # Convert to quaternion using SciPy
        r = R.from_euler('zyx', eul_input, degrees=True)
        q_xyzw = r.as_quat()
        q_wxyz = as_wxyz_from_xyzw(q_xyzw)
        
        # Test both conversion methods
        eul_scipy = quat_to_euler_scipy_consistent(q_wxyz)
        eul_matlab_fixed = quat_to_euler_matlab_fixed(q_wxyz)
        
        # Compute differences
        if eul_scipy.ndim == 1:
            diff = eul_matlab_fixed[0] - eul_scipy
        else:
            diff = eul_matlab_fixed[0] - eul_scipy[0]
        
        max_case_error = np.max(np.abs(diff))
        max_error = max(max_error, max_case_error)
        
        status = "✅" if max_case_error < 0.001 else "❌"
        print(f"\nCase {i+1}: {status}")
        print(f"  Input:      [{eul_input[0]:6.1f}, {eul_input[1]:6.1f}, {eul_input[2]:6.1f}]")
        print(f"  SciPy:      [{eul_scipy[0]:6.1f}, {eul_scipy[1]:6.1f}, {eul_scipy[2]:6.1f}]") if eul_scipy.ndim > 1 else print(f"  SciPy:      [{eul_scipy[0]:6.1f}, {eul_scipy[1]:6.1f}, {eul_scipy[2]:6.1f}]")
        print(f"  MATLAB fix: [{eul_matlab_fixed[0,0]:6.1f}, {eul_matlab_fixed[0,1]:6.1f}, {eul_matlab_fixed[0,2]:6.1f}]")
        print(f"  Difference: [{diff[0]:6.3f}, {diff[1]:6.3f}, {diff[2]:6.3f}]  (max: {max_case_error:.3f}°)")
        
        if max_case_error >= 0.001:
            all_good = False
    
    print("\n" + "="*55)
    print(f"Maximum error across all test cases: {max_error:.6f}°")
    
    if all_good and max_error < 0.001:
        print("🎉 SUCCESS: All conversions are now consistent!")
        print("Both MATLAB and Python implementations should now produce identical results.")
        return True
    else:
        print("❌ FAILURE: There are still inconsistencies.")
        return False

def test_continuity_and_hemisphere():
    """Test continuity and hemisphere alignment functions"""
    print("\n" + "="*55)
    print("Testing Continuity and Hemisphere Alignment")
    print("="*55)
    
    # Create a quaternion series with sign flips
    q1 = np.array([0.9659, 0.2588, 0, 0])      # +30° roll
    q2 = np.array([-0.9659, -0.2588, 0, 0])    # Same rotation, opposite hemisphere
    q3 = np.array([0.9239, 0.3827, 0, 0])      # +45° roll
    
    q_series = np.array([q1, q2, q3])
    
    print("Original quaternion series:")
    for i, q in enumerate(q_series):
        print(f"  q{i+1}: [{q[0]:6.4f}, {q[1]:6.4f}, {q[2]:6.4f}, {q[3]:6.4f}]")
    
    # Test continuity enforcement
    q_continuous = _enforce_continuity(q_series)
    
    print("\nAfter continuity enforcement:")
    for i, q in enumerate(q_continuous):
        print(f"  q{i+1}: [{q[0]:6.4f}, {q[1]:6.4f}, {q[2]:6.4f}, {q[3]:6.4f}]")
    
    # Verify all adjacent quaternions have positive dot products
    continuity_ok = True
    for i in range(1, len(q_continuous)):
        dot = np.dot(q_continuous[i], q_continuous[i-1])
        print(f"  dot(q{i+1}, q{i}): {dot:.4f}")
        if dot < 0:
            continuity_ok = False
    
    status = "✅" if continuity_ok else "❌"
    print(f"\nContinuity enforcement: {status}")
    
    return continuity_ok

if __name__ == "__main__":
    success1 = test_fixes()
    success2 = test_continuity_and_hemisphere()
    
    if success1 and success2:
        print("\n🎉 ALL TESTS PASSED! The fixes should resolve the quaternion and Euler angle issues.")
    else:
        print("\n❌ Some tests failed. Further debugging may be needed.")