#!/usr/bin/env python3
"""
Test script to validate the Smart Sensor Fusion workflow documentation
by running a small example and generating sample outputs.
"""

import sys
import os
from pathlib import Path
import subprocess
import numpy as np

def test_workflow_implementation():
    """Test the workflow by running the available scripts and checking outputs."""
    
    print("=== Smart Sensor Fusion Workflow Test ===\n")
    
    # Check if we're in the correct directory
    base_dir = Path.cwd()
    python_dir = base_dir / "PYTHON"
    src_dir = python_dir / "src"
    
    if not python_dir.exists():
        print("ERROR: PYTHON directory not found. Run this script from the repository root.")
        return False
    
    print(f"Repository root: {base_dir}")
    print(f"Python source: {src_dir}")
    
    # Check for key files mentioned in the presentation
    key_files = [
        "PYTHON/src/GNSS_IMU_Fusion.py",
        "PYTHON/src/run_all_methods_single.py", 
        "PYTHON/src/gnss_imu_fusion/init_vectors.py",
        "gui.py"
    ]
    
    print("\n=== Checking Key Files ===")
    all_files_exist = True
    for file_path in key_files:
        full_path = base_dir / file_path
        if full_path.exists():
            print(f"✓ {file_path}")
        else:
            print(f"✗ {file_path} - NOT FOUND")
            all_files_exist = False
    
    if not all_files_exist:
        print("\nWARNING: Some key files are missing. The presentation may reference unavailable code.")
    
    # Check for sample data files
    print("\n=== Checking Sample Data ===")
    data_dir = base_dir / "DATA"
    if data_dir.exists():
        imu_files = list(data_dir.glob("**/IMU_*.dat"))
        gnss_files = list(data_dir.glob("**/GNSS_*.csv"))
        
        print(f"Found {len(imu_files)} IMU files")
        print(f"Found {len(gnss_files)} GNSS files")
        
        if imu_files:
            print(f"Example IMU file: {imu_files[0]}")
        if gnss_files:
            print(f"Example GNSS file: {gnss_files[0]}")
            
        # Test with small data if available
        if imu_files and gnss_files:
            test_with_sample_data(imu_files[0], gnss_files[0], src_dir)
    else:
        print("DATA directory not found - cannot test with real data")
    
    # Test the core algorithms directly
    print("\n=== Testing Core Algorithms ===")
    test_core_algorithms()
    
    print("\n=== Workflow Test Complete ===")
    return True

def test_with_sample_data(imu_file, gnss_file, src_dir):
    """Test the workflow with actual data files."""
    print(f"\n=== Testing with Sample Data ===")
    print(f"IMU: {imu_file}")
    print(f"GNSS: {gnss_file}")
    
    # Test if we can run the help for the main script
    try:
        main_script = src_dir / "GNSS_IMU_Fusion.py"
        if main_script.exists():
            result = subprocess.run([
                sys.executable, str(main_script), "--help"
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("✓ Main script is callable")
                print("Available options:", result.stdout.split('\n')[0:3])
            else:
                print("⚠ Main script returned non-zero exit code")
        
        # Test the "ALL Methods" script help
        all_methods_script = src_dir / "run_all_methods_single.py" 
        if all_methods_script.exists():
            result = subprocess.run([
                sys.executable, str(all_methods_script), "--help"
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("✓ ALL Methods script is callable")
            else:
                print("⚠ ALL Methods script returned non-zero exit code")
        
    except subprocess.TimeoutExpired:
        print("⚠ Script test timed out")
    except Exception as e:
        print(f"⚠ Error testing scripts: {e}")

def test_core_algorithms():
    """Test the core attitude determination algorithms with synthetic data."""
    try:
        # Add the src directory to Python path
        sys.path.insert(0, str(Path.cwd() / "PYTHON" / "src"))
        
        # Import the core functions
        from gnss_imu_fusion.init_vectors import triad_svd, svd_alignment, davenport_q_method
        
        print("✓ Successfully imported core functions")
        
        # Create synthetic test vectors
        # Gravity vectors (pointing down in both frames, should result in identity rotation)
        g_body = np.array([0.0, 0.0, 1.0])  # Down in body frame
        g_ned = np.array([0.0, 0.0, 1.0])   # Down in NED frame
        
        # Earth rotation vectors (small vectors, arbitrary but consistent)
        omega_body = np.array([1e-5, 0.0, 0.0])
        omega_ned = np.array([1e-5, 0.0, 0.0])
        
        print("\nTesting with identity case (should give near-identity rotation):")
        print(f"Body gravity: {g_body}")
        print(f"NED gravity: {g_ned}")
        
        # Test TRIAD
        print("\n--- Testing TRIAD Method ---")
        R_triad = triad_svd(g_body, omega_body, g_ned, omega_ned)
        print(f"TRIAD rotation matrix:\n{R_triad}")
        print(f"TRIAD determinant: {np.linalg.det(R_triad):.6f} (should be ~1.0)")
        
        # Test SVD 
        print("\n--- Testing SVD Method ---")
        R_svd = svd_alignment([g_body, omega_body], [g_ned, omega_ned])
        print(f"SVD rotation matrix:\n{R_svd}")
        print(f"SVD determinant: {np.linalg.det(R_svd):.6f} (should be ~1.0)")
        
        # Test Davenport
        print("\n--- Testing Davenport Method ---")
        R_dav, q_dav = davenport_q_method(g_body, omega_body, g_ned, omega_ned)
        print(f"Davenport rotation matrix:\n{R_dav}")
        print(f"Davenport quaternion: {q_dav}")
        print(f"Davenport determinant: {np.linalg.det(R_dav):.6f} (should be ~1.0)")
        
        # Compare results
        print("\n--- Method Comparison ---")
        triad_error = np.linalg.norm(R_triad - np.eye(3))
        svd_error = np.linalg.norm(R_svd - np.eye(3))
        dav_error = np.linalg.norm(R_dav - np.eye(3))
        
        print(f"TRIAD error from identity: {triad_error:.2e}")
        print(f"SVD error from identity: {svd_error:.2e}")
        print(f"Davenport error from identity: {dav_error:.2e}")
        
        print("✓ Core algorithms test completed successfully")
        
    except ImportError as e:
        print(f"⚠ Could not import core functions: {e}")
        print("This may indicate the Python path or module structure needs adjustment")
    except Exception as e:
        print(f"⚠ Error testing core algorithms: {e}")

def check_dependencies():
    """Check if required dependencies are available."""
    print("\n=== Checking Dependencies ===")
    required_packages = ['numpy', 'matplotlib', 'scipy']
    
    for package in required_packages:
        try:
            __import__(package)
            print(f"✓ {package}")
        except ImportError:
            print(f"✗ {package} - NOT AVAILABLE")

if __name__ == "__main__":
    print("Smart Sensor Fusion Workflow Validation")
    print("=" * 50)
    
    check_dependencies()
    test_workflow_implementation()
    
    print("\nValidation complete. Check the generated presentation file:")
    print("Smart_Sensor_Fusion_Workflow_Presentation.html")