#!/usr/bin/env python3
"""
Simple test script for IMU vibration simulation and compensation functionality.

This script provides a quick test to verify that the vibration modules
are working correctly.
"""

import numpy as np
import sys
import os

# Add source directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from vibration_model import VibrationModel, WaypointVibrationSimulator
from vibration_compensation import VibrationAnalyzer


def test_basic_functionality():
    """Test basic vibration model and compensation functionality."""
    print("Testing IMU Vibration Simulation and Compensation...")
    
    # Test 1: Basic vibration generation
    print("1. Testing vibration model generation...")
    vm = VibrationModel(fs=400.0, seed=42)
    
    vibration_types = ['sinusoidal', 'random', 'motor', 'rotor']
    for vib_type in vibration_types:
        vib = vm.generate_vibration(1.0, vib_type)
        assert 'accel_vib' in vib and 'gyro_vib' in vib
        assert vib['accel_vib'].shape == (400, 3)
        assert vib['gyro_vib'].shape == (400, 3)
        print(f"   ✓ {vib_type} vibration generation successful")
    
    # Test 2: Waypoint trajectory simulation
    print("2. Testing waypoint trajectory simulation...")
    simulator = WaypointVibrationSimulator(vm)
    waypoints = np.array([[0, 0, 10], [50, 0, 10], [50, 50, 10]])
    times = np.array([0, 5, 10])
    
    trajectory = simulator.generate_trajectory_with_vibration(
        waypoints, times, {'vibration_type': 'motor', 'base_frequency': 60.0}
    )
    
    assert 'accel' in trajectory and 'gyro' in trajectory
    print("   ✓ Waypoint trajectory simulation successful")
    
    # Test 3: Vibration detection and compensation
    print("3. Testing vibration detection and compensation...")
    analyzer = VibrationAnalyzer(fs=400.0)
    
    imu_data = {
        'accel': trajectory['accel'],
        'gyro': trajectory['gyro']
    }
    
    results = analyzer.analyze_and_compensate(
        imu_data,
        detection_method='statistical',
        compensation_method='butterworth'
    )
    
    assert 'accel' in results and 'gyro' in results
    for sensor in ['accel', 'gyro']:
        assert 'detection' in results[sensor]
        assert 'compensation' in results[sensor]
        assert 'vibration_detected' in results[sensor]['detection']
        
    print("   ✓ Vibration detection and compensation successful")
    
    # Test 4: Generate analysis report
    print("4. Testing analysis report generation...")
    report = analyzer.get_vibration_report(results)
    assert isinstance(report, str) and len(report) > 0
    print("   ✓ Analysis report generation successful")
    
    print("\n" + "="*50)
    print("VIBRATION ANALYSIS REPORT")
    print("="*50)
    print(report)
    
    print("All tests passed! ✅")
    return True


if __name__ == "__main__":
    try:
        test_basic_functionality()
        print("\nIMU vibration simulation and compensation modules are working correctly!")
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)