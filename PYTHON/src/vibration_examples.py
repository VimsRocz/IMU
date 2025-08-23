#!/usr/bin/env python3
"""
IMU Vibration Simulation and Compensation Examples

This script demonstrates how to use the vibration model and compensation
modules to simulate and compensate vibration effects on IMU data.

Author: IMU Vibration Project
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Add the source directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from vibration_model import (VibrationModel, WaypointVibrationSimulator,
                            create_sinusoidal_vibration_model, 
                            create_motor_vibration_model,
                            create_quadrotor_vibration_model)
from vibration_compensation import (VibrationAnalyzer, 
                                   detect_vibration_simple,
                                   compensate_vibration_simple)


def example_1_basic_vibration_simulation():
    """
    Example 1: Basic vibration simulation with different vibration types.
    """
    print("=== Example 1: Basic Vibration Simulation ===")
    
    # Create vibration model
    vibration_model = VibrationModel(fs=400.0, seed=42)
    
    # Simulation parameters
    duration = 10.0  # seconds
    
    # Generate different types of vibration
    vibration_types = [
        ('sinusoidal', {'frequency': 50.0, 'amplitude_acc': 1.0, 'amplitude_gyro': 0.1}),
        ('random', {'cutoff_frequency': 100.0, 'amplitude_acc': 0.5, 'amplitude_gyro': 0.05}),
        ('motor', {'base_frequency': 60.0, 'amplitude_acc': 2.0, 'amplitude_gyro': 0.2}),
        ('rotor', {'rotor_frequency': 100.0, 'num_blades': 4, 'amplitude_acc': 3.0, 'amplitude_gyro': 0.3})
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    axes = axes.flatten()
    
    for i, (vib_type, params) in enumerate(vibration_types):
        print(f"Generating {vib_type} vibration...")
        
        # Generate vibration signals
        vib_signals = vibration_model.generate_vibration(duration, 
                                                        vibration_type=vib_type, 
                                                        **params)
        
        # Create time vector
        t = np.arange(len(vib_signals['accel_vib'])) / vibration_model.fs
        
        # Plot accelerometer vibration
        ax = axes[i]
        ax.plot(t, vib_signals['accel_vib'][:, 0], 'r-', label='X-axis', linewidth=0.8)
        ax.plot(t, vib_signals['accel_vib'][:, 1], 'g-', label='Y-axis', linewidth=0.8)  
        ax.plot(t, vib_signals['accel_vib'][:, 2], 'b-', label='Z-axis', linewidth=0.8)
        ax.set_title(f'{vib_type.capitalize()} Vibration - Accelerometer')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Print vibration statistics
        rms_acc = np.sqrt(np.mean(vib_signals['accel_vib']**2, axis=0))
        rms_gyro = np.sqrt(np.mean(vib_signals['gyro_vib']**2, axis=0))
        print(f"  RMS Accelerometer: [{rms_acc[0]:.3f}, {rms_acc[1]:.3f}, {rms_acc[2]:.3f}] m/s²")
        print(f"  RMS Gyroscope: [{rms_gyro[0]:.3f}, {rms_gyro[1]:.3f}, {rms_gyro[2]:.3f}] rad/s")
    
    plt.tight_layout()
    plt.savefig('/tmp/vibration_types_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Saved plot: /tmp/vibration_types_comparison.png\n")


def example_2_trajectory_with_vibration():
    """
    Example 2: Simulate IMU data along a trajectory with vibration.
    """
    print("=== Example 2: Trajectory Simulation with Vibration ===")
    
    # Create vibration model
    vibration_model = create_quadrotor_vibration_model(fs=400.0, 
                                                     rotor_frequency=120.0,
                                                     amplitude_acc=2.5,
                                                     amplitude_gyro=0.25)
    
    # Create waypoint simulator  
    simulator = WaypointVibrationSimulator(vibration_model)
    
    # Define a circular trajectory
    n_waypoints = 8
    radius = 50.0  # meters
    height = 10.0  # meters
    
    angles = np.linspace(0, 2*np.pi, n_waypoints)
    waypoints = np.column_stack([
        radius * np.cos(angles),
        radius * np.sin(angles), 
        np.full(n_waypoints, height)
    ])
    
    # Define times for each waypoint (30 second flight)
    times = np.linspace(0, 30, n_waypoints)
    
    # Generate trajectory with vibration
    print("Generating trajectory with quadrotor vibration...")
    vibration_params = vibration_model.default_params
    
    trajectory_data = simulator.generate_trajectory_with_vibration(
        waypoints, times, vibration_params
    )
    
    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    t = trajectory_data['time']
    
    # Plot 3D trajectory
    ax = axes[0, 0]
    pos = trajectory_data['position']
    ax.plot(pos[:, 0], pos[:, 1], 'b-', linewidth=2, label='Trajectory')
    ax.scatter(waypoints[:, 0], waypoints[:, 1], c='red', s=50, label='Waypoints')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Flight Trajectory (Top View)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Plot accelerometer data
    ax = axes[0, 1]
    accel = trajectory_data['accel']
    ax.plot(t, accel[:, 0], 'r-', label='X', linewidth=0.8)
    ax.plot(t, accel[:, 1], 'g-', label='Y', linewidth=0.8)
    ax.plot(t, accel[:, 2], 'b-', label='Z', linewidth=0.8)
    ax.set_title('Accelerometer with Vibration')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot gyroscope data
    ax = axes[1, 0]
    gyro = trajectory_data['gyro']
    ax.plot(t, gyro[:, 0], 'r-', label='X', linewidth=0.8)
    ax.plot(t, gyro[:, 1], 'g-', label='Y', linewidth=0.8) 
    ax.plot(t, gyro[:, 2], 'b-', label='Z', linewidth=0.8)
    ax.set_title('Gyroscope with Vibration')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Rate (rad/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot velocity
    ax = axes[1, 1]
    vel = trajectory_data['velocity']
    ax.plot(t, vel[:, 0], 'r-', label='X', linewidth=0.8)
    ax.plot(t, vel[:, 1], 'g-', label='Y', linewidth=0.8)
    ax.plot(t, vel[:, 2], 'b-', label='Z', linewidth=0.8)
    ax.set_title('Velocity Profile')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/tmp/trajectory_with_vibration.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Saved plot: /tmp/trajectory_with_vibration.png\n")
    
    return trajectory_data


def example_3_vibration_detection():
    """
    Example 3: Demonstrate vibration detection methods.
    """
    print("=== Example 3: Vibration Detection ===")
    
    # Create test data with and without vibration
    fs = 400.0
    duration = 20.0
    t = np.arange(0, duration, 1/fs)
    
    # Clean IMU signal (low frequency motion)
    clean_freq = 0.5  # Hz
    clean_accel = np.column_stack([
        0.5 * np.sin(2 * np.pi * clean_freq * t),
        0.3 * np.cos(2 * np.pi * clean_freq * t), 
        9.81 + 0.2 * np.sin(2 * np.pi * clean_freq * t)
    ])
    
    # Add vibration to second half
    vibration_model = VibrationModel(fs=fs, seed=42)
    vibration = vibration_model.generate_vibration(duration/2, 
                                                  vibration_type='motor',
                                                  base_frequency=75.0,
                                                  amplitude_acc=1.5)
    
    # Combine clean and vibrated data
    n_half = len(t) // 2
    test_accel = clean_accel.copy()
    test_accel[n_half:] += vibration['accel_vib']
    
    # Test different detection methods
    analyzer = VibrationAnalyzer(fs=fs)
    
    detection_methods = ['statistical', 'frequency', 'entropy']
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot original signal
    ax = axes[0, 0]
    ax.plot(t, test_accel[:, 0], 'r-', label='X', linewidth=0.8)
    ax.plot(t, test_accel[:, 1], 'g-', label='Y', linewidth=0.8)
    ax.plot(t, test_accel[:, 2], 'b-', label='Z', linewidth=0.8)
    ax.axvline(x=duration/2, color='k', linestyle='--', alpha=0.7, label='Vibration Start')
    ax.set_title('Test Signal (Clean + Vibration)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Test each detection method
    for i, method in enumerate(detection_methods):
        print(f"Testing {method} detection method...")
        
        if method == 'statistical':
            result = analyzer.detector.detect_vibration_statistical(test_accel)
        elif method == 'frequency':
            result = analyzer.detector.detect_vibration_frequency(test_accel)
        elif method == 'entropy':
            result = analyzer.detector.detect_vibration_entropy(test_accel)
        
        # Plot detection results
        ax = axes[0, 1] if i == 0 else axes[1, i-1]
        
        detection_signal = result['vibration_detected'].astype(float)
        ax.plot(t, detection_signal, 'r-', linewidth=2, label='Vibration Detected')
        ax.axvline(x=duration/2, color='k', linestyle='--', alpha=0.7, label='True Start')
        ax.set_title(f'{method.capitalize()} Detection')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Detection Flag')
        ax.set_ylim([-0.1, 1.1])
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Calculate detection statistics
        true_positive = np.sum(detection_signal[n_half:])
        false_positive = np.sum(detection_signal[:n_half])
        sensitivity = true_positive / n_half * 100
        specificity = (n_half - false_positive) / n_half * 100
        
        print(f"  {method} Detection - Sensitivity: {sensitivity:.1f}%, Specificity: {specificity:.1f}%")
    
    plt.tight_layout()
    plt.savefig('/tmp/vibration_detection_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Saved plot: /tmp/vibration_detection_comparison.png\n")


def example_4_vibration_compensation():
    """
    Example 4: Demonstrate vibration compensation methods.
    """
    print("=== Example 4: Vibration Compensation ===")
    
    # Generate vibrated IMU data
    fs = 400.0
    duration = 10.0
    
    # Clean trajectory motion
    t = np.arange(0, duration, 1/fs)
    clean_accel = np.column_stack([
        0.8 * np.sin(2 * np.pi * 0.2 * t),
        0.6 * np.cos(2 * np.pi * 0.3 * t),
        9.81 + 0.4 * np.sin(2 * np.pi * 0.1 * t)
    ])
    
    # Add motor vibration
    vibration_model = VibrationModel(fs=fs, seed=42)
    vibration = vibration_model.generate_vibration(duration,
                                                  vibration_type='motor',
                                                  base_frequency=85.0,
                                                  amplitude_acc=2.0)
    
    vibrated_accel = clean_accel + vibration['accel_vib']
    
    # Test compensation methods
    analyzer = VibrationAnalyzer(fs=fs)
    
    compensation_methods = [
        ('butterworth', {'cutoff_freq': 15.0}),
        ('notch', {'notch_freq': 85.0, 'quality_factor': 20.0}),
        ('kalman', {'process_noise': 1e-4, 'measurement_noise': 1e-2})
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot original signals
    ax = axes[0, 0]
    ax.plot(t, clean_accel[:, 2], 'g-', linewidth=2, label='Clean Signal', alpha=0.8)
    ax.plot(t, vibrated_accel[:, 2], 'r-', linewidth=1, label='With Vibration', alpha=0.7)
    ax.set_title('Z-axis Accelerometer: Original vs Vibrated')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Test each compensation method
    for i, (method, params) in enumerate(compensation_methods):
        print(f"Testing {method} compensation...")
        
        if method == 'butterworth':
            result = analyzer.compensator.compensate_butterworth_filter(vibrated_accel, **params)
        elif method == 'notch':
            result = analyzer.compensator.compensate_notch_filter(vibrated_accel, **params)  
        elif method == 'kalman':
            result = analyzer.compensator.compensate_kalman_filter(vibrated_accel, **params)
        
        compensated_accel = result['filtered_data']
        
        # Plot compensation results (Z-axis)
        ax_idx = [0, 1, 1][i]
        ay_idx = [1, 0, 1][i] 
        ax = axes[ax_idx, ay_idx]
        
        ax.plot(t, clean_accel[:, 2], 'g-', linewidth=2, label='True Signal', alpha=0.8)
        ax.plot(t, compensated_accel[:, 2], 'b-', linewidth=1.5, label='Compensated', alpha=0.9)
        ax.set_title(f'{method.capitalize()} Compensation')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Acceleration (m/s²)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Calculate compensation metrics
        error_original = np.sqrt(np.mean((vibrated_accel[:, 2] - clean_accel[:, 2])**2))
        error_compensated = np.sqrt(np.mean((compensated_accel[:, 2] - clean_accel[:, 2])**2))
        improvement = (error_original - error_compensated) / error_original * 100
        
        print(f"  {method} - RMSE reduction: {improvement:.1f}%")
    
    plt.tight_layout()
    plt.savefig('/tmp/vibration_compensation_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Saved plot: /tmp/vibration_compensation_comparison.png\n")


def example_5_integrated_analysis():
    """
    Example 5: Complete integrated vibration analysis and compensation.
    """
    print("=== Example 5: Integrated Vibration Analysis ===")
    
    # Load or generate sample IMU data
    from example_2_trajectory_with_vibration import trajectory_data
    
    # Use trajectory data from example 2 if available, otherwise generate new
    try:
        imu_data = {
            'accel': trajectory_data['accel'],
            'gyro': trajectory_data['gyro']
        }
        print("Using trajectory data from Example 2")
    except:
        print("Generating new test data...")
        fs = 400.0
        duration = 15.0
        
        vibration_model = create_motor_vibration_model(fs=fs, base_frequency=90.0)
        simulator = WaypointVibrationSimulator(vibration_model)
        
        # Simple straight line trajectory
        waypoints = np.array([[0, 0, 5], [100, 50, 5], [200, 0, 5]])
        times = np.array([0, 7.5, 15.0])
        
        trajectory_data = simulator.generate_trajectory_with_vibration(
            waypoints, times, vibration_model.default_params
        )
        
        imu_data = {
            'accel': trajectory_data['accel'],
            'gyro': trajectory_data['gyro']
        }
    
    # Perform integrated analysis
    analyzer = VibrationAnalyzer(fs=400.0)
    
    print("Performing integrated vibration analysis...")
    analysis_results = analyzer.analyze_and_compensate(
        imu_data, 
        detection_method='frequency',
        compensation_method='butterworth',
        cutoff_freq=25.0
    )
    
    # Generate report
    report = analyzer.get_vibration_report(analysis_results)
    print(report)
    
    # Plot before/after comparison  
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    t = np.arange(len(imu_data['accel'])) / 400.0
    
    # Accelerometer comparison
    ax = axes[0, 0]
    ax.plot(t, imu_data['accel'][:, 2], 'r-', linewidth=1, label='Original', alpha=0.8)
    compensated_accel = analysis_results['accel']['compensation']['filtered_data']
    ax.plot(t, compensated_accel[:, 2], 'b-', linewidth=1.5, label='Compensated', alpha=0.9)
    ax.set_title('Accelerometer Z-axis: Before/After')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Gyroscope comparison
    ax = axes[0, 1]
    ax.plot(t, imu_data['gyro'][:, 2], 'r-', linewidth=1, label='Original', alpha=0.8)
    compensated_gyro = analysis_results['gyro']['compensation']['filtered_data']
    ax.plot(t, compensated_gyro[:, 2], 'b-', linewidth=1.5, label='Compensated', alpha=0.9)
    ax.set_title('Gyroscope Z-axis: Before/After')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Rate (rad/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Vibration detection results
    ax = axes[1, 0]
    accel_detection = analysis_results['accel']['detection']['vibration_detected']
    ax.plot(t, accel_detection.astype(float), 'r-', linewidth=2, label='Accelerometer')
    
    gyro_detection = analysis_results['gyro']['detection']['vibration_detected']
    ax.plot(t, gyro_detection.astype(float) + 0.1, 'b-', linewidth=2, label='Gyroscope')
    
    ax.set_title('Vibration Detection Results')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Detection Flag')
    ax.set_ylim([-0.1, 1.2])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Power spectral density comparison
    ax = axes[1, 1]
    from scipy import signal
    
    f_orig, psd_orig = signal.welch(imu_data['accel'][:, 2], fs=400.0, nperseg=1024)
    f_comp, psd_comp = signal.welch(compensated_accel[:, 2], fs=400.0, nperseg=1024)
    
    ax.semilogy(f_orig, psd_orig, 'r-', label='Original', alpha=0.8)
    ax.semilogy(f_comp, psd_comp, 'b-', label='Compensated', alpha=0.8)
    ax.set_title('Power Spectral Density (Accel Z)')
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('PSD (m²/s⁴/Hz)')
    ax.set_xlim([0, 200])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/tmp/integrated_vibration_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("Saved plot: /tmp/integrated_vibration_analysis.png\n")
    
    return analysis_results


def main():
    """
    Run all vibration simulation and compensation examples.
    """
    print("IMU Vibration Simulation and Compensation Examples")
    print("=" * 55)
    print()
    
    # Create output directory
    os.makedirs('/tmp', exist_ok=True)
    
    try:
        # Run examples
        example_1_basic_vibration_simulation()
        trajectory_data = example_2_trajectory_with_vibration()
        example_3_vibration_detection() 
        example_4_vibration_compensation()
        example_5_integrated_analysis()
        
        print("All examples completed successfully!")
        print(f"Generated plots saved in /tmp/")
        
    except Exception as e:
        print(f"Error running examples: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()