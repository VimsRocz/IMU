#!/usr/bin/env python3
"""
Task 8: IMU Vibration Analysis and Compensation

This module provides vibration analysis and compensation for IMU data as part
of the GNSS/IMU fusion pipeline. It integrates with the existing task structure
and provides vibration detection, compensation, and analysis capabilities.

Author: IMU Vibration Project
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from typing import Dict, Any, Optional

# Add path for vibration modules
sys.path.insert(0, os.path.dirname(__file__))

try:
    from vibration_model import VibrationModel
    from vibration_compensation import VibrationAnalyzer
except ImportError:
    # Handle case where modules are in different location
    import importlib.util
    
    # Try to find modules in PYTHON/src
    base_dir = os.path.dirname(os.path.dirname(__file__))
    vibration_model_path = os.path.join(base_dir, 'vibration_model.py')
    vibration_comp_path = os.path.join(base_dir, 'vibration_compensation.py')
    
    if os.path.exists(vibration_model_path):
        spec = importlib.util.spec_from_file_location("vibration_model", vibration_model_path)
        vibration_model = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vibration_model)
        VibrationModel = vibration_model.VibrationModel
        
        spec = importlib.util.spec_from_file_location("vibration_compensation", vibration_comp_path)
        vibration_compensation = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vibration_compensation)
        VibrationAnalyzer = vibration_compensation.VibrationAnalyzer
    else:
        raise ImportError("Could not find vibration modules")


def task8_vibration_analysis(imu_data: Dict[str, np.ndarray], 
                            results_dir: str, 
                            fs: float = 400.0,
                            detection_method: str = 'frequency',
                            compensation_method: str = 'butterworth',
                            **kwargs) -> Dict[str, Any]:
    """
    Task 8: Analyze and compensate vibration in IMU data.
    
    Args:
        imu_data: Dictionary with 'accel', 'gyro', and optionally 'time' keys
        results_dir: Directory to save results and plots
        fs: Sampling frequency in Hz
        detection_method: 'statistical', 'frequency', or 'entropy'
        compensation_method: 'butterworth', 'notch', 'kalman', 'adaptive', or 'wavelet'
        **kwargs: Additional parameters for detection/compensation
        
    Returns:
        Dictionary with vibration analysis results and compensated data
    """
    print("Task 8: IMU Vibration Analysis and Compensation")
    print("=" * 50)
    
    # Ensure results directory exists
    os.makedirs(results_dir, exist_ok=True)
    
    # Initialize vibration analyzer
    analyzer = VibrationAnalyzer(fs=fs)
    
    # Extract IMU data
    required_keys = ['accel', 'gyro']
    for key in required_keys:
        if key not in imu_data:
            raise ValueError(f"Missing required IMU data key: {key}")
    
    # Get time vector
    if 'time' in imu_data:
        time = imu_data['time']
    else:
        n_samples = len(imu_data['accel'])
        time = np.arange(n_samples) / fs
    
    print(f"Analyzing {len(time):.1f} seconds of IMU data at {fs} Hz...")
    
    # Perform comprehensive vibration analysis
    print(f"Detection method: {detection_method}")
    print(f"Compensation method: {compensation_method}")
    
    # Separate detection and compensation parameters
    detection_params = {}
    compensation_params = {}
    
    for key, value in kwargs.items():
        # Parameters specific to frequency detection
        if key in ['vibration_freq_range', 'window_size', 'overlap_ratio', 'threshold_factor']:
            detection_params[key] = value
        # Parameters specific to compensation methods
        elif key in ['cutoff_freq', 'notch_freq', 'quality_factor', 'filter_order', 
                    'process_noise', 'measurement_noise', 'filter_length', 'step_size']:
            compensation_params[key] = value
        else:
            # Put unknown parameters in both (backwards compatibility)
            detection_params[key] = value
            compensation_params[key] = value
    
    analysis_results = analyzer.analyze_and_compensate(
        {'accel': imu_data['accel'], 'gyro': imu_data['gyro']},
        detection_method=detection_method,
        compensation_method=compensation_method,
        detection_params=detection_params,
        compensation_params=compensation_params
    )
    
    # Generate and print report
    report = analyzer.get_vibration_report(analysis_results)
    print("\n" + report)
    
    # Create comprehensive plots
    _plot_vibration_analysis(time, imu_data, analysis_results, results_dir, fs)
    
    # Save results
    results = {
        'analysis_results': analysis_results,
        'report': report,
        'parameters': {
            'fs': fs,
            'detection_method': detection_method,
            'compensation_method': compensation_method,
            'kwargs': kwargs
        },
        'compensated_imu': {
            'accel': analysis_results['accel']['compensation']['filtered_data'] 
                    if analysis_results['accel']['vibration_compensated'] 
                    else imu_data['accel'],
            'gyro': analysis_results['gyro']['compensation']['filtered_data']
                   if analysis_results['gyro']['vibration_compensated']
                   else imu_data['gyro'],
            'time': time
        }
    }
    
    # Save results to file
    results_file = os.path.join(results_dir, 'task8_vibration_analysis.npz')
    np.savez_compressed(results_file, **results)
    print(f"Results saved to: {results_file}")
    
    return results


def _plot_vibration_analysis(time: np.ndarray, 
                           original_imu: Dict[str, np.ndarray],
                           analysis_results: Dict[str, Any],
                           results_dir: str,
                           fs: float) -> None:
    """Create comprehensive vibration analysis plots."""
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Task 8: IMU Vibration Analysis Results', fontsize=16, fontweight='bold')
    
    # Plot 1: Original vs Compensated Accelerometer
    ax = axes[0, 0]
    ax.plot(time, original_imu['accel'][:, 2], 'r-', linewidth=1, label='Original', alpha=0.8)
    
    if analysis_results['accel']['vibration_compensated']:
        compensated_accel = analysis_results['accel']['compensation']['filtered_data']
        ax.plot(time, compensated_accel[:, 2], 'b-', linewidth=1.5, label='Compensated')
    
    ax.set_title('Accelerometer Z-axis: Original vs Compensated')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Original vs Compensated Gyroscope  
    ax = axes[0, 1]
    ax.plot(time, original_imu['gyro'][:, 2], 'r-', linewidth=1, label='Original', alpha=0.8)
    
    if analysis_results['gyro']['vibration_compensated']:
        compensated_gyro = analysis_results['gyro']['compensation']['filtered_data']
        ax.plot(time, compensated_gyro[:, 2], 'b-', linewidth=1.5, label='Compensated')
    
    ax.set_title('Gyroscope Z-axis: Original vs Compensated')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Rate (rad/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Vibration Detection Results
    ax = axes[1, 0]
    accel_detection = analysis_results['accel']['detection']['vibration_detected']
    gyro_detection = analysis_results['gyro']['detection']['vibration_detected']
    
    ax.plot(time, accel_detection.astype(float), 'r-', linewidth=2, label='Accelerometer')
    ax.plot(time, gyro_detection.astype(float) + 0.1, 'b-', linewidth=2, label='Gyroscope')
    ax.set_title('Vibration Detection Results')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Detection Flag')
    ax.set_ylim([-0.1, 1.2])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Vibration Intensity
    ax = axes[1, 1]
    accel_intensity = analysis_results['accel']['detection'].get('vibration_intensity', 
                                                               np.zeros_like(time))
    gyro_intensity = analysis_results['gyro']['detection'].get('vibration_intensity',
                                                              np.zeros_like(time))
    
    if np.max(gyro_intensity) > 0:
        # Normalize gyro intensity to accelerometer scale
        gyro_intensity_norm = gyro_intensity / np.max(gyro_intensity) * np.max(accel_intensity)
    else:
        gyro_intensity_norm = gyro_intensity
    
    ax.plot(time, accel_intensity, 'r-', linewidth=1.5, label='Accel Intensity')
    ax.plot(time, gyro_intensity_norm, 'b-', linewidth=1.5, label='Gyro Intensity (norm)')
    ax.set_title('Vibration Intensity Over Time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Intensity')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 5: Power Spectral Density Comparison (Accelerometer)
    ax = axes[2, 0]
    from scipy import signal as sig
    
    f_orig, psd_orig = sig.welch(original_imu['accel'][:, 2], fs=fs, nperseg=min(1024, len(time)//4))
    ax.semilogy(f_orig, psd_orig, 'r-', label='Original', alpha=0.8)
    
    if analysis_results['accel']['vibration_compensated']:
        compensated_accel = analysis_results['accel']['compensation']['filtered_data']
        f_comp, psd_comp = sig.welch(compensated_accel[:, 2], fs=fs, nperseg=min(1024, len(time)//4))
        ax.semilogy(f_comp, psd_comp, 'b-', label='Compensated')
    
    ax.set_title('Power Spectral Density (Accel Z)')
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('PSD (m²/s⁴/Hz)')
    ax.set_xlim([0, min(200, fs/2)])
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 6: Summary Statistics
    ax = axes[2, 1]
    
    # Vibration ratios
    accel_ratio = analysis_results['accel']['vibration_ratio'] * 100
    gyro_ratio = analysis_results['gyro']['vibration_ratio'] * 100
    
    # Compensation status
    accel_compensated = analysis_results['accel']['vibration_compensated']
    gyro_compensated = analysis_results['gyro']['vibration_compensated']
    
    # Create bar plot
    sensors = ['Accelerometer', 'Gyroscope']
    vibration_ratios = [accel_ratio, gyro_ratio]
    compensation_status = [accel_compensated, gyro_compensated]
    
    colors = ['red' if not comp else 'blue' for comp in compensation_status]
    bars = ax.bar(sensors, vibration_ratios, color=colors, alpha=0.7)
    
    # Add labels
    for bar, ratio, comp in zip(bars, vibration_ratios, compensation_status):
        height = bar.get_height()
        label = f'{ratio:.1f}%\n{"Compensated" if comp else "No compensation"}'
        ax.text(bar.get_x() + bar.get_width()/2., height + 1,
               label, ha='center', va='bottom', fontsize=10)
    
    ax.set_title('Vibration Analysis Summary')
    ax.set_ylabel('Vibration Ratio (%)')
    ax.set_ylim([0, max(max(vibration_ratios) * 1.3, 10)])
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = os.path.join(results_dir, 'task8_vibration_analysis.png')
    plt.savefig(plot_file, dpi=150, bbox_inches='tight')
    print(f"Analysis plot saved to: {plot_file}")
    
    plt.close()


def simulate_imu_with_vibration(duration: float = 10.0, 
                              fs: float = 400.0,
                              vibration_type: str = 'motor',
                              **vibration_params) -> Dict[str, np.ndarray]:
    """
    Simulate IMU data with vibration for testing purposes.
    
    Args:
        duration: Simulation duration in seconds
        fs: Sampling frequency in Hz
        vibration_type: Type of vibration to simulate
        **vibration_params: Additional parameters for vibration model
        
    Returns:
        Dictionary with simulated IMU data including vibration
    """
    print(f"Simulating {duration} seconds of IMU data with {vibration_type} vibration...")
    
    # Create time vector
    t = np.arange(0, duration, 1/fs)
    n_samples = len(t)
    
    # Generate clean trajectory motion (low frequency)
    trajectory_freq = 0.2  # Hz
    clean_accel = np.column_stack([
        1.0 * np.sin(2 * np.pi * trajectory_freq * t),
        0.8 * np.cos(2 * np.pi * trajectory_freq * t),
        9.81 + 1.5 * np.sin(2 * np.pi * trajectory_freq * 0.5 * t)
    ])
    
    clean_gyro = np.column_stack([
        0.2 * np.cos(2 * np.pi * trajectory_freq * t),
        0.2 * np.sin(2 * np.pi * trajectory_freq * t), 
        0.1 * np.sin(2 * np.pi * trajectory_freq * 2 * t)
    ])
    
    # Generate vibration
    vibration_model = VibrationModel(fs=fs, seed=42)
    
    # Set default parameters if not provided
    default_params = {
        'motor': {'base_frequency': 75.0, 'amplitude_acc': 2.0, 'amplitude_gyro': 0.2},
        'rotor': {'rotor_frequency': 120.0, 'amplitude_acc': 2.5, 'amplitude_gyro': 0.25},
        'sinusoidal': {'frequency': 60.0, 'amplitude_acc': 1.5, 'amplitude_gyro': 0.15},
        'random': {'cutoff_frequency': 100.0, 'amplitude_acc': 1.0, 'amplitude_gyro': 0.1}
    }
    
    params = default_params.get(vibration_type, {})
    params.update(vibration_params)
    
    vibration_signals = vibration_model.generate_vibration(duration, vibration_type, **params)
    
    # Apply vibration to clean IMU data
    vibrated_imu = vibration_model.apply_vibration_to_imu(
        {'accel': clean_accel, 'gyro': clean_gyro, 'time': t},
        vibration_signals
    )
    
    print(f"Generated IMU data: {n_samples} samples at {fs} Hz")
    return vibrated_imu


# Example usage and integration with existing pipeline
def run_task8_example():
    """Run an example of Task 8 vibration analysis."""
    print("Running Task 8: IMU Vibration Analysis Example")
    print("=" * 55)
    
    # Create output directory
    results_dir = "PYTHON/results/task8_example"
    os.makedirs(results_dir, exist_ok=True)
    
    # Generate test data with vibration
    imu_data = simulate_imu_with_vibration(
        duration=15.0,
        fs=400.0,
        vibration_type='motor',
        base_frequency=85.0,
        amplitude_acc=2.5
    )
    
    # Run vibration analysis
    results = task8_vibration_analysis(
        imu_data,
        results_dir,
        fs=400.0,
        detection_method='frequency',
        compensation_method='butterworth',
        vibration_freq_range=(40, 200),  # For detection
        threshold_factor=1.5,            # Lower threshold for detection
        cutoff_freq=20.0                 # For compensation
    )
    
    print("\nTask 8 example completed successfully!")
    print(f"Results saved in: {results_dir}")
    
    return results


if __name__ == "__main__":
    # Run example when script is executed directly
    results = run_task8_example()
    print("\nTask 8: IMU Vibration Analysis completed successfully!")