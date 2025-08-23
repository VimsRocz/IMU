#!/usr/bin/env python3
"""
Vibration Compensation for IMU Data

This module provides various algorithms for detecting and compensating vibration
in IMU sensor data. It includes both simple detection methods and sophisticated
filtering techniques for vibration removal.

Author: IMU Vibration Project
"""

import numpy as np
from typing import Optional, Union, Tuple, Dict, Any
from scipy import signal
from scipy.stats import entropy
import warnings


class VibrationDetector:
    """
    Detects presence of vibration in IMU data using various statistical
    and frequency domain methods.
    """
    
    def __init__(self, fs: float = 400.0):
        """
        Initialize vibration detector.
        
        Args:
            fs: Sampling frequency in Hz
        """
        self.fs = fs
        self.dt = 1.0 / fs
        
    def detect_vibration_statistical(self, imu_data: np.ndarray,
                                   window_size: int = 200,
                                   threshold_factor: float = 2.0) -> Dict[str, Any]:
        """
        Detect vibration using statistical methods (variance-based).
        
        Args:
            imu_data: Nx3 IMU data (accelerometer or gyroscope)
            window_size: Size of sliding window for analysis
            threshold_factor: Multiplier for threshold determination
            
        Returns:
            Dictionary with vibration detection results
        """
        n_samples, n_axes = imu_data.shape
        vibration_detected = np.zeros(n_samples, dtype=bool)
        vibration_intensity = np.zeros(n_samples)
        
        # Compute baseline statistics from first portion of data
        baseline_samples = min(window_size * 2, n_samples // 4)
        baseline_var = np.var(imu_data[:baseline_samples], axis=0)
        threshold = threshold_factor * np.mean(baseline_var)
        
        # Sliding window analysis
        for i in range(window_size, n_samples):
            window_data = imu_data[i-window_size:i]
            window_var = np.var(window_data, axis=0)
            avg_var = np.mean(window_var)
            
            vibration_intensity[i] = avg_var / np.mean(baseline_var)
            vibration_detected[i] = avg_var > threshold
            
        return {
            'vibration_detected': vibration_detected,
            'vibration_intensity': vibration_intensity,
            'threshold': threshold,
            'baseline_variance': baseline_var
        }
    
    def detect_vibration_frequency(self, imu_data: np.ndarray,
                                 vibration_freq_range: Tuple[float, float] = (20.0, 200.0),
                                 window_size: int = 1024,
                                 overlap_ratio: float = 0.5,
                                 threshold_factor: float = 3.0) -> Dict[str, Any]:
        """
        Detect vibration using frequency domain analysis.
        
        Args:
            imu_data: Nx3 IMU data
            vibration_freq_range: Frequency range to look for vibrations (Hz)
            window_size: Window size for FFT analysis
            overlap_ratio: Overlap ratio for sliding windows
            threshold_factor: Multiplier for detection threshold
            
        Returns:
            Dictionary with frequency-based vibration detection results
        """
        n_samples, n_axes = imu_data.shape
        hop_size = int(window_size * (1 - overlap_ratio))
        n_windows = (n_samples - window_size) // hop_size + 1
        
        vibration_detected = np.zeros(n_samples, dtype=bool)
        dominant_frequency = np.zeros(n_samples)
        vibration_power = np.zeros(n_samples)
        
        # Define frequency bins
        freqs = np.fft.rfftfreq(window_size, self.dt)
        freq_mask = (freqs >= vibration_freq_range[0]) & (freqs <= vibration_freq_range[1])
        
        # Analyze each window
        for win_idx in range(n_windows):
            start_idx = win_idx * hop_size
            end_idx = start_idx + window_size
            
            if end_idx > n_samples:
                break
                
            window_data = imu_data[start_idx:end_idx]
            
            # Compute power spectral density
            total_vibration_power = 0
            peak_frequencies = []
            
            for axis in range(n_axes):
                # Apply windowing to reduce spectral leakage
                windowed_data = window_data[:, axis] * signal.windows.hann(window_size)
                
                # Compute FFT
                fft_data = np.fft.rfft(windowed_data)
                psd = np.abs(fft_data) ** 2
                
                # Extract vibration frequency range
                vibration_psd = psd[freq_mask]
                total_vibration_power += np.sum(vibration_psd)
                
                # Find dominant frequency in vibration range
                if len(vibration_psd) > 0:
                    peak_idx = np.argmax(vibration_psd)
                    peak_freq = freqs[freq_mask][peak_idx]
                    peak_frequencies.append(peak_freq)
            
            # Compute baseline power (DC and very low frequencies)
            baseline_mask = freqs < vibration_freq_range[0]
            baseline_power = np.sum([np.sum(np.abs(np.fft.rfft(
                window_data[:, axis] * signal.windows.hann(window_size)))[baseline_mask] ** 2)
                                   for axis in range(n_axes)])
            
            # Detection logic
            vibration_ratio = total_vibration_power / (baseline_power + 1e-10)
            is_vibration = vibration_ratio > threshold_factor
            
            # Fill results for this window
            center_idx = start_idx + window_size // 2
            if center_idx < n_samples:
                vibration_detected[start_idx:end_idx] = is_vibration
                vibration_power[center_idx] = total_vibration_power
                if peak_frequencies:
                    dominant_frequency[center_idx] = np.mean(peak_frequencies)
        
        return {
            'vibration_detected': vibration_detected,
            'vibration_power': vibration_power,
            'dominant_frequency': dominant_frequency,
            'frequency_range': vibration_freq_range
        }
    
    def detect_vibration_entropy(self, imu_data: np.ndarray,
                               window_size: int = 200,
                               entropy_threshold: float = 0.7) -> Dict[str, Any]:
        """
        Detect vibration using spectral entropy analysis.
        
        Args:
            imu_data: Nx3 IMU data
            window_size: Size of analysis window
            entropy_threshold: Threshold for entropy-based detection
            
        Returns:
            Dictionary with entropy-based vibration detection results
        """
        n_samples, n_axes = imu_data.shape
        vibration_detected = np.zeros(n_samples, dtype=bool)
        spectral_entropy = np.zeros(n_samples)
        
        for i in range(window_size, n_samples):
            window_data = imu_data[i-window_size:i]
            
            total_entropy = 0
            for axis in range(n_axes):
                # Compute power spectral density
                freqs, psd = signal.welch(window_data[:, axis], 
                                        fs=self.fs, nperseg=min(128, window_size//2))
                
                # Normalize PSD to create probability distribution
                psd_norm = psd / (np.sum(psd) + 1e-10)
                
                # Compute entropy
                total_entropy += entropy(psd_norm + 1e-10)
            
            avg_entropy = total_entropy / n_axes
            spectral_entropy[i] = avg_entropy
            
            # High entropy indicates white noise or vibration
            # Low entropy indicates periodic or tonal signals
            vibration_detected[i] = avg_entropy > entropy_threshold
            
        return {
            'vibration_detected': vibration_detected,
            'spectral_entropy': spectral_entropy,
            'entropy_threshold': entropy_threshold
        }


class VibrationCompensator:
    """
    Compensates for vibration effects in IMU data using various filtering
    and signal processing techniques.
    """
    
    def __init__(self, fs: float = 400.0):
        """
        Initialize vibration compensator.
        
        Args:
            fs: Sampling frequency in Hz
        """
        self.fs = fs
        self.dt = 1.0 / fs
        
    def compensate_butterworth_filter(self, imu_data: np.ndarray,
                                    cutoff_freq: float = 20.0,
                                    filter_order: int = 4,
                                    filter_type: str = 'lowpass') -> Dict[str, Any]:
        """
        Compensate vibration using Butterworth filter.
        
        Args:
            imu_data: Nx3 IMU data
            cutoff_freq: Filter cutoff frequency in Hz
            filter_order: Filter order
            filter_type: 'lowpass', 'highpass', or 'bandstop'
            
        Returns:
            Dictionary with filtered data and filter information
        """
        nyquist = 0.5 * self.fs
        
        if filter_type in ['lowpass', 'highpass']:
            normal_cutoff = cutoff_freq / nyquist
            b, a = signal.butter(filter_order, normal_cutoff, btype=filter_type)
        else:
            raise ValueError(f"Unsupported filter type: {filter_type}")
        
        # Apply zero-phase filtering
        filtered_data = np.zeros_like(imu_data)
        for axis in range(imu_data.shape[1]):
            filtered_data[:, axis] = signal.filtfilt(b, a, imu_data[:, axis])
        
        return {
            'filtered_data': filtered_data,
            'cutoff_frequency': cutoff_freq,
            'filter_order': filter_order,
            'filter_type': filter_type
        }
    
    def compensate_notch_filter(self, imu_data: np.ndarray,
                              notch_freq: float,
                              quality_factor: float = 30.0) -> Dict[str, Any]:
        """
        Compensate specific frequency vibrations using notch filter.
        
        Args:
            imu_data: Nx3 IMU data
            notch_freq: Frequency to notch out in Hz
            quality_factor: Quality factor of the notch filter
            
        Returns:
            Dictionary with filtered data and filter information
        """
        # Design notch filter
        b, a = signal.iirnotch(notch_freq, quality_factor, fs=self.fs)
        
        # Apply filtering
        filtered_data = np.zeros_like(imu_data)
        for axis in range(imu_data.shape[1]):
            filtered_data[:, axis] = signal.filtfilt(b, a, imu_data[:, axis])
        
        return {
            'filtered_data': filtered_data,
            'notch_frequency': notch_freq,
            'quality_factor': quality_factor
        }
    
    def compensate_adaptive_filter(self, imu_data: np.ndarray,
                                 reference_signal: Optional[np.ndarray] = None,
                                 filter_length: int = 32,
                                 step_size: float = 0.01) -> Dict[str, Any]:
        """
        Compensate vibration using adaptive filtering (LMS algorithm).
        
        Args:
            imu_data: Nx3 IMU data
            reference_signal: Reference signal for adaptive filtering
            filter_length: Length of adaptive filter
            step_size: LMS algorithm step size
            
        Returns:
            Dictionary with compensated data and filter information
        """
        n_samples, n_axes = imu_data.shape
        
        if reference_signal is None:
            # Use delayed version of signal as reference (for self-adaptive filtering)
            delay = filter_length // 2
            reference_signal = np.roll(imu_data, delay, axis=0)
        
        compensated_data = np.zeros_like(imu_data)
        filter_weights = np.zeros((n_axes, filter_length))
        
        for axis in range(n_axes):
            # Initialize filter weights
            w = np.zeros(filter_length)
            
            # LMS adaptive filtering
            for n in range(filter_length, n_samples):
                # Extract input vector
                x = reference_signal[n-filter_length:n, axis][::-1]  # Reverse for convolution
                
                # Filter output
                y = np.dot(w, x)
                
                # Error signal
                e = imu_data[n, axis] - y
                compensated_data[n, axis] = e
                
                # Update filter weights
                w += step_size * e * x
            
            filter_weights[axis] = w
        
        return {
            'compensated_data': compensated_data,
            'filter_weights': filter_weights,
            'filter_length': filter_length,
            'step_size': step_size
        }
    
    def compensate_wavelet_denoising(self, imu_data: np.ndarray,
                                   wavelet: str = 'db4',
                                   threshold_mode: str = 'soft',
                                   sigma: Optional[float] = None) -> Dict[str, Any]:
        """
        Compensate vibration using wavelet denoising.
        
        Args:
            imu_data: Nx3 IMU data
            wavelet: Wavelet type for decomposition
            threshold_mode: 'soft' or 'hard' thresholding
            sigma: Noise standard deviation (estimated if None)
            
        Returns:
            Dictionary with denoised data
        """
        try:
            import pywt
        except ImportError:
            raise ImportError("PyWavelets is required for wavelet denoising. Install with: pip install PyWavelets")
        
        denoised_data = np.zeros_like(imu_data)
        noise_levels = np.zeros(imu_data.shape[1])
        
        for axis in range(imu_data.shape[1]):
            signal_data = imu_data[:, axis]
            
            # Estimate noise level if not provided
            if sigma is None:
                # Use median absolute deviation of finest detail coefficients
                coeffs = pywt.wavedec(signal_data, wavelet, level=6)
                sigma_est = np.median(np.abs(coeffs[-1])) / 0.6745
                noise_levels[axis] = sigma_est
            else:
                sigma_est = sigma
                noise_levels[axis] = sigma
            
            # Wavelet denoising
            denoised_data[:, axis] = pywt.threshold(signal_data, 
                                                   sigma_est * np.sqrt(2 * np.log(len(signal_data))),
                                                   mode=threshold_mode)
        
        return {
            'denoised_data': denoised_data,
            'wavelet': wavelet,
            'threshold_mode': threshold_mode,
            'estimated_noise_levels': noise_levels
        }
    
    def compensate_kalman_filter(self, imu_data: np.ndarray,
                               process_noise: float = 1e-4,
                               measurement_noise: float = 1e-2) -> Dict[str, Any]:
        """
        Compensate vibration using Kalman filtering.
        
        Args:
            imu_data: Nx3 IMU data
            process_noise: Process noise variance
            measurement_noise: Measurement noise variance
            
        Returns:
            Dictionary with Kalman-filtered data
        """
        n_samples, n_axes = imu_data.shape
        filtered_data = np.zeros_like(imu_data)
        
        for axis in range(n_axes):
            # Simple Kalman filter for 1D signal
            x = np.zeros(2)  # State: [position, velocity]
            P = np.eye(2)    # Covariance matrix
            
            # State transition matrix (constant velocity model)
            F = np.array([[1, self.dt],
                         [0, 1]])
            
            # Measurement matrix (observe position only)
            H = np.array([[1, 0]])
            
            # Process noise covariance
            Q = process_noise * np.array([[self.dt**4/4, self.dt**3/2],
                                        [self.dt**3/2, self.dt**2]])
            
            # Measurement noise covariance
            R = np.array([[measurement_noise]])
            
            # Kalman filtering
            for k in range(n_samples):
                # Prediction step
                x = F @ x
                P = F @ P @ F.T + Q
                
                # Update step
                z = np.array([imu_data[k, axis]])  # Measurement
                y = z - H @ x                      # Innovation
                S = H @ P @ H.T + R               # Innovation covariance
                K = P @ H.T @ np.linalg.inv(S)    # Kalman gain
                
                x = x + K @ y
                P = P - K @ H @ P
                
                filtered_data[k, axis] = x[0]
        
        return {
            'filtered_data': filtered_data,
            'process_noise': process_noise,
            'measurement_noise': measurement_noise
        }


class VibrationAnalyzer:
    """
    Comprehensive vibration analysis combining detection and compensation.
    """
    
    def __init__(self, fs: float = 400.0):
        """
        Initialize vibration analyzer.
        
        Args:
            fs: Sampling frequency in Hz
        """
        self.fs = fs
        self.detector = VibrationDetector(fs)
        self.compensator = VibrationCompensator(fs)
        
    def analyze_and_compensate(self, imu_data: Dict[str, np.ndarray],
                             detection_method: str = 'frequency',
                             compensation_method: str = 'butterworth',
                             detection_params: Dict[str, Any] = None,
                             compensation_params: Dict[str, Any] = None,
                             **kwargs) -> Dict[str, Any]:
        """
        Perform comprehensive vibration analysis and compensation.
        
        Args:
            imu_data: Dictionary with 'accel' and/or 'gyro' keys
            detection_method: Method for vibration detection
            compensation_method: Method for vibration compensation
            detection_params: Parameters specific to detection method
            compensation_params: Parameters specific to compensation method
            **kwargs: Backwards compatibility - will be used for both detection and compensation
            
        Returns:
            Dictionary with analysis results and compensated data
        """
        # Handle backwards compatibility
        if detection_params is None:
            detection_params = kwargs.copy()
        if compensation_params is None:
            compensation_params = kwargs.copy()
        
        results = {}
        
        for sensor_type in ['accel', 'gyro']:
            if sensor_type not in imu_data:
                continue
                
            sensor_data = imu_data[sensor_type]
            sensor_results = {}
            
            # Vibration detection
            if detection_method == 'statistical':
                detection_result = self.detector.detect_vibration_statistical(sensor_data, **detection_params)
            elif detection_method == 'frequency':
                detection_result = self.detector.detect_vibration_frequency(sensor_data, **detection_params)
            elif detection_method == 'entropy':
                detection_result = self.detector.detect_vibration_entropy(sensor_data, **detection_params)
            else:
                raise ValueError(f"Unknown detection method: {detection_method}")
            
            sensor_results['detection'] = detection_result
            
            # Vibration compensation (only if vibration detected)
            vibration_detected = detection_result['vibration_detected']
            vibration_ratio = np.sum(vibration_detected) / len(vibration_detected)
            
            if vibration_ratio > 0.1:  # Compensate if >10% of data has vibration
                if compensation_method == 'butterworth':
                    compensation_result = self.compensator.compensate_butterworth_filter(sensor_data, **compensation_params)
                elif compensation_method == 'notch':
                    compensation_result = self.compensator.compensate_notch_filter(sensor_data, **compensation_params)
                elif compensation_method == 'adaptive':
                    compensation_result = self.compensator.compensate_adaptive_filter(sensor_data, **compensation_params)
                elif compensation_method == 'wavelet':
                    compensation_result = self.compensator.compensate_wavelet_denoising(sensor_data, **compensation_params)
                elif compensation_method == 'kalman':
                    compensation_result = self.compensator.compensate_kalman_filter(sensor_data, **compensation_params)
                else:
                    raise ValueError(f"Unknown compensation method: {compensation_method}")
                
                sensor_results['compensation'] = compensation_result
                sensor_results['vibration_compensated'] = True
            else:
                sensor_results['compensation'] = {'filtered_data': sensor_data}
                sensor_results['vibration_compensated'] = False
            
            sensor_results['vibration_ratio'] = vibration_ratio
            results[sensor_type] = sensor_results
        
        return results
    
    def get_vibration_report(self, analysis_results: Dict[str, Any]) -> str:
        """
        Generate a human-readable vibration analysis report.
        
        Args:
            analysis_results: Results from analyze_and_compensate()
            
        Returns:
            Formatted string report
        """
        report = "=== IMU Vibration Analysis Report ===\n\n"
        
        for sensor_type, sensor_results in analysis_results.items():
            report += f"{sensor_type.upper()} Sensor Analysis:\n"
            report += f"  Vibration Ratio: {sensor_results['vibration_ratio']:.1%}\n"
            
            detection = sensor_results['detection']
            if 'dominant_frequency' in detection:
                dom_freq = np.mean(detection['dominant_frequency'][detection['dominant_frequency'] > 0])
                if not np.isnan(dom_freq):
                    report += f"  Dominant Vibration Frequency: {dom_freq:.1f} Hz\n"
            
            if sensor_results['vibration_compensated']:
                report += f"  Vibration Compensation: Applied\n"
            else:
                report += f"  Vibration Compensation: Not needed\n"
            
            report += "\n"
        
        return report


# Utility functions for easy usage
def detect_vibration_simple(imu_data: np.ndarray, 
                          fs: float = 400.0,
                          method: str = 'statistical') -> bool:
    """
    Simple vibration detection returning boolean result.
    
    Args:
        imu_data: Nx3 IMU data array
        fs: Sampling frequency in Hz
        method: Detection method ('statistical', 'frequency', 'entropy')
        
    Returns:
        True if vibration detected, False otherwise
    """
    detector = VibrationDetector(fs)
    
    if method == 'statistical':
        result = detector.detect_vibration_statistical(imu_data)
    elif method == 'frequency':
        result = detector.detect_vibration_frequency(imu_data)
    elif method == 'entropy':
        result = detector.detect_vibration_entropy(imu_data)
    else:
        raise ValueError(f"Unknown method: {method}")
    
    # Return True if significant portion of data shows vibration
    return np.sum(result['vibration_detected']) > len(result['vibration_detected']) * 0.1


def compensate_vibration_simple(imu_data: np.ndarray,
                              fs: float = 400.0,
                              method: str = 'butterworth',
                              cutoff_freq: float = 20.0) -> np.ndarray:
    """
    Simple vibration compensation returning filtered data.
    
    Args:
        imu_data: Nx3 IMU data array
        fs: Sampling frequency in Hz  
        method: Compensation method ('butterworth', 'notch')
        cutoff_freq: Cutoff frequency for filtering in Hz
        
    Returns:
        Filtered IMU data array
    """
    compensator = VibrationCompensator(fs)
    
    if method == 'butterworth':
        result = compensator.compensate_butterworth_filter(imu_data, cutoff_freq=cutoff_freq)
    elif method == 'notch':
        result = compensator.compensate_notch_filter(imu_data, notch_freq=cutoff_freq)
    else:
        raise ValueError(f"Unknown method: {method}")
    
    return result['filtered_data']