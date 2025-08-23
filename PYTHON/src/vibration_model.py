#!/usr/bin/env python3
"""
Vibration Model for IMU Simulation

This module provides vibration models that can be used with IMU sensors to simulate
realistic vibration effects on accelerometer and gyroscope measurements. The vibration
models support various types of vibrations including sinusoidal, random, and complex
realistic patterns.

Author: IMU Vibration Project
"""

import numpy as np
from typing import Optional, Union, Tuple, Dict, Any
from scipy import signal


class VibrationModel:
    """
    Base class for IMU vibration modeling.
    
    This class provides the framework for generating vibration signals that can be
    applied to IMU sensor outputs to simulate realistic vibration conditions.
    """
    
    def __init__(self, fs: float = 400.0, seed: Optional[int] = None):
        """
        Initialize vibration model.
        
        Args:
            fs: Sampling frequency in Hz (default: 400 Hz)
            seed: Random seed for reproducible results
        """
        self.fs = fs
        self.dt = 1.0 / fs
        if seed is not None:
            np.random.seed(seed)
            
    def generate_vibration(self, duration: float, 
                          vibration_type: str = 'sinusoidal',
                          **kwargs) -> Dict[str, np.ndarray]:
        """
        Generate vibration signals for accelerometer and gyroscope.
        
        Args:
            duration: Duration in seconds
            vibration_type: Type of vibration ('sinusoidal', 'random', 'motor', 'rotor')
            **kwargs: Additional parameters specific to vibration type
            
        Returns:
            Dictionary with 'accel_vib' and 'gyro_vib' keys containing vibration signals
        """
        n_samples = int(duration * self.fs)
        
        if vibration_type == 'sinusoidal':
            return self._generate_sinusoidal_vibration(n_samples, **kwargs)
        elif vibration_type == 'random':
            return self._generate_random_vibration(n_samples, **kwargs)
        elif vibration_type == 'motor':
            return self._generate_motor_vibration(n_samples, **kwargs)
        elif vibration_type == 'rotor':
            return self._generate_rotor_vibration(n_samples, **kwargs)
        else:
            raise ValueError(f"Unknown vibration type: {vibration_type}")
    
    def _generate_sinusoidal_vibration(self, n_samples: int, 
                                     frequency: float = 50.0,
                                     amplitude_acc: float = 1.0,
                                     amplitude_gyro: float = 0.1,
                                     phase_acc: Optional[np.ndarray] = None,
                                     phase_gyro: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """
        Generate sinusoidal vibration signals.
        
        Args:
            n_samples: Number of samples
            frequency: Vibration frequency in Hz
            amplitude_acc: Accelerometer vibration amplitude in m/s²
            amplitude_gyro: Gyroscope vibration amplitude in rad/s
            phase_acc: Phase offset for accelerometer (3-element array)
            phase_gyro: Phase offset for gyroscope (3-element array)
            
        Returns:
            Dictionary with accelerometer and gyroscope vibration signals
        """
        t = np.arange(n_samples) * self.dt
        
        # Default phase offsets
        if phase_acc is None:
            phase_acc = np.array([0.0, np.pi/3, 2*np.pi/3])  # 120° apart
        if phase_gyro is None:
            phase_gyro = np.array([np.pi/4, np.pi/2, 3*np.pi/4])
            
        # Generate sinusoidal vibrations
        accel_vib = np.zeros((n_samples, 3))
        gyro_vib = np.zeros((n_samples, 3))
        
        for axis in range(3):
            accel_vib[:, axis] = amplitude_acc * np.sin(2 * np.pi * frequency * t + phase_acc[axis])
            gyro_vib[:, axis] = amplitude_gyro * np.sin(2 * np.pi * frequency * t + phase_gyro[axis])
            
        return {'accel_vib': accel_vib, 'gyro_vib': gyro_vib}
    
    def _generate_random_vibration(self, n_samples: int,
                                 cutoff_frequency: float = 100.0,
                                 amplitude_acc: float = 0.5,
                                 amplitude_gyro: float = 0.05) -> Dict[str, np.ndarray]:
        """
        Generate random vibration signals with specified spectral characteristics.
        
        Args:
            n_samples: Number of samples
            cutoff_frequency: Low-pass filter cutoff frequency in Hz
            amplitude_acc: Accelerometer vibration RMS amplitude in m/s²
            amplitude_gyro: Gyroscope vibration RMS amplitude in rad/s
            
        Returns:
            Dictionary with accelerometer and gyroscope vibration signals
        """
        # Generate white noise
        accel_noise = np.random.normal(0, 1, (n_samples, 3))
        gyro_noise = np.random.normal(0, 1, (n_samples, 3))
        
        # Design low-pass filter
        nyquist = 0.5 * self.fs
        normal_cutoff = cutoff_frequency / nyquist
        b, a = signal.butter(4, normal_cutoff, btype='low', analog=False)
        
        # Filter and scale noise
        accel_vib = np.zeros((n_samples, 3))
        gyro_vib = np.zeros((n_samples, 3))
        
        for axis in range(3):
            # Filter noise
            accel_filtered = signal.filtfilt(b, a, accel_noise[:, axis])
            gyro_filtered = signal.filtfilt(b, a, gyro_noise[:, axis])
            
            # Scale to desired amplitude
            accel_vib[:, axis] = amplitude_acc * accel_filtered / np.std(accel_filtered)
            gyro_vib[:, axis] = amplitude_gyro * gyro_filtered / np.std(gyro_filtered)
            
        return {'accel_vib': accel_vib, 'gyro_vib': gyro_vib}
    
    def _generate_motor_vibration(self, n_samples: int,
                                base_frequency: float = 60.0,
                                harmonics: Optional[list] = None,
                                amplitude_acc: float = 2.0,
                                amplitude_gyro: float = 0.2) -> Dict[str, np.ndarray]:
        """
        Generate motor-like vibration with harmonics.
        
        Args:
            n_samples: Number of samples
            base_frequency: Base motor frequency in Hz
            harmonics: List of harmonic multipliers (default: [1, 2, 3, 4])
            amplitude_acc: Base accelerometer amplitude in m/s²
            amplitude_gyro: Base gyroscope amplitude in rad/s
            
        Returns:
            Dictionary with accelerometer and gyroscope vibration signals
        """
        if harmonics is None:
            harmonics = [1.0, 0.5, 0.3, 0.2]  # Decreasing harmonic amplitudes
            
        t = np.arange(n_samples) * self.dt
        
        accel_vib = np.zeros((n_samples, 3))
        gyro_vib = np.zeros((n_samples, 3))
        
        # Different axis orientations for motor vibration
        axis_factors = np.array([[1.0, 0.7, 0.3],  # X-axis dominant
                                [0.3, 1.0, 0.7],   # Y-axis dominant  
                                [0.7, 0.3, 1.0]])  # Z-axis dominant
        
        for i, harmonic_amp in enumerate(harmonics):
            freq = base_frequency * (i + 1)
            for axis in range(3):
                phase = np.random.uniform(0, 2*np.pi)
                factor = axis_factors[axis, axis]
                
                accel_vib[:, axis] += (amplitude_acc * harmonic_amp * factor * 
                                     np.sin(2 * np.pi * freq * t + phase))
                gyro_vib[:, axis] += (amplitude_gyro * harmonic_amp * factor * 
                                    np.sin(2 * np.pi * freq * t + phase + np.pi/4))
                                    
        return {'accel_vib': accel_vib, 'gyro_vib': gyro_vib}
    
    def _generate_rotor_vibration(self, n_samples: int,
                                rotor_frequency: float = 100.0,
                                num_blades: int = 4,
                                amplitude_acc: float = 3.0,
                                amplitude_gyro: float = 0.3) -> Dict[str, np.ndarray]:
        """
        Generate quadrotor-like vibration patterns.
        
        Args:
            n_samples: Number of samples
            rotor_frequency: Rotor frequency in Hz
            num_blades: Number of rotor blades
            amplitude_acc: Accelerometer vibration amplitude in m/s²
            amplitude_gyro: Gyroscope vibration amplitude in rad/s
            
        Returns:
            Dictionary with accelerometer and gyroscope vibration signals
        """
        t = np.arange(n_samples) * self.dt
        
        # Blade passing frequency
        blade_freq = rotor_frequency * num_blades
        
        accel_vib = np.zeros((n_samples, 3))
        gyro_vib = np.zeros((n_samples, 3))
        
        # Rotor vibration affects different axes differently
        # Z-axis (vertical) most affected by thrust variations
        # X,Y axes affected by rotor imbalance
        
        # Z-axis: dominant blade passing frequency
        accel_vib[:, 2] = amplitude_acc * np.sin(2 * np.pi * blade_freq * t)
        
        # X,Y axes: rotor frequency with some harmonics
        for axis in [0, 1]:
            phase = axis * np.pi/2  # 90° phase difference between X and Y
            accel_vib[:, axis] = (amplitude_acc * 0.6 * 
                                np.sin(2 * np.pi * rotor_frequency * t + phase))
            
        # Gyro vibration couples with acceleration through rotor dynamics
        for axis in range(3):
            # Add some coupling between axes
            coupling_factor = 0.3 if axis == 2 else 1.0
            gyro_vib[:, axis] = (amplitude_gyro * coupling_factor * 
                               np.sin(2 * np.pi * rotor_frequency * t + 
                                     axis * np.pi/3 + np.pi/6))
                                     
        return {'accel_vib': accel_vib, 'gyro_vib': gyro_vib}
    
    def apply_vibration_to_imu(self, imu_data: Dict[str, np.ndarray],
                              vibration_signals: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Apply vibration signals to clean IMU data.
        
        Args:
            imu_data: Dictionary with 'accel' and 'gyro' keys
            vibration_signals: Dictionary with 'accel_vib' and 'gyro_vib' keys
            
        Returns:
            Dictionary with vibration-corrupted IMU data
        """
        corrupted_data = {}
        
        # Apply vibration to accelerometer
        if 'accel' in imu_data and 'accel_vib' in vibration_signals:
            corrupted_data['accel'] = imu_data['accel'] + vibration_signals['accel_vib']
        else:
            corrupted_data['accel'] = imu_data.get('accel', np.array([]))
            
        # Apply vibration to gyroscope
        if 'gyro' in imu_data and 'gyro_vib' in vibration_signals:
            corrupted_data['gyro'] = imu_data['gyro'] + vibration_signals['gyro_vib']
        else:
            corrupted_data['gyro'] = imu_data.get('gyro', np.array([]))
            
        # Copy other fields unchanged
        for key in imu_data:
            if key not in ['accel', 'gyro']:
                corrupted_data[key] = imu_data[key]
                
        return corrupted_data


class WaypointVibrationSimulator:
    """
    Simulates IMU with vibration along a waypoint trajectory.
    
    This class combines waypoint trajectory generation with vibration modeling
    to simulate realistic IMU data for a moving device subject to vibration.
    """
    
    def __init__(self, vibration_model: VibrationModel):
        """
        Initialize waypoint vibration simulator.
        
        Args:
            vibration_model: VibrationModel instance
        """
        self.vibration_model = vibration_model
        
    def generate_trajectory_with_vibration(self, waypoints: np.ndarray,
                                         times: np.ndarray,
                                         vibration_params: Dict[str, Any]) -> Dict[str, np.ndarray]:
        """
        Generate IMU data along a trajectory with vibration.
        
        Args:
            waypoints: Nx3 array of position waypoints [x, y, z] in meters
            times: N-element array of times at each waypoint in seconds  
            vibration_params: Parameters for vibration generation
            
        Returns:
            Dictionary with simulated IMU data including vibration effects
        """
        # Generate smooth trajectory between waypoints
        trajectory_data = self._generate_smooth_trajectory(waypoints, times)
        
        # Generate clean IMU signals from trajectory
        clean_imu = self._trajectory_to_imu(trajectory_data)
        
        # Generate vibration signals
        duration = times[-1] - times[0]
        vibration_signals = self.vibration_model.generate_vibration(duration, **vibration_params)
        
        # Apply vibration to clean IMU data
        vibrated_imu = self.vibration_model.apply_vibration_to_imu(clean_imu, vibration_signals)
        
        # Add trajectory information
        vibrated_imu.update(trajectory_data)
        
        return vibrated_imu
    
    def _generate_smooth_trajectory(self, waypoints: np.ndarray, 
                                  times: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Generate smooth trajectory between waypoints using spline interpolation.
        
        Args:
            waypoints: Nx3 array of position waypoints
            times: N-element array of waypoint times
            
        Returns:
            Dictionary with position, velocity, and acceleration trajectories
        """
        from scipy.interpolate import interp1d
        
        # Create time vector for output
        t_start, t_end = times[0], times[-1]
        t_out = np.arange(t_start, t_end, self.vibration_model.dt)
        n_samples = len(t_out)
        
        # For small number of waypoints, use linear interpolation
        if len(waypoints) < 4:
            interp_kind = 'linear'
        else:
            interp_kind = 'cubic'
        
        # Interpolate position using splines
        f_pos = interp1d(times, waypoints, axis=0, kind=interp_kind, 
                        bounds_error=False, fill_value='extrapolate')
        position = f_pos(t_out)
        
        # Compute velocity and acceleration numerically
        velocity = np.gradient(position, self.vibration_model.dt, axis=0)
        acceleration = np.gradient(velocity, self.vibration_model.dt, axis=0)
        
        return {
            'time': t_out,
            'position': position,
            'velocity': velocity, 
            'acceleration': acceleration
        }
    
    def _trajectory_to_imu(self, trajectory_data: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Convert trajectory data to clean IMU measurements.
        
        Args:
            trajectory_data: Dictionary with trajectory information
            
        Returns:
            Dictionary with clean IMU accelerometer and gyroscope data
        """
        time = trajectory_data['time']
        acceleration = trajectory_data['acceleration']
        
        # For simplicity, assume body frame aligns with NED frame
        # In practice, this would involve proper coordinate transformations
        
        # Add gravity to acceleration (assuming Z-down convention)
        g_ned = np.array([0, 0, 9.81])  # Gravity in NED frame
        accel_body = acceleration + g_ned
        
        # Generate gyroscope data from trajectory curvature
        # This is a simplified model - real implementation would be more complex
        velocity = trajectory_data['velocity']
        speed = np.linalg.norm(velocity, axis=1)
        
        # Avoid division by zero
        speed_safe = np.maximum(speed, 1e-6)
        
        # Compute angular rates from trajectory curvature (simplified)
        gyro_body = np.zeros_like(acceleration)
        
        # Yaw rate from horizontal velocity direction changes
        vel_horizontal = velocity[:, :2]  # X,Y components
        vel_horizontal_norm = np.linalg.norm(vel_horizontal, axis=1)
        vel_horizontal_norm_safe = np.maximum(vel_horizontal_norm, 1e-6)
        
        # Angular velocity around Z-axis (yaw)
        for i in range(1, len(time)):
            dt = time[i] - time[i-1]
            if dt > 0:
                # Change in heading angle
                v_prev = vel_horizontal[i-1] / vel_horizontal_norm_safe[i-1]
                v_curr = vel_horizontal[i] / vel_horizontal_norm_safe[i]
                
                # Cross product gives yaw rate magnitude
                cross = v_prev[0] * v_curr[1] - v_prev[1] * v_curr[0]
                gyro_body[i, 2] = np.arcsin(np.clip(cross, -1, 1)) / dt
        
        return {
            'time': time,
            'accel': accel_body,
            'gyro': gyro_body
        }


# Utility functions for easy usage
def create_sinusoidal_vibration_model(fs: float = 400.0, 
                                    frequency: float = 50.0,
                                    amplitude_acc: float = 1.0,
                                    amplitude_gyro: float = 0.1) -> VibrationModel:
    """
    Create a preconfigured sinusoidal vibration model.
    
    Args:
        fs: Sampling frequency in Hz
        frequency: Vibration frequency in Hz  
        amplitude_acc: Accelerometer amplitude in m/s²
        amplitude_gyro: Gyroscope amplitude in rad/s
        
    Returns:
        Configured VibrationModel instance
    """
    model = VibrationModel(fs=fs)
    model.default_params = {
        'vibration_type': 'sinusoidal',
        'frequency': frequency,
        'amplitude_acc': amplitude_acc, 
        'amplitude_gyro': amplitude_gyro
    }
    return model


def create_motor_vibration_model(fs: float = 400.0,
                               base_frequency: float = 60.0,
                               amplitude_acc: float = 2.0,
                               amplitude_gyro: float = 0.2) -> VibrationModel:
    """
    Create a preconfigured motor vibration model.
    
    Args:
        fs: Sampling frequency in Hz
        base_frequency: Motor base frequency in Hz
        amplitude_acc: Accelerometer amplitude in m/s²
        amplitude_gyro: Gyroscope amplitude in rad/s
        
    Returns:
        Configured VibrationModel instance
    """
    model = VibrationModel(fs=fs)
    model.default_params = {
        'vibration_type': 'motor',
        'base_frequency': base_frequency,
        'amplitude_acc': amplitude_acc,
        'amplitude_gyro': amplitude_gyro
    }
    return model


def create_quadrotor_vibration_model(fs: float = 400.0,
                                   rotor_frequency: float = 100.0,
                                   amplitude_acc: float = 3.0,
                                   amplitude_gyro: float = 0.3) -> VibrationModel:
    """
    Create a preconfigured quadrotor vibration model.
    
    Args:
        fs: Sampling frequency in Hz
        rotor_frequency: Rotor frequency in Hz
        amplitude_acc: Accelerometer amplitude in m/s²
        amplitude_gyro: Gyroscope amplitude in rad/s
        
    Returns:
        Configured VibrationModel instance
    """
    model = VibrationModel(fs=fs)
    model.default_params = {
        'vibration_type': 'rotor',
        'rotor_frequency': rotor_frequency,
        'amplitude_acc': amplitude_acc,
        'amplitude_gyro': amplitude_gyro
    }
    return model