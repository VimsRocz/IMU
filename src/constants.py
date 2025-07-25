"""Shared physical constants for the navigation algorithms.

GRAVITY
    Standard gravity expressed in metres per second squared (m/s^2).  This
    value is used when normalising accelerometer measurements.

EARTH_RATE
    Earth's angular rotation rate expressed in radians per second (rad/s).
    It is required for inertial navigation computations.

These constants mirror those defined in ``MATLAB/constants.m`` to keep the
Python and MATLAB implementations in sync.
"""

GRAVITY = 9.81
EARTH_RATE = 7.2921e-5
