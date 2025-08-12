"""Shared physical constants for the navigation algorithms.

GRAVITY
    Typical magnitude of gravitational acceleration near Earth's surface
    in metres per second squared (m/s^2).  Local gravity varies with
    location, so algorithms may compute a more precise value; this default
    is used when none is provided.

EARTH_RATE
    Earth's angular rotation rate expressed in radians per second (rad/s).
    It is required for inertial navigation computations.

These constants mirror those defined in ``MATLAB/constants.m`` so that the
Python and MATLAB implementations remain in sync.
"""

# Gravity magnitude used throughout the Python pipeline (m/s^2)
# Mirrors MATLAB/constants.m for cross-language parity
GRAVITY = 9.79424753
EARTH_RATE = 7.2921e-5
