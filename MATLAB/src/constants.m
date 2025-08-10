classdef constants
    % Mirrors constants defined in src/constants.py for cross-language parity
    % GRAVITY is a typical magnitude of gravitational acceleration (m/s^2).
    % Algorithms may replace this with locally computed values when available.
    properties (Constant)
        % Gravity magnitude used across the MATLAB pipeline (m/s^2)
        % Matches the value in Python's constants for IMU/GNSS dataset X002
        GRAVITY = 9.79424753;
        EARTH_RATE = 7.2921e-5;
    end
end
