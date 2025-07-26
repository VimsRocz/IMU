classdef constants
    % Mirrors constants defined in src/constants.py for cross-language parity
    % GRAVITY is a typical magnitude of gravitational acceleration (m/s^2).
    % Algorithms may replace this with locally computed values when available.
    properties (Constant)
        GRAVITY = 9.81;
        EARTH_RATE = 7.2921e-5;
    end
end
