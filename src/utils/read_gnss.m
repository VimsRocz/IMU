function gnss = read_gnss(filename)
%READ_GNSS  Load GNSS CSV measurements.
%   GNSS = READ_GNSS(FILENAME) returns a struct with ECEF coordinates and
%   velocities. Column names must match the sample logs distributed with
%   the repository.

    T = readtable(filename);
    gnss.time_s     = T.Posix_Time;
    gnss.X_ECEF_m   = T.X_ECEF_m;
    gnss.Y_ECEF_m   = T.Y_ECEF_m;
    gnss.Z_ECEF_m   = T.Z_ECEF_m;
    gnss.VX_ECEF_mps = T.VX_ECEF_mps;
    gnss.VY_ECEF_mps = T.VY_ECEF_mps;
    gnss.VZ_ECEF_mps = T.VZ_ECEF_mps;
end
