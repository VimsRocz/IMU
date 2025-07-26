function gnss = read_gnss(filename)
%READ_GNSS  Load GNSS CSV log into a table.
%   GNSS = READ_GNSS(FILENAME) returns a table containing the columns
%   ``X_ECEF_m``, ``Y_ECEF_m``, ``Z_ECEF_m`` and their velocities in m/s.

    opts = detectImportOptions(filename);
    gnss = readtable(filename, opts);
end
