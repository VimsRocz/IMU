function data = read_gnss(path)
%READ_GNSS Load GNSS CSV measurements.
%   DATA = READ_GNSS(PATH) returns a table with named columns.

    data = readtable(path);
end
