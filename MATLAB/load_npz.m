function S = load_npz(npzFile)
%LOAD_NPZ Load data from a NumPy .npz archive.
%   S = LOAD_NPZ(FILE) reads FILE using py.numpy.load and converts the
%   contained arrays to MATLAB types. Each array becomes a field in the
%   returned struct.
%
%   This helper requires Python and NumPy to be available via the MATLAB
%   "py" interface.
%
%   Example:
%       data = load_npz('results_X001.npz');

    np = py.importlib.import_module('numpy');
    loaded = np.load(npzFile);
    keys = cell(py.list(loaded.files));
    S = struct();
    for i = 1:numel(keys)
        key = string(keys{i});
        try
            S.(key) = double(loaded{key});
        catch
            S.(key) = loaded{key};
        end
    end
end
