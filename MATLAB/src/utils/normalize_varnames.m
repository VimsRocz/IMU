function names = normalize_varnames(namesIn)
%NORMALIZE_VARNAMES Convert input names to valid, lower-case MATLAB identifiers.
%   NAMES = NORMALIZE_VARNAMES(NAMESIN) accepts a cell array or string array
%   of names and returns a string array of lower-case, valid MATLAB
%   identifiers with invalid characters removed. This mirrors the Python
%   helper of the same name.

    if iscellstr(namesIn) || (iscell(namesIn) && all(cellfun(@ischar,namesIn)))
        names = string(namesIn);
    elseif isstring(namesIn)
        names = namesIn;
    else
        error('normalize_varnames:unsupportedType','Unsupported: %s',class(namesIn));
    end
    names = lower(names);
    names = matlab.lang.makeValidName(names,'ReplacementStyle','delete');
end
