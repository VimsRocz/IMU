function varargout = ecef_to_geodetic(varargin)
%ECEF_TO_GEODETIC Compatibility wrapper for ecef2geodetic.
%   [LAT_DEG, LON_DEG, ALT] = ECEF_TO_GEODETIC(X,Y,Z) forwards all arguments
%   to ECEF2GEODETIC so older scripts continue to function.
    [varargout{1:nargout}] = ecef2geodetic(varargin{:});
end
