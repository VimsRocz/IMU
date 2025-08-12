function s = strings_compat(varargin)
%STRINGS_COMPAT Create string array compatible with both MATLAB and Octave
%   S = STRINGS_COMPAT(SIZE1, SIZE2, ...) creates an empty string array
%   with the specified dimensions. In MATLAB this uses strings(), in 
%   Octave this uses cell arrays which are then converted to strings when needed.
%
%   This function provides backward compatibility for the strings() function
%   which is not available in Octave.

    if exist('strings', 'builtin') == 5 || exist('strings', 'file') == 2
        % MATLAB - use native strings function
        s = strings(varargin{:});
    else
        % Octave - use cell array as fallback
        if nargin == 0
            s = {};
        elseif nargin == 1
            if varargin{1} == 0
                s = {};
            else
                s = cell(varargin{1}, 1);
                for i = 1:length(s)
                    s{i} = '';
                end
            end
        elseif nargin == 2
            if varargin{1} == 0 || varargin{2} == 0
                s = {};
            else
                s = cell(varargin{:});
                for i = 1:numel(s)
                    s{i} = '';
                end
            end
        else
            % General case for multiple dimensions
            if any([varargin{:}] == 0)
                s = {};
            else
                s = cell(varargin{:});
                for i = 1:numel(s)
                    s{i} = '';
                end
            end
        end
    end
end