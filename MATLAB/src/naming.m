function path = naming(varargin)
%NAMING Utility helpers for standardised filenames.
%   TAG = NAMING('build_tag', IMU_FILE, GNSS_FILE, METHOD) returns a dataset
%   tag such as 'IMU_X001_GNSS_X001_TRIAD'.
%   FILE = NAMING('plot_path', DIR, TAG, TASK, SUBTASK, TYPE, EXT) builds a
%   full path for plots or results.
%
%   Example:
%       tag = naming('build_tag', 'IMU_X001.dat', 'GNSS_X001.csv', 'TRIAD');
%       f = naming('plot_path', 'results', tag, 7, '3', 'residuals_position_velocity');

op = varargin{1};
if strcmp(op, 'build_tag')
    imu_file = varargin{2};
    gnss_file = varargin{3};
    method = varargin{4};
    [~, ibase, ~] = fileparts(imu_file);
    [~, gbase, ~] = fileparts(gnss_file);
    path = sprintf('%s_%s_%s', ibase, gbase, method);
elseif strcmp(op, 'plot_path')
    dir = varargin{2};
    tag = varargin{3};
    task = varargin{4};
    subtask = varargin{5};
    type = varargin{6};
    if nargin < 7
        ext = '.pdf';
    else
        ext = varargin{7};
    end
    fname = sprintf('%s_task%d_%s_%s%s', tag, task, subtask, type, ext);
    path = fullfile(dir, fname);
else
    error('Unknown operation');
end
end
