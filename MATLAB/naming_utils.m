function out = naming_utils()
%NAMING_UTILS  Return function handles for standardised naming.
%   UTILS = NAMING_UTILS() returns a struct with helper functions to
%   construct dataset-method tags and plot filenames matching the Python
%   utilities. Example:
%       u = naming_utils();
%       tag = u.make_tag('IMU_X001.dat','GNSS_X001.csv','TRIAD');
%       fname = u.plot_filename('IMU_X001.dat','GNSS_X001.csv','TRIAD',5,'8','final_position');
%
%   The resulting fname is
%   'IMU_X001_GNSS_X001_TRIAD_task5_8_final_position.pdf'.

    out.make_tag = @make_tag;
    out.script_name = @script_name;
    out.output_dir = @output_dir;
    out.plot_filename = @plot_filename;
end

function tag = make_tag(dataset, gnss, method)
    [~, dname, ~] = fileparts(dataset);
    [~, gname, ~] = fileparts(gnss);
    tag = sprintf('%s_%s_%s', dname, gname, method);
end

function name = script_name(dataset, method, task, ext)
    if nargin < 4
        ext = 'm';
    end
    [~, dname, ~] = fileparts(dataset);
    name = sprintf('%s_%s_task%d.%s', dname, method, task, ext);
end

function dir = output_dir(task, dataset, gnss, method)
    tag = make_tag(dataset, gnss, method);
    dir = fullfile('results', sprintf('task%d', task), tag);
end

function fname = plot_filename(dataset, gnss, method, task, subtask, out_type, ext)
    if nargin < 7
        ext = 'pdf';
    end
    tag = make_tag(dataset, gnss, method);
    fname = sprintf('%s_task%d_%s_%s.%s', tag, task, subtask, out_type, ext);
end

