function rd = get_results_dir_matlab()
%GET_RESULTS_DIR_MATLAB Always return the MATLAB-only results folder.
%   RD = GET_RESULTS_DIR_MATLAB() resolves the project paths and returns
%   the location used for MATLAB-generated artifacts.

p = project_paths();
rd = p.matlab_results;
end
