function dir = get_results_dir()
%GET_RESULTS_DIR Legacy wrapper for GET_MATLAB_RESULTS_DIR.
%   DIR = GET_RESULTS_DIR() forwards to GET_MATLAB_RESULTS_DIR to maintain
%   compatibility with older scripts. MATLAB results are stored under
%   <repository>/MATLAB/results.
%
%   See also GET_MATLAB_RESULTS_DIR.

    dir = get_matlab_results_dir();
end
