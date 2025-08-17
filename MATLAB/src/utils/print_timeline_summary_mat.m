function out_txt = print_timeline_summary_mat(rid, imu_path, gnss_path, truth_path, out_dir)
%PRINT_TIMELINE_SUMMARY_MAT Wrapper around PRINT_TIMELINE_MATLAB.
%   OUT_TXT = PRINT_TIMELINE_SUMMARY_MAT(RID, IMU_PATH, GNSS_PATH, TRUTH_PATH,
%   OUT_DIR) simply forwards to ``print_timeline_matlab`` for generating the
%   timeline summary. This exists for backward compatibility.
%
%   See also: PRINT_TIMELINE_MATLAB

    if nargin < 5 || isempty(out_dir)
        paths = project_paths();
        out_dir = paths.matlab_results;
    end
    out_txt = print_timeline_matlab(rid, imu_path, gnss_path, truth_path, out_dir);
end
