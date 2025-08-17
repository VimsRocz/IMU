function plot_state_grid(t, pos, vel, acc, frame, varargin)
%PLOT_STATE_GRID Deprecated per-quantity Task 4 plots (no-op).
%   This function previously generated three separate figures for NED/ECEF
%   position, velocity and acceleration with filenames like
%   "..._task4_NED_velocity.png". These plots are confusing in Task 4 and
%   have been removed. Use task4_plot_comparisons instead, which produces a
%   single 3x3 grid per frame.

% Preserve signature and options to avoid breaking callers, but do nothing.
p = inputParser; %#ok<NASGU>
addParameter(p,'visible','off');
addParameter(p,'save_dir','');
addParameter(p,'run_id','');
try, parse(p,varargin{:}); catch, end

fprintf('plot_state_grid: skipped (deprecated for Task 4; no files saved).\n');
end
