function [t_true, pos_ecef_true, vel_ecef_true] = load_state_truth(path)
%LOAD_STATE_TRUTH Load reference state file.
%   [t_true, pos_ecef_true, vel_ecef_true] = LOAD_STATE_TRUTH(PATH) reads the
%   reference state file located at PATH using READMATRIX while ignoring lines
%   starting with '#'. The function returns the timestamp vector and the ECEF
%   position and velocity components as double precision arrays.

data = readmatrix(path, 'CommentStyle', '#');
t_true = double(data(:,2));
pos_ecef_true = double(data(:,3:5));
vel_ecef_true = double(data(:,6:8));
end
