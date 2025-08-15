% Run all datasets using all methods
% Author: Auto-generated
% Date: 2025-08-15 09:49:01
% Description: Entrypoint to execute all datasets with all methods.

clear; clc;

% Resolve MATLAB directory and environment
this_file = mfilename('fullpath');
matlab_dir = fileparts(this_file);
orig_dir = pwd; cd(matlab_dir);
cleanupObj = onCleanup(@() cd(orig_dir));

addpath(genpath(matlab_dir));
format long;
set(0, 'DefaultFigureVisible', 'on');

% Delegate to methods orchestrator
run('run_all_methods.m');
