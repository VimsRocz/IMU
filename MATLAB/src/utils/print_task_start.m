function print_task_start(taskName)
%PRINT_TASK_START Prints a standard task start message
%   This function ensures no \u warnings in MATLAB
%   Usage: print_task_start('Task_1');
    fprintf('%s Running %s...\n', char(0x25B6), taskName);
end
