MATLAB — IMU/GNSS Processing

MATLAB code is in src/, outputs in results/.

Setup & Paths

```matlab
cd('MATLAB/src');
addpath(genpath(pwd));
```

Data

Use shared inputs under ../DATA/:
• ../DATA/IMU/IMU_X002.dat
• ../DATA/GNSS/GNSS_X002.csv
• ../DATA/Truth/STATE_X001.txt

Run Task-1 (TRIAD)

```matlab
Task_1    % or run_triad_only.m

Outputs: ../results/ (i.e., MATLAB/results/)
```

Troubleshooting

If a script can’t find data, build paths explicitly:

```matlab
imuPath   = fullfile('..','DATA','IMU','IMU_X002.dat');
gnssPath  = fullfile('..','DATA','GNSS','GNSS_X002.csv');
truthPath = fullfile('..','DATA','Truth','STATE_X001.txt');
```

