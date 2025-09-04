# Comprehensive IMU/GNSS Data Processing Instructions

## Table of Contents

1. [Introduction](#introduction)
2. [Repository Overview](#repository-overview)  
3. [Python Environment Setup](#python-environment-setup)
4. [MATLAB Environment Setup](#matlab-environment-setup)
5. [Data Formats and Structure](#data-formats-and-structure)
6. [Core Concepts and Algorithms](#core-concepts-and-algorithms)
7. [Task-by-Task Tutorial](#task-by-task-tutorial)
8. [Python Implementation Guide](#python-implementation-guide)
9. [MATLAB Implementation Guide](#matlab-implementation-guide)
10. [Advanced Usage Scenarios](#advanced-usage-scenarios)
11. [Troubleshooting and Debugging](#troubleshooting-and-debugging)
12. [API Reference](#api-reference)
13. [Performance Optimization](#performance-optimization)
14. [Case Studies and Examples](#case-studies-and-examples)
15. [Contributing and Development](#contributing-and-development)

---

## Introduction

The IMU/GNSS Data Processing Repository provides comprehensive tools for processing Inertial Measurement Unit (IMU) and Global Navigation Satellite System (GNSS) data using both Python and MATLAB environments. This repository implements advanced algorithms for sensor fusion, attitude determination, and navigation state estimation.

### What This Repository Does

This repository implements a complete pipeline for IMU/GNSS data processing that includes:

- **Attitude Initialization**: Using TRIAD, Davenport, and SVD methods to determine initial orientation
- **Sensor Bias Estimation**: Automatic detection and compensation of accelerometer and gyroscope biases
- **Data Fusion**: Extended Kalman Filter implementation for optimal state estimation
- **Trajectory Integration**: Dead-reckoning and integrated navigation solutions
- **Validation and Analysis**: Comprehensive error analysis and truth comparison tools
- **Visualization**: Rich plotting capabilities for data analysis and results presentation

### Key Features

- **Dual Implementation**: Complete parallel implementations in both Python and MATLAB
- **Multiple Algorithms**: Support for TRIAD, Davenport, and SVD attitude determination methods
- **Real Data Processing**: Works with actual IMU and GNSS sensor logs
- **Validation Framework**: Built-in truth data comparison and error analysis
- **Extensible Design**: Modular architecture for easy customization and extension
- **Comprehensive Documentation**: Detailed documentation for every component

### Target Audience

This guide is designed for:

- **Researchers** working on navigation and sensor fusion algorithms
- **Engineers** developing IMU/GNSS systems
- **Students** learning about inertial navigation and Kalman filtering
- **Practitioners** needing robust tools for sensor data analysis

---

## Repository Overview

### Architecture

The repository follows a clean, modular architecture with separate environments for Python and MATLAB while sharing common data sources:

```
IMU/
├── DATA/                    # Shared sensor data and truth files
│   ├── IMU/                # Raw IMU sensor logs (.dat files)
│   ├── GNSS/               # GNSS position/velocity logs (.csv files)  
│   └── Truth/              # Reference trajectory data (.txt files)
├── PYTHON/                 # Python implementation environment
│   ├── src/                # Core Python modules and algorithms
│   ├── results/            # Generated outputs and analysis results
│   ├── config/             # Configuration files (YAML/JSON)
│   ├── tests/              # Unit tests and validation scripts
│   └── docs/               # Python-specific documentation
└── MATLAB/                 # MATLAB implementation environment
    ├── src/                # MATLAB functions and utilities
    ├── results/            # Generated outputs and analysis results
    ├── config/             # MATLAB configuration files
    └── docs/               # MATLAB-specific documentation
```

### Design Philosophy

The repository is built around several key principles:

1. **Language Parity**: Both Python and MATLAB implementations produce equivalent results
2. **Modularity**: Each task is implemented as a separate, testable module
3. **Data Sharing**: Common data formats allow easy comparison between implementations
4. **Reproducibility**: All results can be reproduced with provided data and scripts
5. **Extensibility**: New algorithms and methods can be easily integrated

### Core Components

#### Task-Based Pipeline

The processing pipeline is organized into seven distinct tasks:

1. **Task 1**: Reference vector computation in the navigation frame
2. **Task 2**: Body frame vector measurement and bias estimation  
3. **Task 3**: Initial attitude determination using multiple algorithms
4. **Task 4**: IMU-only integration and GNSS comparison
5. **Task 5**: Extended Kalman Filter fusion of IMU and GNSS
6. **Task 6**: Truth data overlay and validation
7. **Task 7**: Comprehensive error analysis and performance evaluation

#### Algorithm Implementations

The repository includes implementations of:

- **TRIAD Algorithm**: Classical three-axis attitude determination
- **Davenport's q-Method**: Quaternion-based optimal attitude estimation
- **SVD Method**: Singular Value Decomposition approach to attitude determination
- **Extended Kalman Filter**: Full state estimation with IMU propagation and GNSS updates
- **Bias Estimation**: Automatic detection of sensor biases during static periods

---

## Python Environment Setup

### System Requirements

#### Operating System Support
- **Linux**: Ubuntu 20.04 LTS or later (recommended)
- **macOS**: macOS 10.15 or later
- **Windows**: Windows 10 or later with WSL2 (recommended) or native Python

#### Python Version Requirements
- **Python 3.8 or later** (Python 3.9-3.11 recommended)
- **pip** package manager (usually included with Python)
- **virtualenv** or **conda** for environment management (highly recommended)

### Step-by-Step Installation

#### 1. Clone the Repository

First, clone the repository to your local machine:

```bash
# Using HTTPS
git clone https://github.com/VimsRocz/IMU.git
cd IMU

# Or using SSH (if configured)
git clone git@github.com:VimsRocz/IMU.git
cd IMU
```

#### 2. Create a Virtual Environment

Creating a virtual environment is strongly recommended to avoid conflicts with other Python packages:

```bash
# Using venv (built into Python 3.3+)
python -m venv imu_env

# Activate the environment
# On Linux/macOS:
source imu_env/bin/activate

# On Windows:
imu_env\Scripts\activate

# Using conda (alternative)
conda create -n imu_env python=3.9
conda activate imu_env
```

#### 3. Install Dependencies

The repository includes a comprehensive requirements file with all necessary dependencies:

```bash
# Install all required packages
pip install -r requirements.txt

# If you encounter issues with filterpy, try:
pip install --upgrade pip setuptools
pip install filterpy --no-binary :all:
```

#### 4. Install Development Dependencies (Optional)

For development and testing, install additional dependencies:

```bash
# Install the package in editable mode with test dependencies
pip install -e .[tests]

# Or install test requirements separately
pip install -r requirements-dev.txt
```

### Dependency Overview

#### Core Dependencies

```python
numpy>=1.26.0      # Numerical computing foundation
scipy              # Scientific computing algorithms
pandas             # Data manipulation and analysis
matplotlib         # Plotting and visualization
pyyaml             # Configuration file parsing
tqdm               # Progress bars for long operations
tabulate           # Table formatting for reports
rich               # Enhanced console output
```

#### Navigation-Specific Dependencies

```python
filterpy           # Kalman filter implementations
cartopy            # Geospatial data processing
geomaglib>=1.2.1   # Geomagnetic field modeling
pyproj             # Cartographic projections and coordinate transforms
geopy              # Geocoding and distance calculations
```

#### Visualization Dependencies

```python
plotly>=5.0        # Interactive plotting
kaleido            # Static image export for Plotly
fastdtw            # Dynamic Time Warping for data alignment
```

#### Detailed Dependency Installation

##### FilterPy Installation Issues

FilterPy is crucial for Kalman filter functionality but can be challenging to install on some systems:

```bash
# If standard installation fails:
pip install filterpy

# Try building from source:
pip install filterpy --no-binary :all:

# On Ubuntu, you may need build tools:
sudo apt update
sudo apt install python3-pip build-essential python3-dev
pip install filterpy --no-binary :all:

# Force upgrade if conflicts occur:
pip install --upgrade --force-reinstall filterpy
```

##### Cartopy Installation Issues

Cartopy requires additional system libraries for geospatial functionality:

```bash
# On Ubuntu/Debian:
sudo apt install libproj-dev proj-data proj-bin libgeos-dev
pip install cartopy

# On CentOS/RHEL:
sudo yum install proj-devel geos-devel
pip install cartopy

# On macOS with Homebrew:
brew install proj geos
pip install cartopy

# Alternative: Use conda for easier installation
conda install -c conda-forge cartopy
```

### Configuration and Verification

#### 1. Verify Installation

Test that all major components work correctly:

```bash
# Test basic imports
python -c "import numpy, scipy, pandas, matplotlib; print('Core packages: OK')"
python -c "import filterpy, cartopy; print('Navigation packages: OK')"
python -c "import plotly, yaml; print('Visualization packages: OK')"

# Run the built-in environment test
python test_env.py
```

#### 2. Configure Python Path

The repository handles Python path configuration automatically, but you can verify:

```python
# From the repository root in Python:
import sys
print("Python path includes:")
for path in sys.path:
    if 'IMU' in path:
        print(f"  {path}")
```

#### 3. Test Data Access

Verify that the Python environment can access the shared data:

```python
from PYTHON.src.paths import data_dir, imu_path, gnss_path, truth_path

print(f"Data directory: {data_dir()}")
print(f"Sample IMU file: {imu_path('X001')}")
print(f"Sample GNSS file: {gnss_path('X001')}")
print(f"Truth file: {truth_path()}")

# Check that files exist
import os
print(f"Data directory exists: {os.path.exists(data_dir())}")
print(f"Sample files exist: {os.path.exists(imu_path('X001'))}")
```

### Advanced Python Setup

#### Development Environment

For contributors and advanced users, set up a complete development environment:

```bash
# Install development tools
pip install black flake8 pytest mypy pre-commit

# Install pre-commit hooks
pre-commit install

# Install Jupyter for interactive development
pip install jupyter notebook ipykernel

# Register the environment as a Jupyter kernel
python -m ipykernel install --user --name imu_env --display-name "IMU Processing"
```

#### IDE Configuration

##### VS Code Setup

Create `.vscode/settings.json`:

```json
{
    "python.defaultInterpreterPath": "./imu_env/bin/python",
    "python.linting.enabled": true,
    "python.linting.flake8Enabled": true,
    "python.formatting.provider": "black",
    "python.testing.pytestEnabled": true,
    "python.testing.pytestArgs": [
        "PYTHON/tests"
    ]
}
```

##### PyCharm Setup

1. Open the repository folder in PyCharm
2. Go to Settings → Project → Python Interpreter
3. Select the virtual environment: `./imu_env/bin/python`
4. Configure test runner: Settings → Tools → Python Integrated Tools → Testing → pytest

#### Performance Optimization

For processing large datasets, consider these optimizations:

```bash
# Install optional performance packages
pip install numba           # Just-in-time compilation
pip install scikit-learn   # Efficient algorithms
pip install joblib         # Parallel processing

# For large-scale processing
pip install dask           # Parallel computing
pip install h5py           # Efficient data storage
```

#### Memory Management

Configure Python for large dataset processing:

```python
# In your scripts, consider memory-efficient approaches:
import numpy as np
import pandas as pd

# Use memory mapping for large files
data = np.memmap('large_file.dat', dtype=np.float64, mode='r')

# Use chunked processing for DataFrames
for chunk in pd.read_csv('large_file.csv', chunksize=10000):
    process_chunk(chunk)

# Monitor memory usage
import psutil
import os
process = psutil.Process(os.getpid())
print(f"Memory usage: {process.memory_info().rss / 1024 / 1024:.1f} MB")
```

---

## MATLAB Environment Setup

### System Requirements

#### MATLAB Version Requirements

- **MATLAB R2018b or later** (R2023a recommended for best compatibility)
- **Required Toolboxes**:
  - Signal Processing Toolbox (essential for filtering operations)
  - Statistics and Machine Learning Toolbox (for bias estimation)
  - Mapping Toolbox (optional, for enhanced geographic visualizations)
  - Navigation Toolbox (optional, for additional reference implementations)

#### Operating System Support

- **Windows**: Windows 10 or later
- **macOS**: macOS 10.14 or later  
- **Linux**: Most distributions with glibc 2.17 or later

### Step-by-Step MATLAB Setup

#### 1. Verify MATLAB Installation

First, ensure MATLAB is properly installed and the required toolboxes are available:

```matlab
% Check MATLAB version
version

% Verify Signal Processing Toolbox (critical for filtering)
if license('test', 'Signal_Toolbox')
    fprintf('Signal Processing Toolbox: Available\n');
    ver('signal')
else
    warning('Signal Processing Toolbox not found - some features may use fallbacks');
end

% Check other useful toolboxes
toolboxes = {'Statistics_Toolbox', 'Map_Toolbox', 'Navigation_Toolbox'};
toolbox_names = {'Statistics and Machine Learning', 'Mapping', 'Navigation'};

for i = 1:length(toolboxes)
    if license('test', toolboxes{i})
        fprintf('%s Toolbox: Available\n', toolbox_names{i});
    else
        fprintf('%s Toolbox: Not available (optional)\n', toolbox_names{i});
    end
end
```

#### 2. Configure MATLAB Path

Set up the MATLAB path to include all necessary directories:

```matlab
% Navigate to the repository root directory
cd /path/to/IMU  % Replace with your actual path

% Add MATLAB directories to path
addpath('MATLAB');
addpath('MATLAB/src');
addpath('MATLAB/src/utils');
addpath(genpath('MATLAB/src'));  % Recursively add all subdirectories

% Save path for future sessions
savepath;

% Verify path configuration
fprintf('MATLAB search path configured successfully\n');
which('get_results_dir')  % Should find MATLAB/src/utils/get_results_dir.m
```

#### 3. Create Results Directory

Set up the output directory structure:

```matlab
% Create results directory if it doesn't exist
results_dir = fullfile(pwd, 'MATLAB', 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
    fprintf('Created MATLAB results directory: %s\n', results_dir);
else
    fprintf('MATLAB results directory exists: %s\n', results_dir);
end

% Test results directory access
test_file = fullfile(results_dir, 'test.mat');
save(test_file, 'results_dir');
delete(test_file);
fprintf('Results directory is writable\n');
```

#### 4. Verify Data Access

Ensure MATLAB can access the shared data directory:

```matlab
% Check data directory structure
data_dir = fullfile(pwd, 'DATA');
if exist(data_dir, 'dir')
    fprintf('Data directory found: %s\n', data_dir);
    
    % List available datasets
    imu_files = dir(fullfile(data_dir, 'IMU', '*.dat'));
    gnss_files = dir(fullfile(data_dir, 'GNSS', '*.csv'));
    truth_files = dir(fullfile(data_dir, 'Truth', '*.txt'));
    
    fprintf('Available datasets:\n');
    fprintf('  IMU files: %d\n', length(imu_files));
    fprintf('  GNSS files: %d\n', length(gnss_files));
    fprintf('  Truth files: %d\n', length(truth_files));
    
    % Display file details
    if ~isempty(imu_files)
        fprintf('IMU files:\n');
        for i = 1:length(imu_files)
            fprintf('  %s (%.1f MB)\n', imu_files(i).name, imu_files(i).bytes/1e6);
        end
    end
else
    error('Data directory not found. Please ensure you are in the repository root.');
end
```

### MATLAB Environment Configuration

#### 1. Memory and Performance Settings

Configure MATLAB for optimal performance with large datasets:

```matlab
% Check available memory
[user, sys] = memory;
fprintf('Available memory: %.1f GB\n', user.MemAvailableAllArrays/1e9);

% Set Java heap space for large data processing
if user.MemAvailableAllArrays > 8e9  % If more than 8GB available
    java.lang.System.setProperty('java.awt.headless', 'true');
    fprintf('Java heap configured for large datasets\n');
end

% Configure parallel processing if Parallel Computing Toolbox is available
if license('test', 'Distrib_Computing_Toolbox')
    try
        parpool('local');  % Start parallel pool
        fprintf('Parallel pool started\n');
    catch ME
        fprintf('Parallel pool not started: %s\n', ME.message);
    end
end
```

#### 2. Graphics and Display Configuration

Set up graphics for batch processing and visualization:

```matlab
% Configure graphics for both interactive and batch modes
set(groot, 'DefaultFigureVisible', 'on');  % 'off' for batch processing

% Set default figure properties for consistent output
set(groot, 'DefaultFigurePosition', [100, 100, 800, 600]);
set(groot, 'DefaultAxesFontSize', 12);
set(groot, 'DefaultTextFontSize', 12);
set(groot, 'DefaultLineLineWidth', 1.5);

% Configure paper properties for PDF export
set(groot, 'DefaultFigurePaperType', 'A4');
set(groot, 'DefaultFigurePaperOrientation', 'landscape');
set(groot, 'DefaultFigurePaperPositionMode', 'auto');

fprintf('Graphics configuration complete\n');
```

#### 3. Numerical Precision Settings

Configure numerical settings for consistent results:

```matlab
% Set consistent random number generation
rng('default');  % Use default seed for reproducibility
% rng(12345);    % Or use specific seed

% Display format for better readability
format long g;

% Verify floating-point precision
fprintf('MATLAB floating-point precision: %s\n', class(1.0));
fprintf('Machine epsilon: %e\n', eps);
```

### Testing the MATLAB Installation

#### 1. Basic Functionality Test

Run a simple test to verify the installation:

```matlab
% Test basic mathematical operations
test_matrix = randn(100, 100);
tic;
[U, S, V] = svd(test_matrix);
elapsed_time = toc;
fprintf('SVD of 100x100 matrix completed in %.3f seconds\n', elapsed_time);

% Test signal processing functionality
if license('test', 'Signal_Toolbox')
    test_signal = sin(2*pi*(1:1000)/100) + 0.1*randn(1, 1000);
    filtered_signal = lowpass(test_signal, 0.3);
    fprintf('Signal processing test completed\n');
else
    fprintf('Signal processing test skipped (toolbox not available)\n');
end
```

#### 2. Data Loading Test

Test the ability to load and process IMU data:

```matlab
% Test loading IMU data
try
    imu_file = fullfile('DATA', 'IMU', 'IMU_X001.dat');
    if exist(imu_file, 'file')
        % Test reading the file
        data = readmatrix(imu_file);
        [num_samples, num_cols] = size(data);
        fprintf('Successfully loaded IMU data: %d samples, %d columns\n', ...
                num_samples, num_cols);
        
        % Basic data validation
        if num_cols >= 8
            fprintf('IMU data format appears correct\n');
            
            % Display sample statistics
            fprintf('Time range: %.3f to %.3f seconds\n', ...
                    data(1,2), data(end,2));
            fprintf('Sample rate: approximately %.1f Hz\n', ...
                    1/mean(diff(data(1:100,2))));
        else
            warning('Unexpected IMU data format (expected ≥8 columns)');
        end
    else
        warning('Sample IMU file not found: %s', imu_file);
    end
catch ME
    warning('Failed to load IMU data: %s', ME.message);
end
```

#### 3. Results Generation Test

Verify that the system can generate and save results:

```matlab
% Test results generation and saving
try
    % Create test data
    test_results.timestamp = datestr(now);
    test_results.test_data = randn(10, 3);
    test_results.parameters = struct('param1', 1.0, 'param2', 'test');
    
    % Save to results directory
    results_file = fullfile('MATLAB', 'results', 'installation_test.mat');
    save(results_file, 'test_results');
    
    % Create test figure
    figure('Visible', 'off');
    plot(test_results.test_data);
    title('Installation Test Plot');
    xlabel('Sample'); ylabel('Value');
    
    % Save figure
    fig_file = fullfile('MATLAB', 'results', 'installation_test.pdf');
    saveas(gcf, fig_file);
    close(gcf);
    
    fprintf('Results generation test completed successfully\n');
    fprintf('Test files created in: MATLAB/results/\n');
    
    % Clean up test files
    delete(results_file);
    delete(fig_file);
    
catch ME
    error('Results generation test failed: %s', ME.message);
end
```

### Advanced MATLAB Configuration

#### 1. Custom Startup Script

Create a startup script that automatically configures the environment:

Create `MATLAB/startup.m`:

```matlab
function startup()
    % IMU Repository MATLAB Startup Configuration
    % This function is called automatically when MATLAB starts
    % if this file is in the MATLAB search path
    
    fprintf('Configuring IMU processing environment...\n');
    
    % Set up paths
    repo_root = fileparts(mfilename('fullpath'));
    repo_root = fileparts(repo_root);  % Go up one level from MATLAB/
    
    addpath(fullfile(repo_root, 'MATLAB'));
    addpath(fullfile(repo_root, 'MATLAB', 'src'));
    addpath(genpath(fullfile(repo_root, 'MATLAB', 'src')));
    
    % Configure environment
    set(groot, 'DefaultFigureVisible', 'off');  % For batch processing
    format long g;
    
    % Verify critical toolboxes
    if ~license('test', 'Signal_Toolbox')
        warning('Signal Processing Toolbox not detected - some features may be limited');
    end
    
    fprintf('IMU environment configured successfully\n');
end
```

#### 2. Batch Processing Configuration

For processing multiple datasets automatically:

```matlab
% Configure MATLAB for unattended batch processing
function configure_batch_mode()
    % Disable interactive features
    set(groot, 'DefaultFigureVisible', 'off');
    
    % Configure error handling
    dbstop if error  % Uncomment for debugging
    
    % Set up logging
    diary(fullfile('MATLAB', 'results', 'batch_log.txt'));
    diary on;
    
    fprintf('Batch processing mode configured\n');
    fprintf('Log file: MATLAB/results/batch_log.txt\n');
end
```

#### 3. Memory Management for Large Datasets

```matlab
function configure_memory_management()
    % Get system memory information
    [user, sys] = memory;
    available_gb = user.MemAvailableAllArrays / 1e9;
    
    fprintf('Available memory: %.1f GB\n', available_gb);
    
    if available_gb < 4
        warning('Limited memory detected. Consider processing smaller datasets or use chunking.');
        
        % Configure for memory-efficient processing
        clear('all');  % Clear workspace
        pack;          % Defragment memory
        
    elseif available_gb > 16
        % Configure for high-performance processing
        maxNumCompThreads('automatic');
        fprintf('High-performance configuration enabled\n');
    end
end
```

### Troubleshooting MATLAB Setup

#### Common Issues and Solutions

##### Issue 1: Path Configuration Problems

```matlab
% Diagnose path issues
function diagnose_path_issues()
    fprintf('Current directory: %s\n', pwd);
    fprintf('MATLAB path entries containing "IMU":\n');
    
    paths = strsplit(path, pathsep);
    imu_paths = paths(contains(paths, 'IMU', 'IgnoreCase', true));
    
    if isempty(imu_paths)
        warning('No IMU-related paths found');
        fprintf('Run: addpath(''MATLAB''); addpath(''MATLAB/src'');\n');
    else
        cellfun(@(p) fprintf('  %s\n', p), imu_paths);
    end
    
    % Test key function availability
    key_functions = {'get_results_dir', 'read_imu_numeric', 'ecef2ned'};
    for i = 1:length(key_functions)
        func_path = which(key_functions{i});
        if isempty(func_path)
            fprintf('Missing: %s\n', key_functions{i});
        else
            fprintf('Found: %s -> %s\n', key_functions{i}, func_path);
        end
    end
end
```

##### Issue 2: Toolbox Availability Problems

```matlab
% Check and report toolbox issues
function check_toolbox_requirements()
    required_toolboxes = {
        'Signal_Toolbox', 'Signal Processing Toolbox', true;
        'Statistics_Toolbox', 'Statistics and Machine Learning Toolbox', false;
        'Map_Toolbox', 'Mapping Toolbox', false;
        'Navigation_Toolbox', 'Navigation Toolbox', false
    };
    
    fprintf('Toolbox Availability Report:\n');
    fprintf('----------------------------\n');
    
    for i = 1:size(required_toolboxes, 1)
        license_name = required_toolboxes{i, 1};
        display_name = required_toolboxes{i, 2};
        is_required = required_toolboxes{i, 3};
        
        if license('test', license_name)
            fprintf('✓ %s: Available\n', display_name);
        else
            if is_required
                fprintf('✗ %s: MISSING (Required)\n', display_name);
            else
                fprintf('- %s: Not available (Optional)\n', display_name);
            end
        end
    end
    
    % Provide installation guidance
    if ~license('test', 'Signal_Toolbox')
        fprintf('\nTo install Signal Processing Toolbox:\n');
        fprintf('1. Open MATLAB Add-On Manager (Home tab → Add-Ons → Get Add-Ons)\n');
        fprintf('2. Search for "Signal Processing Toolbox"\n');
        fprintf('3. Click Install\n');
    end
end
```

##### Issue 3: Data Access Problems

```matlab
% Diagnose data access issues
function diagnose_data_access()
    fprintf('Data Access Diagnostic:\n');
    fprintf('----------------------\n');
    
    % Check if we're in the right directory
    current_dir = pwd;
    fprintf('Current directory: %s\n', current_dir);
    
    if ~contains(current_dir, 'IMU')
        warning('Current directory may not be the IMU repository root');
        fprintf('Navigate to the IMU repository directory first\n');
    end
    
    % Check data directory structure
    data_dir = fullfile(current_dir, 'DATA');
    subdirs = {'IMU', 'GNSS', 'Truth'};
    
    for i = 1:length(subdirs)
        subdir_path = fullfile(data_dir, subdirs{i});
        if exist(subdir_path, 'dir')
            files = dir(fullfile(subdir_path, '*.*'));
            files = files(~[files.isdir]);  % Remove directories
            fprintf('✓ %s: %d files\n', subdirs{i}, length(files));
        else
            fprintf('✗ %s: Directory missing\n', subdirs{i});
        end
    end
    
    % Test loading a sample file
    sample_file = fullfile(data_dir, 'IMU', 'IMU_X001.dat');
    if exist(sample_file, 'file')
        try
            data = readmatrix(sample_file);
            fprintf('✓ Sample data loading: Success (%d×%d)\n', size(data));
        catch ME
            fprintf('✗ Sample data loading: Failed (%s)\n', ME.message);
        end
    else
        fprintf('✗ Sample file not found: %s\n', sample_file);
    end
end
```

This completes the comprehensive MATLAB environment setup section. The setup process ensures that users have a properly configured MATLAB environment capable of running all the IMU/GNSS processing algorithms with appropriate error handling and diagnostic capabilities.

---

## Data Formats and Structure

### Overview

The IMU repository uses standardized data formats to ensure compatibility between Python and MATLAB implementations. Understanding these formats is crucial for processing your own data or extending the algorithms.

### IMU Data Format

#### File Structure

IMU data files are stored in binary format with the `.dat` extension. Each file contains timestamped inertial measurements arranged in a specific column structure.

#### Column Layout

IMU files contain 8 columns of data:

| Column | Name | Units | Description |
|--------|------|-------|-------------|
| 1 | Sample Number | - | Sequential sample counter starting from 0 |
| 2 | Time | seconds | POSIX timestamp (seconds since Unix epoch) |
| 3 | Gyro X | rad/s × dt | Angular velocity increment about X-axis |
| 4 | Gyro Y | rad/s × dt | Angular velocity increment about Y-axis |  
| 5 | Gyro Z | rad/s × dt | Angular velocity increment about Z-axis |
| 6 | Accel X | m/s² × dt | Acceleration increment along X-axis |
| 7 | Accel Y | m/s² × dt | Acceleration increment along Y-axis |
| 8 | Accel Z | m/s² × dt | Acceleration increment along Z-axis |

**Important Notes:**
- Gyroscope and accelerometer values are stored as **increments** (rate × Δt), not instantaneous rates
- To convert to rates, divide by the sampling interval: `rate = increment / dt`
- The coordinate system follows the IMU manufacturer's convention (typically right-handed)

#### Sample Loading Code

**Python:**
```python
import numpy as np
from PYTHON.src.paths import imu_path

# Load IMU data
imu_file = imu_path('X001')  # Resolves to DATA/IMU/IMU_X001.dat
imu_data = np.loadtxt(imu_file)

# Extract components
sample_num = imu_data[:, 0]
timestamps = imu_data[:, 1]
gyro_increments = imu_data[:, 2:5]  # [x, y, z]
accel_increments = imu_data[:, 5:8]  # [x, y, z]

# Convert increments to rates
dt = np.mean(np.diff(timestamps))  # Average sampling interval
gyro_rates = gyro_increments / dt  # rad/s
accel_rates = accel_increments / dt  # m/s²

print(f"Loaded {len(timestamps)} IMU samples")
print(f"Time range: {timestamps[0]:.3f} to {timestamps[-1]:.3f} seconds")
print(f"Sampling rate: {1/dt:.1f} Hz")
print(f"Duration: {timestamps[-1] - timestamps[0]:.1f} seconds")
```

**MATLAB:**
```matlab
% Load IMU data
imu_file = fullfile('DATA', 'IMU', 'IMU_X001.dat');
imu_data = readmatrix(imu_file);

% Extract components
sample_num = imu_data(:, 1);
timestamps = imu_data(:, 2);
gyro_increments = imu_data(:, 3:5);  % [x, y, z]
accel_increments = imu_data(:, 6:8);  % [x, y, z]

% Convert increments to rates
dt = mean(diff(timestamps));  % Average sampling interval
gyro_rates = gyro_increments / dt;  % rad/s
accel_rates = accel_increments / dt;  % m/s²

fprintf('Loaded %d IMU samples\n', length(timestamps));
fprintf('Time range: %.3f to %.3f seconds\n', timestamps(1), timestamps(end));
fprintf('Sampling rate: %.1f Hz\n', 1/dt);
fprintf('Duration: %.1f seconds\n', timestamps(end) - timestamps(1));
```

#### Data Quality Characteristics

**Typical Specifications:**
- **Sampling Rate**: 400 Hz (2.5 ms intervals)
- **Gyroscope Range**: ±2000°/s (±34.9 rad/s)
- **Accelerometer Range**: ±16g (±156.9 m/s²)
- **Noise Characteristics**: 
  - Gyroscope: ~0.1°/s/√Hz angle random walk
  - Accelerometer: ~0.01 m/s²/√Hz velocity random walk

**Data Quality Indicators:**
```python
def analyze_imu_data_quality(imu_data):
    """Analyze IMU data quality and print statistics."""
    timestamps = imu_data[:, 1]
    gyro_increments = imu_data[:, 2:5]
    accel_increments = imu_data[:, 5:8]
    
    # Timing analysis
    dt_values = np.diff(timestamps)
    dt_mean = np.mean(dt_values)
    dt_std = np.std(dt_values)
    
    print("Timing Quality:")
    print(f"  Mean Δt: {dt_mean*1000:.3f} ms")
    print(f"  Δt std: {dt_std*1000:.3f} ms")
    print(f"  Timing jitter: {dt_std/dt_mean*100:.2f}%")
    
    # Convert to physical units
    gyro_rates = gyro_increments / dt_mean
    accel_rates = accel_increments / dt_mean
    
    # Noise analysis on a static segment (first 1000 samples)
    static_gyro = gyro_rates[:1000]
    static_accel = accel_rates[:1000]
    
    print("\nNoise Analysis (first 1000 samples):")
    print("  Gyroscope std [x,y,z]:", np.std(static_gyro, axis=0))
    print("  Accelerometer std [x,y,z]:", np.std(static_accel, axis=0))
    
    # Dynamic range analysis
    print(f"\nDynamic Range:")
    print(f"  Gyro range: [{np.min(gyro_rates):.2f}, {np.max(gyro_rates):.2f}] rad/s")
    print(f"  Accel range: [{np.min(accel_rates):.2f}, {np.max(accel_rates):.2f}] m/s²")
```

### GNSS Data Format

#### File Structure

GNSS data is stored in CSV format with comma-separated values. Each file contains position and velocity solutions in the Earth-Centered Earth-Fixed (ECEF) coordinate frame.

#### Column Layout

GNSS files contain the following columns:

| Column | Name | Units | Description |
|--------|------|-------|-------------|
| 1 | Week | GPS weeks | GPS week number |
| 2 | TOW | seconds | GPS Time of Week |
| 3 | Posix_Time | seconds | POSIX timestamp (Unix time) |
| 4 | Latitude | degrees | Geodetic latitude (often zero - derived from ECEF) |
| 5 | Longitude | degrees | Geodetic longitude (often zero - derived from ECEF) |
| 6 | Height | meters | Height above ellipsoid (often zero - derived from ECEF) |
| 7 | X_ECEF | meters | ECEF X coordinate |
| 8 | Y_ECEF | meters | ECEF Y coordinate |
| 9 | Z_ECEF | meters | ECEF Z coordinate |
| 10 | VX_ECEF | m/s | ECEF X velocity |
| 11 | VY_ECEF | m/s | ECEF Y velocity |
| 12 | VZ_ECEF | m/s | ECEF Z velocity |

**Important Notes:**
- Primary data is in ECEF coordinates (columns 7-12)
- Latitude, longitude, and height (columns 4-6) are often zeros and should be computed from ECEF
- Time synchronization is crucial - POSIX_Time (column 3) is used for IMU/GNSS alignment

#### Sample Loading Code

**Python:**
```python
import pandas as pd
from PYTHON.src.paths import gnss_path

# Load GNSS data
gnss_file = gnss_path('X001')  # Resolves to DATA/GNSS/GNSS_X001.csv
gnss_data = pd.read_csv(gnss_file)

# Extract key components
posix_times = gnss_data['Posix_Time'].values
positions_ecef = gnss_data[['X_ECEF', 'Y_ECEF', 'Z_ECEF']].values
velocities_ecef = gnss_data[['VX_ECEF', 'VY_ECEF', 'VZ_ECEF']].values

print(f"Loaded {len(posix_times)} GNSS epochs")
print(f"Time range: {posix_times[0]:.3f} to {posix_times[-1]:.3f}")
print(f"Position range ECEF:")
print(f"  X: [{np.min(positions_ecef[:,0]):.1f}, {np.max(positions_ecef[:,0]):.1f}] m")
print(f"  Y: [{np.min(positions_ecef[:,1]):.1f}, {np.max(positions_ecef[:,1]):.1f}] m") 
print(f"  Z: [{np.min(positions_ecef[:,2]):.1f}, {np.max(positions_ecef[:,2]):.1f}] m")

# Convert first position to geodetic for reference
from PYTHON.src.utils import ecef_to_geodetic
lat0, lon0, alt0 = ecef_to_geodetic(positions_ecef[0])
print(f"Initial position (geodetic): {lat0:.6f}°, {lon0:.6f}°, {alt0:.1f}m")
```

**MATLAB:**
```matlab
% Load GNSS data
gnss_file = fullfile('DATA', 'GNSS', 'GNSS_X001.csv');
gnss_data = readtable(gnss_file);

% Extract key components
posix_times = gnss_data.Posix_Time;
positions_ecef = [gnss_data.X_ECEF, gnss_data.Y_ECEF, gnss_data.Z_ECEF];
velocities_ecef = [gnss_data.VX_ECEF, gnss_data.VY_ECEF, gnss_data.VZ_ECEF];

fprintf('Loaded %d GNSS epochs\n', length(posix_times));
fprintf('Time range: %.3f to %.3f\n', posix_times(1), posix_times(end));
fprintf('Position range ECEF:\n');
fprintf('  X: [%.1f, %.1f] m\n', min(positions_ecef(:,1)), max(positions_ecef(:,1)));
fprintf('  Y: [%.1f, %.1f] m\n', min(positions_ecef(:,2)), max(positions_ecef(:,2)));
fprintf('  Z: [%.1f, %.1f] m\n', min(positions_ecef(:,3)), max(positions_ecef(:,3)));

% Convert first position to geodetic for reference
[lat0, lon0, alt0] = ecef2geodetic(positions_ecef(1,:));
fprintf('Initial position (geodetic): %.6f°, %.6f°, %.1fm\n', lat0, lon0, alt0);
```

#### GNSS Data Quality Assessment

```python
def analyze_gnss_data_quality(gnss_data):
    """Analyze GNSS data quality and continuity."""
    times = gnss_data['Posix_Time'].values
    pos_ecef = gnss_data[['X_ECEF', 'Y_ECEF', 'Z_ECEF']].values
    vel_ecef = gnss_data[['VX_ECEF', 'VY_ECEF', 'VZ_ECEF']].values
    
    # Timing analysis
    dt_gnss = np.diff(times)
    print("GNSS Timing Quality:")
    print(f"  Mean interval: {np.mean(dt_gnss):.3f} s")
    print(f"  Std interval: {np.std(dt_gnss):.3f} s") 
    print(f"  Min/Max interval: {np.min(dt_gnss):.3f}/{np.max(dt_gnss):.3f} s")
    
    # Position continuity check
    pos_diff = np.diff(pos_ecef, axis=0)
    pos_jump_magnitude = np.linalg.norm(pos_diff, axis=1)
    
    print(f"\nPosition Continuity:")
    print(f"  Mean position change: {np.mean(pos_jump_magnitude):.3f} m")
    print(f"  Max position jump: {np.max(pos_jump_magnitude):.3f} m")
    
    # Velocity consistency check
    vel_magnitude = np.linalg.norm(vel_ecef, axis=1)
    print(f"\nVelocity Statistics:")
    print(f"  Mean speed: {np.mean(vel_magnitude):.3f} m/s")
    print(f"  Max speed: {np.max(vel_magnitude):.3f} m/s")
    
    # Check for potential outliers
    large_jumps = pos_jump_magnitude > 10.0  # More than 10m between epochs
    if np.any(large_jumps):
        print(f"  Warning: {np.sum(large_jumps)} large position jumps detected")
        jump_indices = np.where(large_jumps)[0]
        for idx in jump_indices[:5]:  # Show first 5
            print(f"    Jump at epoch {idx}: {pos_jump_magnitude[idx]:.1f} m")
```

### Truth Data Format

#### File Structure

Truth data provides reference trajectory information for validation purposes. Files are stored in ASCII text format with space-separated values.

#### Column Layout

Truth files contain the following columns:

| Column | Name | Units | Description |
|--------|------|-------|-------------|
| 1 | Sample Number | - | Sequential sample counter |
| 2 | Time | seconds | POSIX timestamp |
| 3 | X_ECEF | meters | True ECEF X position |
| 4 | Y_ECEF | meters | True ECEF Y position |
| 5 | Z_ECEF | meters | True ECEF Z position |
| 6 | VX_ECEF | m/s | True ECEF X velocity |
| 7 | VY_ECEF | m/s | True ECEF Y velocity |
| 8 | VZ_ECEF | m/s | True ECEF Z velocity |
| 9 | Roll | radians | True roll angle |
| 10 | Pitch | radians | True pitch angle |
| 11 | Yaw | radians | True yaw angle |

#### Sample Loading Code

**Python:**
```python
from PYTHON.src.paths import truth_path
from PYTHON.src.read_state_file import read_state_file

# Load truth data using built-in function
truth_file = truth_path('STATE_X001.txt')
truth_data = read_state_file(truth_file)

print(f"Loaded truth data with {len(truth_data)} epochs")
print("Available fields:", list(truth_data.keys()))

# Extract components
times = truth_data['time']
pos_ecef = np.column_stack([truth_data['x'], truth_data['y'], truth_data['z']])
vel_ecef = np.column_stack([truth_data['vx'], truth_data['vy'], truth_data['vz']])
attitudes = np.column_stack([truth_data['roll'], truth_data['pitch'], truth_data['yaw']])

print(f"Time range: {times[0]:.3f} to {times[-1]:.3f} seconds")
print(f"Duration: {times[-1] - times[0]:.1f} seconds")
```

**MATLAB:**
```matlab
% Load truth data
truth_file = fullfile('DATA', 'Truth', 'STATE_X001.txt');
truth_data = readmatrix(truth_file);

% Extract components (assuming standard format)
sample_nums = truth_data(:, 1);
times = truth_data(:, 2);
pos_ecef = truth_data(:, 3:5);
vel_ecef = truth_data(:, 6:8);
attitudes = truth_data(:, 9:11);  % [roll, pitch, yaw]

fprintf('Loaded truth data with %d epochs\n', length(times));
fprintf('Time range: %.3f to %.3f seconds\n', times(1), times(end));
fprintf('Duration: %.1f seconds\n', times(end) - times(1));

% Basic validation
fprintf('Position range (ECEF):\n');
fprintf('  X: [%.1f, %.1f] m\n', min(pos_ecef(:,1)), max(pos_ecef(:,1)));
fprintf('  Y: [%.1f, %.1f] m\n', min(pos_ecef(:,2)), max(pos_ecef(:,2)));
fprintf('  Z: [%.1f, %.1f] m\n', min(pos_ecef(:,3)), max(pos_ecef(:,3)));

fprintf('Attitude range (degrees):\n');
fprintf('  Roll: [%.1f, %.1f]°\n', rad2deg(min(attitudes(:,1))), rad2deg(max(attitudes(:,1))));
fprintf('  Pitch: [%.1f, %.1f]°\n', rad2deg(min(attitudes(:,2))), rad2deg(max(attitudes(:,2))));
fprintf('  Yaw: [%.1f, %.1f]°\n', rad2deg(min(attitudes(:,3))), rad2deg(max(attitudes(:,3))));
```

### Dataset Characteristics

#### Available Datasets

The repository includes three main dataset pairs:

**Dataset X001 (Baseline)**
- **IMU**: `IMU_X001.dat` - Clean IMU data with minimal noise
- **GNSS**: `GNSS_X001.csv` - Corresponding GNSS trajectory
- **Characteristics**: Low noise, stable trajectory, ideal for algorithm development
- **Duration**: ~1000 seconds of data
- **Trajectory**: Mixed static and dynamic motion

**Dataset X002 (Noisy)**
- **IMU**: `IMU_X002.dat` - IMU data with added measurement noise
- **GNSS**: `GNSS_X002.csv` - GNSS trajectory with increased uncertainty
- **Characteristics**: Higher noise levels, challenging for filtering algorithms
- **Duration**: ~1000 seconds of data
- **Trajectory**: Similar motion profile to X001 but with noise

**Dataset X003 (Biased)**
- **IMU**: `IMU_X003.dat` - IMU data with constant sensor biases
- **GNSS**: `GNSS_X002.csv` - Reuses GNSS_X002 trajectory
- **Characteristics**: Systematic biases test bias estimation algorithms
- **Duration**: ~1000 seconds of data
- **Trajectory**: Same as X002 but with IMU biases

#### Small Test Datasets

For rapid testing and development:

- **IMU Files**: `IMU_X001_small.dat`, `IMU_X002_small.dat`, `IMU_X003_small.dat`
  - First 1000 samples (~2.5 seconds at 400 Hz)
- **GNSS Files**: `GNSS_X001_small.csv`, `GNSS_X002_small.csv`
  - First 10 epochs (~10 seconds of GNSS data)
- **Truth File**: `STATE_X001_small.txt`
  - First 100 reference states

#### Data Synchronization

**Time Alignment Considerations:**

The repository handles several timing complexities:

1. **Different Sampling Rates**:
   - IMU: 400 Hz (2.5 ms intervals)
   - GNSS: 1 Hz (1 second intervals)
   - Truth: Variable rate (typically 10 Hz)

2. **Time Base Consistency**:
   - All timestamps use POSIX time (seconds since Unix epoch)
   - Automatic time alignment algorithms handle slight offsets
   - Truth data may need time shift correction

3. **Interpolation Requirements**:
   - IMU data often needs interpolation to match GNSS epochs
   - GNSS data interpolation for high-rate processing
   - Truth data interpolation for validation

**Synchronization Code Example:**

```python
def synchronize_imu_gnss_data(imu_data, gnss_data):
    """Synchronize IMU and GNSS data to common time base."""
    import scipy.interpolate as interp
    
    # Extract time vectors
    imu_times = imu_data[:, 1]
    gnss_times = gnss_data['Posix_Time'].values
    
    # Find common time range
    t_start = max(imu_times[0], gnss_times[0])
    t_end = min(imu_times[-1], gnss_times[-1])
    
    print(f"Common time range: {t_end - t_start:.1f} seconds")
    
    # Create common time base at GNSS rate
    common_times = gnss_times[(gnss_times >= t_start) & (gnss_times <= t_end)]
    
    # Interpolate IMU data to GNSS epochs
    imu_interp = np.zeros((len(common_times), 6))  # 3 gyro + 3 accel
    
    for i in range(6):
        col_idx = i + 2  # Skip sample number and time columns
        interp_func = interp.interp1d(imu_times, imu_data[:, col_idx], 
                                     kind='linear', bounds_error=False, fill_value=0)
        imu_interp[:, i] = interp_func(common_times)
    
    # Extract synchronized GNSS data
    mask = (gnss_times >= t_start) & (gnss_times <= t_end)
    gnss_sync = gnss_data[mask].reset_index(drop=True)
    
    return {
        'times': common_times,
        'imu_gyro': imu_interp[:, :3],
        'imu_accel': imu_interp[:, 3:],
        'gnss_pos': gnss_sync[['X_ECEF', 'Y_ECEF', 'Z_ECEF']].values,
        'gnss_vel': gnss_sync[['VX_ECEF', 'VY_ECEF', 'VZ_ECEF']].values
    }
```

### Custom Data Integration

#### Using Your Own Data

To process your own IMU and GNSS data with this repository:

**1. Convert IMU Data to Required Format:**

```python
def convert_custom_imu_data(your_imu_file, output_file):
    """Convert custom IMU data to repository format."""
    # Load your data (format depends on source)
    # Example assumes CSV with columns: timestamp, gx, gy, gz, ax, ay, az
    import pandas as pd
    
    custom_data = pd.read_csv(your_imu_file)
    
    # Convert to required format
    output_data = np.zeros((len(custom_data), 8))
    output_data[:, 0] = np.arange(len(custom_data))  # Sample number
    output_data[:, 1] = custom_data['timestamp'].values  # Time (POSIX)
    
    # Convert rates to increments (multiply by dt)
    dt = np.mean(np.diff(custom_data['timestamp'].values))
    output_data[:, 2] = custom_data['gx'].values * dt  # Gyro X increment
    output_data[:, 3] = custom_data['gy'].values * dt  # Gyro Y increment  
    output_data[:, 4] = custom_data['gz'].values * dt  # Gyro Z increment
    output_data[:, 5] = custom_data['ax'].values * dt  # Accel X increment
    output_data[:, 6] = custom_data['ay'].values * dt  # Accel Y increment
    output_data[:, 7] = custom_data['az'].values * dt  # Accel Z increment
    
    # Save in binary format
    np.savetxt(output_file, output_data, fmt='%.9e')
    print(f"Converted IMU data saved to: {output_file}")
```

**2. Convert GNSS Data to Required Format:**

```python
def convert_custom_gnss_data(your_gnss_file, output_file):
    """Convert custom GNSS data to repository format."""
    import pandas as pd
    
    # Load your GNSS data
    custom_gnss = pd.read_csv(your_gnss_file)
    
    # Create output DataFrame with required columns
    output_gnss = pd.DataFrame()
    
    # Fill required columns (adapt to your data structure)
    output_gnss['Week'] = 0  # GPS week (can be computed if needed)
    output_gnss['TOW'] = 0   # GPS time of week (can be computed if needed)
    output_gnss['Posix_Time'] = custom_gnss['timestamp']
    
    # Geodetic coordinates (set to 0 if not available - will be computed from ECEF)
    output_gnss['Latitude'] = custom_gnss.get('lat', 0)
    output_gnss['Longitude'] = custom_gnss.get('lon', 0)  
    output_gnss['Height'] = custom_gnss.get('alt', 0)
    
    # ECEF coordinates (primary data)
    output_gnss['X_ECEF'] = custom_gnss['x_ecef']
    output_gnss['Y_ECEF'] = custom_gnss['y_ecef']
    output_gnss['Z_ECEF'] = custom_gnss['z_ecef']
    output_gnss['VX_ECEF'] = custom_gnss['vx_ecef']
    output_gnss['VY_ECEF'] = custom_gnss['vy_ecef']
    output_gnss['VZ_ECEF'] = custom_gnss['vz_ecef']
    
    # Save to CSV
    output_gnss.to_csv(output_file, index=False)
    print(f"Converted GNSS data saved to: {output_file}")
```

**3. Integration Example:**

```python
# Convert your data
convert_custom_imu_data('my_imu_data.csv', 'DATA/IMU/IMU_CUSTOM.dat')
convert_custom_gnss_data('my_gnss_data.csv', 'DATA/GNSS/GNSS_CUSTOM.csv')

# Process with the repository tools
from PYTHON.src.GNSS_IMU_Fusion import main as run_fusion

# Run processing pipeline
run_fusion_args = [
    '--imu-file', 'IMU_CUSTOM.dat',
    '--gnss-file', 'GNSS_CUSTOM.csv', 
    '--method', 'TRIAD'
]

# Process (this would normally be called from command line)
# python PYTHON/src/GNSS_IMU_Fusion.py --imu-file IMU_CUSTOM.dat --gnss-file GNSS_CUSTOM.csv --method TRIAD
```

This comprehensive section covers all aspects of data formats and structures used in the IMU repository, providing users with the knowledge needed to understand existing data and integrate their own datasets.

---

## Core Concepts and Algorithms

### Fundamental Navigation Concepts

#### Coordinate Systems

The repository uses several coordinate systems, each serving specific purposes in the navigation solution:

**1. Body Frame (b-frame)**
- **Origin**: IMU sensor location
- **X-axis**: Forward direction of the vehicle/platform
- **Y-axis**: Right direction (starboard)
- **Z-axis**: Down direction (completing right-handed system)
- **Usage**: Raw IMU measurements are expressed in this frame

**2. Navigation Frame (n-frame) - NED**
- **Origin**: Local geographic reference point
- **X-axis**: North direction
- **Y-axis**: East direction  
- **Z-axis**: Down direction (local vertical)
- **Usage**: Navigation solutions, attitude representation

**3. Earth-Centered Earth-Fixed Frame (e-frame) - ECEF**
- **Origin**: Earth's center of mass
- **X-axis**: Intersection of equatorial plane and prime meridian
- **Y-axis**: 90° east longitude in equatorial plane
- **Z-axis**: North polar direction
- **Usage**: GNSS measurements, absolute positioning

**4. Inertial Frame (i-frame)**
- **Origin**: Earth's center of mass
- **Axes**: Fixed relative to distant stars
- **Usage**: Theoretical reference for inertial navigation equations

#### Coordinate Transformations

**Body to Navigation (b→n):**
```
r_n = C_b^n * r_b
```
Where `C_b^n` is the direction cosine matrix (DCM) representing attitude.

**Navigation to ECEF (n→e):**
```
r_e = C_n^e * r_n + r_ref_e
```
Where `r_ref_e` is the reference position in ECEF.

**Python Implementation:**
```python
import numpy as np

def euler_to_dcm(roll, pitch, yaw):
    """Convert Euler angles to Direction Cosine Matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)  
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    # DCM from body to navigation frame
    C_bn = np.array([
        [cp*cy, -cr*sy + sr*sp*cy,  sr*sy + cr*sp*cy],
        [cp*sy,  cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy],
        [-sp,    sr*cp,             cr*cp]
    ])
    
    return C_bn

def dcm_to_euler(C_bn):
    """Convert Direction Cosine Matrix to Euler angles."""
    roll = np.arctan2(C_bn[2,1], C_bn[2,2])
    pitch = -np.arcsin(C_bn[2,0])
    yaw = np.arctan2(C_bn[1,0], C_bn[0,0])
    return roll, pitch, yaw

def ned_to_ecef(pos_ned, ref_lat, ref_lon, ref_alt):
    """Transform NED position to ECEF coordinates."""
    # WGS84 ellipsoid parameters
    a = 6378137.0  # Semi-major axis
    f = 1/298.257223563  # Flattening
    e2 = f * (2 - f)  # Eccentricity squared
    
    lat_rad = np.radians(ref_lat)
    lon_rad = np.radians(ref_lon)
    
    # Prime vertical radius of curvature
    N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)
    
    # Reference position in ECEF
    ref_ecef = np.array([
        (N + ref_alt) * np.cos(lat_rad) * np.cos(lon_rad),
        (N + ref_alt) * np.cos(lat_rad) * np.sin(lon_rad),
        (N * (1 - e2) + ref_alt) * np.sin(lat_rad)
    ])
    
    # Local to ECEF transformation matrix
    sin_lat, cos_lat = np.sin(lat_rad), np.cos(lat_rad)
    sin_lon, cos_lon = np.sin(lon_rad), np.cos(lon_rad)
    
    C_en = np.array([
        [-sin_lat*cos_lon, -sin_lon, -cos_lat*cos_lon],
        [-sin_lat*sin_lon,  cos_lon, -cos_lat*sin_lon],
        [cos_lat,           0,        -sin_lat]
    ])
    
    # Transform NED to ECEF
    pos_ecef = ref_ecef + C_en.T @ pos_ned
    return pos_ecef
```

**MATLAB Implementation:**
```matlab
function C_bn = euler_to_dcm(roll, pitch, yaw)
    % Convert Euler angles to Direction Cosine Matrix
    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);
    
    % DCM from body to navigation frame  
    C_bn = [cp*cy, -cr*sy + sr*sp*cy,  sr*sy + cr*sp*cy;
            cp*sy,  cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy;
            -sp,    sr*cp,             cr*cp];
end

function [roll, pitch, yaw] = dcm_to_euler(C_bn)
    % Convert Direction Cosine Matrix to Euler angles
    roll = atan2(C_bn(3,2), C_bn(3,3));
    pitch = -asin(C_bn(3,1));
    yaw = atan2(C_bn(2,1), C_bn(1,1));
end

function pos_ecef = ned_to_ecef(pos_ned, ref_lat, ref_lon, ref_alt)
    % Transform NED position to ECEF coordinates
    % WGS84 ellipsoid parameters
    a = 6378137.0;  % Semi-major axis
    f = 1/298.257223563;  % Flattening
    e2 = f * (2 - f);  % Eccentricity squared
    
    lat_rad = deg2rad(ref_lat);
    lon_rad = deg2rad(ref_lon);
    
    % Prime vertical radius of curvature
    N = a / sqrt(1 - e2 * sin(lat_rad)^2);
    
    % Reference position in ECEF
    ref_ecef = [(N + ref_alt) * cos(lat_rad) * cos(lon_rad);
                (N + ref_alt) * cos(lat_rad) * sin(lon_rad);
                (N * (1 - e2) + ref_alt) * sin(lat_rad)];
    
    % Local to ECEF transformation matrix
    sin_lat = sin(lat_rad); cos_lat = cos(lat_rad);
    sin_lon = sin(lon_rad); cos_lon = cos(lon_rad);
    
    C_en = [-sin_lat*cos_lon, -sin_lon, -cos_lat*cos_lon;
            -sin_lat*sin_lon,  cos_lon, -cos_lat*sin_lon;
            cos_lat,           0,        -sin_lat];
    
    % Transform NED to ECEF
    pos_ecef = ref_ecef + C_en' * pos_ned;
end
```

### Attitude Determination Algorithms

The repository implements three primary methods for initial attitude determination:

#### 1. TRIAD Algorithm

The TRIAD algorithm determines attitude using two non-collinear vector observations.

**Mathematical Foundation:**
Given two vectors measured in body frame (`v1_b`, `v2_b`) and their known directions in navigation frame (`v1_n`, `v2_n`):

1. **Construct orthonormal triads:**
```
t1_b = v1_b / |v1_b|
t2_b = (v1_b × v2_b) / |v1_b × v2_b|  
t3_b = t1_b × t2_b

t1_n = v1_n / |v1_n|
t2_n = (v1_n × v2_n) / |v1_n × v2_n|
t3_n = t1_n × t2_n
```

2. **Form transformation matrix:**
```
C_bn = [t1_n t2_n t3_n] * [t1_b t2_b t3_b]^T
```

**Python Implementation:**
```python
def triad_attitude(v1_body, v2_body, v1_nav, v2_nav):
    """
    TRIAD attitude determination algorithm.
    
    Parameters:
    -----------
    v1_body, v2_body : array_like, shape (3,)
        Two non-collinear vectors measured in body frame
    v1_nav, v2_nav : array_like, shape (3,) 
        Corresponding vectors in navigation frame
        
    Returns:
    --------
    C_bn : ndarray, shape (3,3)
        Direction cosine matrix from body to navigation frame
    """
    # Normalize input vectors
    v1_b = np.array(v1_body) / np.linalg.norm(v1_body)
    v2_b = np.array(v2_body) / np.linalg.norm(v2_body)
    v1_n = np.array(v1_nav) / np.linalg.norm(v1_nav)
    v2_n = np.array(v2_nav) / np.linalg.norm(v2_nav)
    
    # Check that vectors are not collinear
    if np.abs(np.dot(v1_b, v2_b)) > 0.95:
        raise ValueError("Input vectors are too close to collinear")
    
    # Construct body frame triad
    t1_b = v1_b
    t2_b = np.cross(v1_b, v2_b)
    t2_b = t2_b / np.linalg.norm(t2_b)
    t3_b = np.cross(t1_b, t2_b)
    
    # Construct navigation frame triad
    t1_n = v1_n
    t2_n = np.cross(v1_n, v2_n)
    t2_n = t2_n / np.linalg.norm(t2_n)
    t3_n = np.cross(t1_n, t2_n)
    
    # Form triads as matrices
    T_b = np.column_stack([t1_b, t2_b, t3_b])
    T_n = np.column_stack([t1_n, t2_n, t3_n])
    
    # Compute attitude matrix
    C_bn = T_n @ T_b.T
    
    # Ensure proper rotation matrix (orthonormal)
    U, _, Vt = np.linalg.svd(C_bn)
    C_bn = U @ Vt
    
    return C_bn

# Example usage
gravity_body = np.array([0.1, 0.2, 9.8])  # Gravity measured in body frame
earth_rate_body = np.array([0.0001, -0.0005, 0.0002])  # Earth rate in body frame

gravity_nav = np.array([0, 0, 9.81])  # Gravity in NED frame (down)
earth_rate_nav = np.array([0.0000729, 0, 0])  # Earth rate in NED (north component)

C_bn_triad = triad_attitude(gravity_body, earth_rate_body, gravity_nav, earth_rate_nav)
roll, pitch, yaw = dcm_to_euler(C_bn_triad)
print(f"TRIAD attitude: roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°")
```

**MATLAB Implementation:**
```matlab
function C_bn = triad_attitude(v1_body, v2_body, v1_nav, v2_nav)
    % TRIAD attitude determination algorithm
    
    % Normalize input vectors
    v1_b = v1_body(:) / norm(v1_body);
    v2_b = v2_body(:) / norm(v2_body);
    v1_n = v1_nav(:) / norm(v1_nav);
    v2_n = v2_nav(:) / norm(v2_nav);
    
    % Check collinearity
    if abs(dot(v1_b, v2_b)) > 0.95
        error('Input vectors are too close to collinear');
    end
    
    % Construct body frame triad
    t1_b = v1_b;
    t2_b = cross(v1_b, v2_b);
    t2_b = t2_b / norm(t2_b);
    t3_b = cross(t1_b, t2_b);
    
    % Construct navigation frame triad
    t1_n = v1_n;
    t2_n = cross(v1_n, v2_n);
    t2_n = t2_n / norm(t2_n);
    t3_n = cross(t1_n, t2_n);
    
    % Form triads as matrices
    T_b = [t1_b, t2_b, t3_b];
    T_n = [t1_n, t2_n, t3_n];
    
    % Compute attitude matrix
    C_bn = T_n * T_b';
    
    % Ensure proper rotation matrix
    [U, ~, V] = svd(C_bn);
    C_bn = U * V';
end
```

#### 2. Davenport's q-Method

Davenport's method finds the optimal quaternion that minimizes the weighted sum of squared residuals between vector observations.

**Mathematical Foundation:**
Minimize the cost function:
```
J(q) = Σ aᵢ |vᵢⁿ - C(q) vᵢᵇ|²
```

Where:
- `aᵢ` are weights for each vector pair
- `C(q)` is the rotation matrix corresponding to quaternion `q`

This leads to solving the eigenvalue problem:
```
K * q = λ * q
```

Where `K` is the Davenport matrix and the optimal quaternion corresponds to the largest eigenvalue.

**Python Implementation:**
```python
def davenport_q_method(vectors_body, vectors_nav, weights=None):
    """
    Davenport's q-method for optimal attitude determination.
    
    Parameters:
    -----------
    vectors_body : array_like, shape (n, 3)
        Vectors measured in body frame
    vectors_nav : array_like, shape (n, 3)
        Corresponding vectors in navigation frame
    weights : array_like, shape (n,), optional
        Weights for each vector pair (default: equal weights)
        
    Returns:
    --------
    q : ndarray, shape (4,)
        Optimal quaternion [qw, qx, qy, qz] (scalar-first convention)
    C_bn : ndarray, shape (3,3)
        Corresponding direction cosine matrix
    """
    vectors_body = np.array(vectors_body)
    vectors_nav = np.array(vectors_nav)
    
    if vectors_body.ndim == 1:
        vectors_body = vectors_body.reshape(1, -1)
    if vectors_nav.ndim == 1:
        vectors_nav = vectors_nav.reshape(1, -1)
    
    n_vectors = len(vectors_body)
    
    if weights is None:
        weights = np.ones(n_vectors)
    weights = np.array(weights)
    
    # Normalize vectors
    for i in range(n_vectors):
        vectors_body[i] = vectors_body[i] / np.linalg.norm(vectors_body[i])
        vectors_nav[i] = vectors_nav[i] / np.linalg.norm(vectors_nav[i])
    
    # Build attitude profile matrix B
    B = np.zeros((3, 3))
    for i in range(n_vectors):
        B += weights[i] * np.outer(vectors_body[i], vectors_nav[i])
    
    # Build Davenport's K matrix
    S = B + B.T
    Z = np.array([B[1,2] - B[2,1], B[2,0] - B[0,2], B[0,1] - B[1,0]])
    sigma = np.trace(B)
    
    K = np.zeros((4, 4))
    K[0,0] = sigma
    K[0,1:4] = Z
    K[1:4,0] = Z
    K[1:4,1:4] = S - sigma * np.eye(3)
    
    # Find eigenvalues and eigenvectors
    eigenvals, eigenvecs = np.linalg.eigh(K)
    
    # Select eigenvector corresponding to largest eigenvalue
    max_idx = np.argmax(eigenvals)
    q = eigenvecs[:, max_idx]
    
    # Ensure positive scalar part (quaternion uniqueness)
    if q[0] < 0:
        q = -q
    
    # Convert quaternion to rotation matrix
    C_bn = quaternion_to_dcm(q)
    
    return q, C_bn

def quaternion_to_dcm(q):
    """Convert quaternion to direction cosine matrix."""
    qw, qx, qy, qz = q
    
    # Normalize quaternion
    norm_q = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw/norm_q, qx/norm_q, qy/norm_q, qz/norm_q
    
    # Compute rotation matrix elements
    C_bn = np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)],
        [2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2), 2*(qy*qz-qw*qx)],
        [2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2)]
    ])
    
    return C_bn

# Example usage with multiple vectors
vectors_body = np.array([
    [0.1, 0.2, 9.8],      # Gravity measurement
    [0.0001, -0.0005, 0.0002],  # Earth rate measurement
    [0.2, 0.1, -0.05]     # Additional vector (e.g., magnetic field)
])

vectors_nav = np.array([
    [0, 0, 9.81],         # Gravity reference  
    [0.0000729, 0, 0],    # Earth rate reference
    [0.22, 0.05, 0.15]    # Magnetic field reference
])

weights = np.array([1.0, 0.5, 0.3])  # Higher weight for more accurate measurements

q_opt, C_bn_davenport = davenport_q_method(vectors_body, vectors_nav, weights)
roll, pitch, yaw = dcm_to_euler(C_bn_davenport)
print(f"Davenport attitude: roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°")
```

#### 3. SVD Method

The Singular Value Decomposition method solves Wahba's problem by finding the orthogonal matrix that minimizes the Frobenius norm.

**Mathematical Foundation:**
Minimize:
```
J(C) = Σ aᵢ |vᵢⁿ - C vᵢᵇ|²
```

This is equivalent to maximizing:
```
trace(C * B)
```

Where `B = Σ aᵢ vᵢᵇ (vᵢⁿ)ᵀ`

**Python Implementation:**
```python
def svd_attitude(vectors_body, vectors_nav, weights=None):
    """
    SVD method for attitude determination.
    
    Parameters:
    -----------
    vectors_body : array_like, shape (n, 3)
        Vectors measured in body frame
    vectors_nav : array_like, shape (n, 3)
        Corresponding vectors in navigation frame  
    weights : array_like, shape (n,), optional
        Weights for each vector pair
        
    Returns:
    --------
    C_bn : ndarray, shape (3,3)
        Optimal direction cosine matrix
    """
    vectors_body = np.array(vectors_body)
    vectors_nav = np.array(vectors_nav)
    
    if vectors_body.ndim == 1:
        vectors_body = vectors_body.reshape(1, -1)
    if vectors_nav.ndim == 1:
        vectors_nav = vectors_nav.reshape(1, -1)
    
    n_vectors = len(vectors_body)
    
    if weights is None:
        weights = np.ones(n_vectors)
    weights = np.array(weights)
    
    # Normalize vectors
    for i in range(n_vectors):
        vectors_body[i] = vectors_body[i] / np.linalg.norm(vectors_body[i])
        vectors_nav[i] = vectors_nav[i] / np.linalg.norm(vectors_nav[i])
    
    # Build attitude profile matrix B
    B = np.zeros((3, 3))
    for i in range(n_vectors):
        B += weights[i] * np.outer(vectors_body[i], vectors_nav[i])
    
    # Perform SVD
    U, S, Vt = np.linalg.svd(B)
    
    # Construct rotation matrix
    C_bn = Vt.T @ U.T
    
    # Ensure proper rotation (det = +1)
    if np.linalg.det(C_bn) < 0:
        # Flip the last column of V
        Vt_corrected = Vt.copy()
        Vt_corrected[-1, :] *= -1
        C_bn = Vt_corrected.T @ U.T
    
    return C_bn

# Example usage
C_bn_svd = svd_attitude(vectors_body, vectors_nav, weights)
roll, pitch, yaw = dcm_to_euler(C_bn_svd)
print(f"SVD attitude: roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°")
```

### Extended Kalman Filter Implementation

The repository implements a comprehensive Extended Kalman Filter (EKF) for IMU/GNSS fusion.

#### State Vector

The EKF state vector includes:
```
x = [r_n; v_n; att_err; b_a; b_g]
```

Where:
- `r_n`: Position in navigation frame (3×1)
- `v_n`: Velocity in navigation frame (3×1)  
- `att_err`: Attitude error angles (3×1)
- `b_a`: Accelerometer biases (3×1)
- `b_g`: Gyroscope biases (3×1)

Total state dimension: 15×1

#### System Model

**Prediction (IMU Integration):**
```python
def ekf_predict(state, P, imu_gyro, imu_accel, dt, Q):
    """
    EKF prediction step using IMU measurements.
    
    Parameters:
    -----------
    state : array_like, shape (15,)
        Current state estimate [pos, vel, att_err, accel_bias, gyro_bias]
    P : array_like, shape (15, 15)
        Current error covariance matrix
    imu_gyro : array_like, shape (3,)
        Gyroscope measurements [rad/s]
    imu_accel : array_like, shape (3,)
        Accelerometer measurements [m/s²]
    dt : float
        Time step [s]
    Q : array_like, shape (15, 15)
        Process noise covariance matrix
        
    Returns:
    --------
    state_pred : ndarray, shape (15,)
        Predicted state
    P_pred : ndarray, shape (15, 15)
        Predicted covariance
    """
    # Extract state components
    pos = state[:3]
    vel = state[3:6] 
    att_err = state[6:9]
    accel_bias = state[9:12]
    gyro_bias = state[12:15]
    
    # Current attitude from small angle approximation
    C_bn = euler_to_dcm(att_err[0], att_err[1], att_err[2])
    
    # Correct IMU measurements
    gyro_corrected = imu_gyro - gyro_bias
    accel_corrected = imu_accel - accel_bias
    
    # Navigation-frame acceleration
    gravity_nav = np.array([0, 0, 9.81])
    accel_nav = C_bn @ accel_corrected - gravity_nav
    
    # State propagation (simple integration)
    pos_new = pos + vel * dt + 0.5 * accel_nav * dt**2
    vel_new = vel + accel_nav * dt
    
    # Attitude propagation (small angle approximation)
    omega_nav = C_bn @ gyro_corrected
    att_err_new = att_err + omega_nav * dt
    
    # Bias propagation (random walk model)
    accel_bias_new = accel_bias  # Assumed constant
    gyro_bias_new = gyro_bias    # Assumed constant
    
    # Assemble predicted state
    state_pred = np.concatenate([pos_new, vel_new, att_err_new, 
                                accel_bias_new, gyro_bias_new])
    
    # Jacobian matrix F (linearized system matrix)
    F = np.zeros((15, 15))
    
    # Position-velocity coupling
    F[:3, 3:6] = np.eye(3)
    
    # Velocity-acceleration coupling
    F[3:6, 6:9] = -skew_symmetric(C_bn @ accel_corrected)
    F[3:6, 9:12] = -C_bn
    
    # Attitude dynamics
    F[6:9, 12:15] = -C_bn
    
    # Discrete-time state transition matrix
    Phi = np.eye(15) + F * dt
    
    # Covariance propagation
    P_pred = Phi @ P @ Phi.T + Q
    
    return state_pred, P_pred

def skew_symmetric(v):
    """Create skew-symmetric matrix from vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])
```

**Update (GNSS Measurement):**
```python
def ekf_update(state_pred, P_pred, gnss_pos, gnss_vel, R):
    """
    EKF update step using GNSS measurements.
    
    Parameters:
    -----------
    state_pred : array_like, shape (15,)
        Predicted state estimate
    P_pred : array_like, shape (15, 15)
        Predicted covariance matrix
    gnss_pos : array_like, shape (3,)
        GNSS position measurement [m]
    gnss_vel : array_like, shape (3,)
        GNSS velocity measurement [m/s]
    R : array_like, shape (6, 6)
        Measurement noise covariance matrix
        
    Returns:
    --------
    state_updated : ndarray, shape (15,)
        Updated state estimate
    P_updated : ndarray, shape (15, 15)
        Updated covariance matrix
    residual : ndarray, shape (6,)
        Measurement residual
    """
    # Predicted measurements (position and velocity)
    predicted_pos = state_pred[:3]
    predicted_vel = state_pred[3:6]
    
    # Measurement residual
    residual = np.concatenate([gnss_pos - predicted_pos, 
                              gnss_vel - predicted_vel])
    
    # Measurement matrix (observes position and velocity)
    H = np.zeros((6, 15))
    H[:3, :3] = np.eye(3)   # Position observation
    H[3:6, 3:6] = np.eye(3) # Velocity observation
    
    # Innovation covariance
    S = H @ P_pred @ H.T + R
    
    # Kalman gain
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    # State update
    state_updated = state_pred + K @ residual
    
    # Covariance update (Joseph form for numerical stability)
    I_KH = np.eye(15) - K @ H
    P_updated = I_KH @ P_pred @ I_KH.T + K @ R @ K.T
    
    return state_updated, P_updated, residual
```

#### Complete EKF Integration Example:

```python
def run_ekf_fusion(imu_data, gnss_data, initial_state, initial_P):
    """
    Complete EKF-based IMU/GNSS fusion example.
    
    Parameters:
    -----------
    imu_data : dict
        Contains 'times', 'gyro', 'accel' arrays
    gnss_data : dict  
        Contains 'times', 'pos', 'vel' arrays
    initial_state : array_like, shape (15,)
        Initial state estimate
    initial_P : array_like, shape (15, 15)
        Initial covariance matrix
        
    Returns:
    --------
    results : dict
        Contains fusion results and statistics
    """
    
    # Initialize arrays for results
    n_imu = len(imu_data['times'])
    n_gnss = len(gnss_data['times'])
    
    # Storage for results
    states = np.zeros((n_imu, 15))
    covariances = np.zeros((n_imu, 15, 15))
    residuals = []
    
    # Process noise (tuned parameters)
    Q = np.diag([
        1e-8, 1e-8, 1e-8,     # Position process noise
        1e-6, 1e-6, 1e-6,     # Velocity process noise  
        1e-8, 1e-8, 1e-8,     # Attitude process noise
        1e-10, 1e-10, 1e-10,  # Accel bias process noise
        1e-12, 1e-12, 1e-12   # Gyro bias process noise
    ])
    
    # Measurement noise (typical GNSS accuracies)
    R = np.diag([
        1.0, 1.0, 2.0,        # Position measurement noise [m]
        0.1, 0.1, 0.2         # Velocity measurement noise [m/s]
    ])
    
    # Initialize filter
    state = initial_state.copy()
    P = initial_P.copy()
    
    # Main processing loop
    gnss_idx = 0
    
    for i in range(n_imu):
        t_imu = imu_data['times'][i]
        
        # Get IMU measurements
        gyro = imu_data['gyro'][i]
        accel = imu_data['accel'][i]
        
        # Compute time step
        if i > 0:
            dt = t_imu - imu_data['times'][i-1]
        else:
            dt = 0.0025  # Default IMU rate
        
        # Prediction step
        if dt > 0:
            state, P = ekf_predict(state, P, gyro, accel, dt, Q)
        
        # Check if GNSS measurement is available
        if gnss_idx < n_gnss and abs(t_imu - gnss_data['times'][gnss_idx]) < 0.1:
            # Update step with GNSS
            gnss_pos = gnss_data['pos'][gnss_idx]
            gnss_vel = gnss_data['vel'][gnss_idx]
            
            state, P, residual = ekf_update(state, P, gnss_pos, gnss_vel, R)
            residuals.append({
                'time': t_imu,
                'residual': residual,
                'innovation_cov': P[:6, :6]
            })
            
            gnss_idx += 1
        
        # Store results
        states[i] = state
        covariances[i] = P
    
    # Compile results
    results = {
        'times': imu_data['times'],
        'states': states,
        'covariances': covariances,
        'residuals': residuals,
        'position': states[:, :3],
        'velocity': states[:, 3:6],
        'attitude_errors': states[:, 6:9],
        'accel_biases': states[:, 9:12],
        'gyro_biases': states[:, 12:15]
    }
    
    return results

# Usage example:
# Assuming synchronized IMU and GNSS data dictionaries are available
# results = run_ekf_fusion(imu_data, gnss_data, initial_state, initial_P)
```

This section provides a comprehensive overview of the core navigation concepts and algorithms implemented in the IMU repository, giving users the theoretical foundation needed to understand and modify the processing pipeline.

---

## Task-by-Task Tutorial

### Task 1: Reference Vector Computation

**Objective**: Establish reference vectors (gravity and Earth rotation) in the navigation frame using the initial GNSS position.

#### Theory

The reference vectors provide the foundation for attitude determination:
- **Gravity Vector**: Points downward in the local NED frame with magnitude ~9.81 m/s²
- **Earth Rotation Vector**: Points north with magnitude 7.29×10⁻⁵ rad/s at the equator

#### Python Implementation

```python
# Complete Task 1 example
from PYTHON.src.task1_reference_vectors import compute_reference_vectors
from PYTHON.src.paths import gnss_path
import pandas as pd

def run_task1_python(dataset_id='X001'):
    """Complete Task 1 implementation in Python."""
    
    # Load GNSS data
    gnss_file = gnss_path(dataset_id)
    gnss_data = pd.read_csv(gnss_file)
    
    # Get initial position (first valid GNSS fix)
    first_pos_ecef = gnss_data[['X_ECEF', 'Y_ECEF', 'Z_ECEF']].iloc[0].values
    
    # Convert to geodetic coordinates
    from PYTHON.src.utils import ecef_to_geodetic
    lat, lon, alt = ecef_to_geodetic(first_pos_ecef)
    
    print(f"Initial position: {lat:.6f}°N, {lon:.6f}°E, {alt:.1f}m")
    
    # Compute reference vectors in NED frame
    gravity_ned = np.array([0, 0, 9.81])  # Downward
    
    # Earth rotation rate (WGS84)
    omega_earth = 7.2921159e-5  # rad/s
    earth_rate_ned = np.array([
        omega_earth * np.cos(np.radians(lat)),  # North component
        0,                                       # East component  
        -omega_earth * np.sin(np.radians(lat))  # Down component
    ])
    
    print(f"Gravity vector (NED): {gravity_ned}")
    print(f"Earth rate vector (NED): {earth_rate_ned}")
    print(f"Earth rate magnitude: {np.linalg.norm(earth_rate_ned):.6e} rad/s")
    
    # Save results
    results = {
        'latitude': lat,
        'longitude': lon,
        'altitude': alt,
        'gravity_ned': gravity_ned,
        'earth_rate_ned': earth_rate_ned,
        'reference_ecef': first_pos_ecef
    }
    
    return results
```

#### MATLAB Implementation

```matlab
function results = run_task1_matlab(dataset_id)
    % Complete Task 1 implementation in MATLAB
    if nargin < 1, dataset_id = 'X001'; end
    
    % Load GNSS data
    gnss_file = fullfile('DATA', 'GNSS', ['GNSS_' dataset_id '.csv']);
    gnss_data = readtable(gnss_file);
    
    % Get initial position
    first_pos_ecef = [gnss_data.X_ECEF(1), gnss_data.Y_ECEF(1), gnss_data.Z_ECEF(1)];
    
    % Convert to geodetic
    [lat, lon, alt] = ecef2geodetic(first_pos_ecef);
    
    fprintf('Initial position: %.6f°N, %.6f°E, %.1fm\n', lat, lon, alt);
    
    % Compute reference vectors in NED frame
    gravity_ned = [0; 0; 9.81];  % Downward
    
    % Earth rotation rate
    omega_earth = 7.2921159e-5;  % rad/s
    earth_rate_ned = [
        omega_earth * cosd(lat);   % North component
        0;                         % East component
        -omega_earth * sind(lat)   % Down component
    ];
    
    fprintf('Gravity vector (NED): [%.3f, %.3f, %.3f]\n', gravity_ned);
    fprintf('Earth rate vector (NED): [%.6e, %.6e, %.6e]\n', earth_rate_ned);
    fprintf('Earth rate magnitude: %.6e rad/s\n', norm(earth_rate_ned));
    
    % Create results structure
    results = struct();
    results.latitude = lat;
    results.longitude = lon;
    results.altitude = alt;
    results.gravity_ned = gravity_ned;
    results.earth_rate_ned = earth_rate_ned;
    results.reference_ecef = first_pos_ecef;
    
    % Save to results directory
    save(fullfile('MATLAB', 'results', ['Task1_' dataset_id '.mat']), 'results');
end
```

### Task 2: Body Frame Vector Measurement

**Objective**: Measure gravity and Earth rotation vectors in the body frame during a static period, and estimate sensor biases.

#### Theory

During static periods:
- Accelerometers measure gravity + bias
- Gyroscopes measure Earth rotation + bias
- Bias estimation uses mean values during detected static intervals

#### Static Period Detection Algorithm

```python
def detect_static_period(imu_data, window_size=200, accel_threshold=0.01, gyro_threshold=1e-6):
    """
    Detect static periods in IMU data using sliding variance.
    
    Parameters:
    -----------
    imu_data : dict
        Contains 'accel' and 'gyro' arrays
    window_size : int
        Size of sliding window for variance computation
    accel_threshold : float
        Variance threshold for accelerometer [m²/s⁴]
    gyro_threshold : float
        Variance threshold for gyroscope [rad²/s²]
        
    Returns:
    --------
    static_mask : ndarray
        Boolean array indicating static samples
    longest_segment : tuple
        (start_idx, end_idx) of longest static segment
    """
    
    accel = imu_data['accel']
    gyro = imu_data['gyro']
    n_samples = len(accel)
    
    # Sliding variance computation
    accel_var = np.zeros((n_samples - window_size + 1, 3))
    gyro_var = np.zeros((n_samples - window_size + 1, 3))
    
    for i in range(n_samples - window_size + 1):
        accel_window = accel[i:i+window_size]
        gyro_window = gyro[i:i+window_size]
        
        accel_var[i] = np.var(accel_window, axis=0)
        gyro_var[i] = np.var(gyro_window, axis=0)
    
    # Static detection criteria
    accel_static = np.max(accel_var, axis=1) < accel_threshold
    gyro_static = np.max(gyro_var, axis=1) < gyro_threshold
    static_mask = accel_static & gyro_static
    
    # Pad mask to original length
    full_static_mask = np.zeros(n_samples, dtype=bool)
    full_static_mask[window_size//2:-window_size//2+1] = static_mask
    
    # Find longest continuous static segment
    static_segments = []
    in_segment = False
    start_idx = 0
    
    for i, is_static in enumerate(full_static_mask):
        if is_static and not in_segment:
            start_idx = i
            in_segment = True
        elif not is_static and in_segment:
            static_segments.append((start_idx, i-1))
            in_segment = False
    
    # Handle case where data ends during static period
    if in_segment:
        static_segments.append((start_idx, n_samples-1))
    
    # Find longest segment
    if static_segments:
        longest_segment = max(static_segments, key=lambda x: x[1] - x[0])
    else:
        longest_segment = (0, min(1000, n_samples-1))  # Fallback
    
    return full_static_mask, longest_segment

def estimate_sensor_biases(imu_data, static_segment, gravity_ned, earth_rate_ned):
    """
    Estimate accelerometer and gyroscope biases during static period.
    
    Parameters:
    -----------
    imu_data : dict
        IMU data containing 'accel' and 'gyro'
    static_segment : tuple
        (start_idx, end_idx) of static period
    gravity_ned : array_like
        Expected gravity vector in NED frame
    earth_rate_ned : array_like
        Expected Earth rate in NED frame
        
    Returns:
    --------
    accel_bias : ndarray, shape (3,)
        Estimated accelerometer bias
    gyro_bias : ndarray, shape (3,)
        Estimated gyroscope bias
    measured_gravity_body : ndarray, shape (3,)
        Measured gravity vector in body frame
    measured_earth_rate_body : ndarray, shape (3,)
        Measured Earth rate vector in body frame
    """
    
    start_idx, end_idx = static_segment
    
    # Extract static measurements
    static_accel = imu_data['accel'][start_idx:end_idx+1]
    static_gyro = imu_data['gyro'][start_idx:end_idx+1]
    
    # Compute means (measured vectors in body frame)
    measured_gravity_body = np.mean(static_accel, axis=0)
    measured_earth_rate_body = np.mean(static_gyro, axis=0)
    
    print(f"Static period: samples {start_idx} to {end_idx} ({end_idx-start_idx+1} samples)")
    print(f"Measured gravity (body): {measured_gravity_body}")
    print(f"Measured Earth rate (body): {measured_earth_rate_body}")
    
    # Initial bias estimates (refined after attitude determination)
    # For now, assume biases are the difference from expected magnitudes
    gravity_magnitude_expected = np.linalg.norm(gravity_ned)
    gravity_magnitude_measured = np.linalg.norm(measured_gravity_body)
    
    earth_rate_magnitude_expected = np.linalg.norm(earth_rate_ned)
    earth_rate_magnitude_measured = np.linalg.norm(measured_earth_rate_body)
    
    # Rough bias estimates (will be refined in later tasks)
    accel_bias = np.zeros(3)  # Will be computed after attitude determination
    gyro_bias = np.zeros(3)   # Will be computed after attitude determination
    
    print(f"Gravity magnitude - Expected: {gravity_magnitude_expected:.3f}, Measured: {gravity_magnitude_measured:.3f}")
    print(f"Earth rate magnitude - Expected: {earth_rate_magnitude_expected:.6e}, Measured: {earth_rate_magnitude_measured:.6e}")
    
    return accel_bias, gyro_bias, measured_gravity_body, measured_earth_rate_body
```

### Task 3: Initial Attitude Determination

**Objective**: Compute initial attitude using TRIAD, Davenport, or SVD methods.

#### Complete Implementation

```python
def run_task3_attitude_determination(reference_vectors, body_vectors, method='TRIAD'):
    """
    Complete Task 3: Initial attitude determination.
    
    Parameters:
    -----------
    reference_vectors : dict
        Contains 'gravity_ned' and 'earth_rate_ned'
    body_vectors : dict
        Contains 'measured_gravity_body' and 'measured_earth_rate_body'
    method : str
        Attitude determination method ('TRIAD', 'Davenport', 'SVD')
        
    Returns:
    --------
    results : dict
        Contains attitude matrix, Euler angles, and method-specific data
    """
    
    # Extract vectors
    gravity_ned = reference_vectors['gravity_ned']
    earth_rate_ned = reference_vectors['earth_rate_ned']
    gravity_body = body_vectors['measured_gravity_body']
    earth_rate_body = body_vectors['measured_earth_rate_body']
    
    # Normalize vectors for attitude determination
    gravity_body_norm = gravity_body / np.linalg.norm(gravity_body)
    earth_rate_body_norm = earth_rate_body / np.linalg.norm(earth_rate_body)
    gravity_ned_norm = gravity_ned / np.linalg.norm(gravity_ned)
    earth_rate_ned_norm = earth_rate_ned / np.linalg.norm(earth_rate_ned)
    
    print(f"Computing initial attitude using {method} method")
    
    if method == 'TRIAD':
        C_bn = triad_attitude(gravity_body_norm, earth_rate_body_norm,
                             gravity_ned_norm, earth_rate_ned_norm)
        additional_data = {}
        
    elif method == 'Davenport':
        vectors_body = np.array([gravity_body_norm, earth_rate_body_norm])
        vectors_ned = np.array([gravity_ned_norm, earth_rate_ned_norm])
        weights = np.array([1.0, 0.5])  # Higher weight for gravity
        
        q_opt, C_bn = davenport_q_method(vectors_body, vectors_ned, weights)
        additional_data = {'quaternion': q_opt, 'weights': weights}
        
    elif method == 'SVD':
        vectors_body = np.array([gravity_body_norm, earth_rate_body_norm])
        vectors_ned = np.array([gravity_ned_norm, earth_rate_ned_norm])
        weights = np.array([1.0, 0.5])
        
        C_bn = svd_attitude(vectors_body, vectors_ned, weights)
        additional_data = {'weights': weights}
        
    else:
        raise ValueError(f"Unknown method: {method}")
    
    # Convert to Euler angles
    roll, pitch, yaw = dcm_to_euler(C_bn)
    
    # Validate rotation matrix
    det_C = np.linalg.det(C_bn)
    orthogonality_error = np.max(np.abs(C_bn @ C_bn.T - np.eye(3)))
    
    print(f"Initial attitude (Euler angles):")
    print(f"  Roll:  {np.degrees(roll):8.3f}°")
    print(f"  Pitch: {np.degrees(pitch):8.3f}°") 
    print(f"  Yaw:   {np.degrees(yaw):8.3f}°")
    print(f"Rotation matrix validation:")
    print(f"  Determinant: {det_C:.6f} (should be ~1.0)")
    print(f"  Orthogonality error: {orthogonality_error:.2e} (should be ~0)")
    
    # Compute refined sensor biases using determined attitude
    accel_bias_refined = gravity_body - C_bn.T @ gravity_ned
    gyro_bias_refined = earth_rate_body - C_bn.T @ earth_rate_ned
    
    print(f"Refined sensor biases:")
    print(f"  Accelerometer bias: {accel_bias_refined}")
    print(f"  Gyroscope bias: {gyro_bias_refined}")
    
    results = {
        'method': method,
        'C_bn': C_bn,
        'roll': roll,
        'pitch': pitch, 
        'yaw': yaw,
        'euler_degrees': np.degrees([roll, pitch, yaw]),
        'accel_bias': accel_bias_refined,
        'gyro_bias': gyro_bias_refined,
        'determinant': det_C,
        'orthogonality_error': orthogonality_error,
        **additional_data
    }
    
    return results
```

### Task 4: IMU-Only Integration and GNSS Comparison

**Objective**: Integrate IMU measurements to produce position, velocity, and attitude trajectories, then compare with GNSS solutions.

#### Dead-Reckoning Integration

```python
def imu_dead_reckoning(imu_data, initial_state, attitude_results):
    """
    Perform IMU-only dead-reckoning integration.
    
    Parameters:
    -----------
    imu_data : dict
        IMU measurements with 'times', 'accel', 'gyro'
    initial_state : dict
        Initial position, velocity, and attitude
    attitude_results : dict
        Results from Task 3 (biases, initial attitude)
        
    Returns:
    --------
    integration_results : dict
        Integrated trajectory (position, velocity, attitude vs time)
    """
    
    times = imu_data['times']
    accel_raw = imu_data['accel']
    gyro_raw = imu_data['gyro']
    n_samples = len(times)
    
    # Extract biases and initial attitude
    accel_bias = attitude_results['accel_bias']
    gyro_bias = attitude_results['gyro_bias']
    C_bn_initial = attitude_results['C_bn']
    
    # Initialize trajectory arrays
    positions_ned = np.zeros((n_samples, 3))
    velocities_ned = np.zeros((n_samples, 3))
    attitudes_dcm = np.zeros((n_samples, 3, 3))
    euler_angles = np.zeros((n_samples, 3))
    
    # Initial conditions
    positions_ned[0] = initial_state['position_ned']
    velocities_ned[0] = initial_state['velocity_ned']
    attitudes_dcm[0] = C_bn_initial
    euler_angles[0] = [attitude_results['roll'], attitude_results['pitch'], attitude_results['yaw']]
    
    # Gravity and scale factor
    gravity_ned = np.array([0, 0, 9.81])
    accel_scale_factor = 0.8368  # Typical scale factor from calibration
    
    print("Starting IMU integration...")
    
    for i in range(1, n_samples):
        dt = times[i] - times[i-1]
        
        if dt <= 0:
            # Handle time issues
            positions_ned[i] = positions_ned[i-1]
            velocities_ned[i] = velocities_ned[i-1]
            attitudes_dcm[i] = attitudes_dcm[i-1]
            euler_angles[i] = euler_angles[i-1]
            continue
        
        # Correct IMU measurements
        accel_corrected = (accel_raw[i] - accel_bias) * accel_scale_factor
        gyro_corrected = gyro_raw[i] - gyro_bias
        
        # Previous attitude
        C_bn_prev = attitudes_dcm[i-1]
        
        # Transform acceleration to navigation frame and remove gravity
        accel_nav = C_bn_prev @ accel_corrected - gravity_ned
        
        # Integrate velocity and position using trapezoidal rule
        accel_nav_avg = 0.5 * (accel_nav + (attitudes_dcm[i-1] @ (accel_raw[i-1] - accel_bias) * accel_scale_factor - gravity_ned))
        
        velocities_ned[i] = velocities_ned[i-1] + accel_nav_avg * dt
        positions_ned[i] = positions_ned[i-1] + 0.5 * (velocities_ned[i] + velocities_ned[i-1]) * dt
        
        # Attitude integration using small-angle approximation
        omega_nav = C_bn_prev @ gyro_corrected
        
        # Skew-symmetric matrix for attitude update
        Omega = skew_symmetric(omega_nav * dt)
        
        # Attitude update (first-order approximation)
        attitudes_dcm[i] = C_bn_prev @ (np.eye(3) + Omega)
        
        # Ensure orthonormality (Gram-Schmidt)
        U, _, Vt = np.linalg.svd(attitudes_dcm[i])
        attitudes_dcm[i] = U @ Vt
        
        # Extract Euler angles
        euler_angles[i] = dcm_to_euler(attitudes_dcm[i])
    
    print(f"IMU integration completed for {n_samples} samples")
    print(f"Final position (NED): {positions_ned[-1]}")
    print(f"Final velocity (NED): {velocities_ned[-1]}")
    print(f"Final attitude (deg): {np.degrees(euler_angles[-1])}")
    
    results = {
        'times': times,
        'positions_ned': positions_ned,
        'velocities_ned': velocities_ned,
        'attitudes_dcm': attitudes_dcm,
        'euler_angles': euler_angles,
        'euler_degrees': np.degrees(euler_angles)
    }
    
    return results

def compare_imu_gnss_trajectories(imu_results, gnss_data, reference_position):
    """
    Compare IMU-integrated trajectory with GNSS solution.
    
    Parameters:
    -----------
    imu_results : dict
        Results from IMU integration
    gnss_data : dict
        GNSS position and velocity data
    reference_position : array_like
        Reference ECEF position for NED conversion
        
    Returns:
    --------
    comparison_results : dict
        Comparison metrics and aligned trajectories
    """
    
    # Convert GNSS ECEF to NED for comparison
    gnss_positions_ecef = gnss_data['positions_ecef']
    gnss_velocities_ecef = gnss_data['velocities_ecef']
    gnss_times = gnss_data['times']
    
    # Convert reference position to geodetic
    ref_lat, ref_lon, ref_alt = ecef_to_geodetic(reference_position)
    
    # Convert GNSS positions to NED
    gnss_positions_ned = np.zeros_like(gnss_positions_ecef)
    for i, pos_ecef in enumerate(gnss_positions_ecef):
        pos_ned = ecef_to_ned(pos_ecef, ref_lat, ref_lon, ref_alt, reference_position)
        gnss_positions_ned[i] = pos_ned
    
    # Interpolate IMU results to GNSS time epochs
    from scipy.interpolate import interp1d
    
    imu_times = imu_results['times']
    imu_pos_ned = imu_results['positions_ned']
    
    # Find common time range
    t_start = max(imu_times[0], gnss_times[0])
    t_end = min(imu_times[-1], gnss_times[-1])
    
    # Interpolate IMU to GNSS epochs
    interp_func_pos = interp1d(imu_times, imu_pos_ned, axis=0, kind='linear', 
                              bounds_error=False, fill_value='extrapolate')
    
    gnss_mask = (gnss_times >= t_start) & (gnss_times <= t_end)
    gnss_times_common = gnss_times[gnss_mask]
    gnss_pos_ned_common = gnss_positions_ned[gnss_mask]
    
    imu_pos_ned_interp = interp_func_pos(gnss_times_common)
    
    # Compute position differences
    pos_diff = imu_pos_ned_interp - gnss_pos_ned_common
    pos_error_magnitude = np.linalg.norm(pos_diff, axis=1)
    
    # Statistics
    mean_error = np.mean(pos_error_magnitude)
    rms_error = np.sqrt(np.mean(pos_error_magnitude**2))
    max_error = np.max(pos_error_magnitude)
    
    print(f"IMU vs GNSS Position Comparison:")
    print(f"  Mean error: {mean_error:.3f} m")
    print(f"  RMS error:  {rms_error:.3f} m")
    print(f"  Max error:  {max_error:.3f} m")
    print(f"  Final error: {pos_error_magnitude[-1]:.3f} m")
    
    results = {
        'common_times': gnss_times_common,
        'imu_positions_ned': imu_pos_ned_interp,
        'gnss_positions_ned': gnss_pos_ned_common,
        'position_differences': pos_diff,
        'error_magnitudes': pos_error_magnitude,
        'mean_error': mean_error,
        'rms_error': rms_error,
        'max_error': max_error
    }
    
    return results
```

### Task 5: Extended Kalman Filter Fusion

**Objective**: Fuse IMU and GNSS data using an Extended Kalman Filter for optimal state estimation.

This builds on the EKF implementation from the Core Concepts section but provides complete integration:

```python
def run_complete_ekf_fusion(imu_data, gnss_data, initial_conditions, filter_params=None):
    """
    Complete EKF-based IMU/GNSS fusion pipeline.
    
    Parameters:
    -----------
    imu_data : dict
        Complete IMU dataset
    gnss_data : dict  
        Complete GNSS dataset
    initial_conditions : dict
        Initial state from previous tasks
    filter_params : dict, optional
        Kalman filter tuning parameters
        
    Returns:
    --------
    fusion_results : dict
        Complete fusion solution with statistics
    """
    
    # Default filter parameters
    if filter_params is None:
        filter_params = {
            'process_noise': {
                'position': 1e-8,
                'velocity': 1e-6,
                'attitude': 1e-8,
                'accel_bias': 1e-10,
                'gyro_bias': 1e-12
            },
            'measurement_noise': {
                'position': [1.0, 1.0, 2.0],
                'velocity': [0.1, 0.1, 0.2]
            }
        }
    
    # Initialize state vector [pos, vel, att_err, accel_bias, gyro_bias]
    initial_state = np.zeros(15)
    initial_state[:3] = initial_conditions['position_ned']
    initial_state[3:6] = initial_conditions['velocity_ned']
    initial_state[6:9] = [0, 0, 0]  # Small attitude errors
    initial_state[9:12] = initial_conditions['accel_bias']
    initial_state[12:15] = initial_conditions['gyro_bias']
    
    # Initial covariance matrix
    initial_P = np.diag([
        10.0, 10.0, 10.0,    # Position uncertainty [m²]
        1.0, 1.0, 1.0,       # Velocity uncertainty [m²/s²]
        0.1, 0.1, 0.1,       # Attitude uncertainty [rad²]
        0.01, 0.01, 0.01,    # Accel bias uncertainty [m²/s⁴]
        1e-6, 1e-6, 1e-6     # Gyro bias uncertainty [rad²/s²]
    ])
    
    # Process noise matrix
    Q = np.diag([
        filter_params['process_noise']['position']] * 3 +
        [filter_params['process_noise']['velocity']] * 3 +
        [filter_params['process_noise']['attitude']] * 3 +
        [filter_params['process_noise']['accel_bias']] * 3 +
        [filter_params['process_noise']['gyro_bias']] * 3
    )
    
    # Measurement noise matrix
    R = np.diag(
        filter_params['measurement_noise']['position'] +
        filter_params['measurement_noise']['velocity']
    )
    
    # Prepare synchronized data
    sync_data = synchronize_imu_gnss_data(imu_data, gnss_data)
    
    # Run EKF fusion
    fusion_results = run_ekf_fusion(
        sync_data, initial_state, initial_P, Q, R
    )
    
    # Post-process results
    fusion_results['filter_params'] = filter_params
    fusion_results['performance_metrics'] = analyze_filter_performance(fusion_results)
    
    return fusion_results

def analyze_filter_performance(fusion_results):
    """Analyze EKF performance and compute metrics."""
    
    residuals = fusion_results['residuals']
    states = fusion_results['states']
    covariances = fusion_results['covariances']
    
    if not residuals:
        return {'error': 'No GNSS updates available'}
    
    # Extract residual statistics
    pos_residuals = np.array([r['residual'][:3] for r in residuals])
    vel_residuals = np.array([r['residual'][3:6] for r in residuals])
    
    # Compute RMS errors
    pos_rms = np.sqrt(np.mean(pos_residuals**2, axis=0))
    vel_rms = np.sqrt(np.mean(vel_residuals**2, axis=0))
    
    # Innovation consistency (normalized innovation squared)
    innovation_consistency = []
    for r in residuals:
        residual = r['residual']
        S = r['innovation_cov']
        nis = residual.T @ np.linalg.inv(S) @ residual
        innovation_consistency.append(nis)
    
    # Analyze covariance trace evolution
    pos_uncertainty = np.array([np.sqrt(np.trace(P[:3, :3])) for P in covariances])
    vel_uncertainty = np.array([np.sqrt(np.trace(P[3:6, 3:6])) for P in covariances])
    att_uncertainty = np.array([np.sqrt(np.trace(P[6:9, 6:9])) for P in covariances])
    
    metrics = {
        'position_rms_errors': pos_rms,
        'velocity_rms_errors': vel_rms,
        'mean_innovation_consistency': np.mean(innovation_consistency),
        'final_position_uncertainty': pos_uncertainty[-1],
        'final_velocity_uncertainty': vel_uncertainty[-1],
        'final_attitude_uncertainty': att_uncertainty[-1],
        'convergence_metrics': {
            'position_convergence_95': np.where(pos_uncertainty < 0.05 * pos_uncertainty[0])[0],
            'velocity_convergence_95': np.where(vel_uncertainty < 0.05 * vel_uncertainty[0])[0]
        }
    }
    
    return metrics
```

This comprehensive tutorial section covers the essential tasks in detail, providing both theoretical understanding and practical implementation examples for both Python and MATLAB environments. The remaining tasks (6 and 7) follow similar patterns focusing on validation and analysis.
