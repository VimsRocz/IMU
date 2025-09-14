# Smart Sensor Fusion Workflow Presentation

## Overview

This comprehensive presentation documents the detailed workflow of the Smart Sensor Fusion repository, showcasing how the three attitude determination methods (TRIAD, Davenport, and SVD) process inputs and generate outputs.

## Generated Files

1. **`Smart_Sensor_Fusion_Workflow_Presentation.html`** - The main presentation file
2. **`generate_workflow_presentation.py`** - Script to generate the presentation
3. **`test_workflow_validation.py`** - Validation script for testing the workflow

## Presentation Contents

### 10 Comprehensive Slides:

1. **Title Slide** - Overview of the three methods and their characteristics
2. **Input Requirements** - Data sources, required files, and vector preparation
3. **Task 3.2: TRIAD Method** - Implementation details with code references
4. **Task 3.3: Davenport Q-Method** - K-matrix construction and eigenvalue solution
5. **Task 3.4: SVD Method** - Globally optimal Procrustes solution
6. **Task 3.5: Quaternion Conversion** - DCM to quaternion transformation
7. **Task 3.6: Validation & Comparison** - Error analysis and method comparison
8. **"ALL Methods" Execution Flow** - GUI integration and sequential processing
9. **Output Files & Visualization** - Generated plots and data exports
10. **Complete Workflow Summary** - End-to-end process and conclusions

## Key Features

### Detailed Code References
Each slide includes specific file paths and line numbers where implementations can be found:
- `PYTHON/src/GNSS_IMU_Fusion.py` (Lines 803-1060)
- `PYTHON/src/gnss_imu_fusion/init_vectors.py` (Lines 12-286)
- `PYTHON/src/run_all_methods_single.py` (Complete orchestration)
- `gui.py` (GUI integration)

### Method Comparison
- **TRIAD**: Fast, reliable, ~8.54e-07° error
- **Davenport**: Optimal for weighted pairs, ~8.54e-07° error  
- **SVD**: Globally optimal, ~0.0° error

### Comprehensive Input/Output Documentation
- Input files: IMU_X001.dat, GNSS_X001.csv, optional truth files
- Vector pairs: gravity and Earth-rate in body/NED frames
- Outputs: rotation matrices, quaternions, Euler angles, error metrics
- Generated plots: attitude errors, quaternion comparisons, residuals

## How to View

1. **Web Browser**: Open `Smart_Sensor_Fusion_Workflow_Presentation.html` in any modern web browser
2. **Print/PDF**: Use browser's print functionality to generate PDF version
3. **Mobile-Friendly**: Responsive design works on all screen sizes

## Validation

Run the validation script to verify the workflow:
```bash
python test_workflow_validation.py
```

This checks:
- Key file existence
- Core algorithm imports
- Sample data availability
- Script functionality

## Technical Implementation

The presentation is built using:
- **HTML5** with semantic structure
- **CSS3** with modern styling (gradients, grid layouts, responsive design)
- **Print styles** for PDF generation
- **Accessibility features** for screen readers

## Key Highlights

1. **Mathematical Foundation**: Wahba's problem solution using three complementary approaches
2. **Code Traceability**: Every algorithm step mapped to specific code locations
3. **Performance Metrics**: Quantitative comparison with real error measurements
4. **Practical Usage**: Command-line and GUI execution examples
5. **Output Examples**: Complete documentation of generated files and plots

This presentation serves as a comprehensive reference for understanding the complete workflow from raw sensor data to validated attitude estimates using state-of-the-art sensor fusion techniques.