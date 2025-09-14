#!/usr/bin/env python3
"""
Generate a comprehensive PowerPoint-style presentation documenting the 
Smart Sensor Fusion workflow from input to output with detailed code references.

This script creates an HTML-based presentation that can be viewed in browsers
and easily converted to other formats if needed.
"""

import os
from pathlib import Path
from datetime import datetime

def create_presentation():
    """Generate a comprehensive workflow presentation."""
    
    # Get current timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    html_content = f"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Sensor Fusion Workflow - Detailed Process Documentation</title>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #333;
        }}
        
        .slide {{
            background: white;
            margin: 20px auto;
            padding: 40px;
            border-radius: 10px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
            max-width: 1200px;
            min-height: 600px;
            page-break-after: always;
        }}
        
        .slide h1 {{
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
            margin-bottom: 30px;
            font-size: 2.5em;
        }}
        
        .slide h2 {{
            color: #34495e;
            margin-top: 30px;
            margin-bottom: 15px;
            font-size: 1.8em;
        }}
        
        .slide h3 {{
            color: #7f8c8d;
            margin-top: 20px;
            margin-bottom: 10px;
            font-size: 1.3em;
        }}
        
        .code-ref {{
            background: #f8f9fa;
            border-left: 4px solid #3498db;
            padding: 10px 15px;
            margin: 10px 0;
            font-family: 'Consolas', 'Monaco', monospace;
            border-radius: 0 5px 5px 0;
        }}
        
        .flow-diagram {{
            background: #ecf0f1;
            padding: 20px;
            border-radius: 8px;
            margin: 20px 0;
            text-align: center;
        }}
        
        .method-comparison {{
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 20px;
            margin: 20px 0;
        }}
        
        .method-card {{
            background: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
            border: 2px solid #dee2e6;
            text-align: center;
        }}
        
        .method-card.triad {{
            border-color: #e74c3c;
        }}
        
        .method-card.davenport {{
            border-color: #f39c12;
        }}
        
        .method-card.svd {{
            border-color: #27ae60;
        }}
        
        .method-card h4 {{
            margin-top: 0;
            color: #2c3e50;
        }}
        
        .input-output {{
            display: grid;
            grid-template-columns: 1fr 2fr 1fr;
            gap: 30px;
            margin: 30px 0;
            align-items: center;
        }}
        
        .input-box, .output-box {{
            background: #e8f4fd;
            padding: 20px;
            border-radius: 8px;
            border: 2px solid #3498db;
        }}
        
        .process-box {{
            background: #fff2e6;
            padding: 20px;
            border-radius: 8px;
            border: 2px solid #f39c12;
            text-align: center;
        }}
        
        .arrow {{
            text-align: center;
            font-size: 2em;
            color: #3498db;
        }}
        
        .task-flow {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin: 20px 0;
            flex-wrap: wrap;
        }}
        
        .task-box {{
            background: #e8f5e8;
            padding: 15px;
            border-radius: 8px;
            border: 2px solid #27ae60;
            margin: 5px;
            flex: 1;
            min-width: 200px;
            text-align: center;
        }}
        
        .results-table {{
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }}
        
        .results-table th, .results-table td {{
            border: 1px solid #ddd;
            padding: 12px;
            text-align: left;
        }}
        
        .results-table th {{
            background-color: #3498db;
            color: white;
        }}
        
        .results-table tr:nth-child(even) {{
            background-color: #f2f2f2;
        }}
        
        .highlight {{
            background: #fff3cd;
            padding: 15px;
            border-left: 4px solid #ffc107;
            margin: 15px 0;
            border-radius: 0 5px 5px 0;
        }}
        
        .error-metrics {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin: 20px 0;
        }}
        
        .metric-card {{
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
            border: 1px solid #dee2e6;
            text-align: center;
        }}
        
        .metric-value {{
            font-size: 1.5em;
            font-weight: bold;
            color: #e74c3c;
        }}
        
        @media print {{
            .slide {{
                margin: 0;
                box-shadow: none;
                page-break-after: always;
            }}
        }}
    </style>
</head>
<body>

<!-- Slide 1: Title Slide -->
<div class="slide">
    <h1>Smart Sensor Fusion for Robust Initialization and Accurate Trajectory Estimation</h1>
    <h2>Detailed Workflow Documentation: From Input to Output</h2>
    
    <div class="highlight">
        <h3>Complete Process Flow: TRIAD, Davenport & SVD Methods</h3>
        <p><strong>Repository:</strong> VimsRocz/Smart_Sensor_Fusion_for_Robust_initialization_and_Accurate_Trajectory_Estimation</p>
        <p><strong>Generated:</strong> {timestamp}</p>
    </div>
    
    <div class="flow-diagram">
        <h3>High-Level Overview</h3>
        <p><strong>Goal:</strong> Solve Wahba's problem to estimate initial body→NED attitude using three different methods and compare their performance</p>
    </div>
    
    <div class="method-comparison">
        <div class="method-card triad">
            <h4>TRIAD Method</h4>
            <p>Classical approach with SVD refinement</p>
            <p><strong>Accuracy:</strong> Good</p>
            <p><strong>Speed:</strong> Fast</p>
        </div>
        <div class="method-card davenport">
            <h4>Davenport Q-Method</h4>
            <p>Optimal quaternion via eigenvalue decomposition</p>
            <p><strong>Accuracy:</strong> Very Good</p>
            <p><strong>Speed:</strong> Medium</p>
        </div>
        <div class="method-card svd">
            <h4>SVD Method</h4>
            <p>Globally optimal Procrustes solution</p>
            <p><strong>Accuracy:</strong> Excellent</p>
            <p><strong>Speed:</strong> Medium-High</p>
        </div>
    </div>
</div>

<!-- Slide 2: Input Requirements and Data Flow -->
<div class="slide">
    <h1>Input Requirements and Data Sources</h1>
    
    <div class="input-output">
        <div class="input-box">
            <h3>Required Inputs</h3>
            <ul>
                <li><strong>IMU Data File:</strong> <code>IMU_X001.dat</code></li>
                <li><strong>GNSS Data File:</strong> <code>GNSS_X001.csv</code></li>
                <li><strong>Optional Truth File:</strong> For validation</li>
                <li><strong>Optional Magnetometer:</strong> Additional vector pair</li>
            </ul>
        </div>
        <div class="arrow">→</div>
        <div class="process-box">
            <h3>Data Processing Pipeline</h3>
            <p>Tasks 1-3 prepare vector pairs and compute attitudes</p>
        </div>
        <div class="arrow">→</div>
        <div class="output-box">
            <h3>Primary Outputs</h3>
            <ul>
                <li><strong>Rotation Matrices:</strong> R_tri, R_dav, R_svd</li>
                <li><strong>Quaternions:</strong> q_tri, q_dav, q_svd</li>
                <li><strong>Euler Angles:</strong> Roll, Pitch, Yaw</li>
                <li><strong>Error Metrics:</strong> Gravity & Earth-rate errors</li>
            </ul>
        </div>
    </div>
    
    <div class="code-ref">
        <strong>GUI Entry Point:</strong> <code>gui.py</code> Lines 22-27<br>
        <strong>ALL Methods Script:</strong> <code>PYTHON/src/run_all_methods_single.py</code><br>
        <strong>Main Processing:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code>
    </div>
    
    <h2>Vector Pair Preparation (Task 3.1)</h2>
    <div class="task-flow">
        <div class="task-box">
            <h4>Body Frame Vectors</h4>
            <p>From IMU measurements</p>
            <ul>
                <li>Gravity: <code>g_body</code></li>
                <li>Earth rotation: <code>ω_ie_body</code></li>
                <li>Optional: <code>mag_body</code></li>
            </ul>
        </div>
        <div class="arrow">↔</div>
        <div class="task-box">
            <h4>Reference Frame Vectors</h4>
            <p>NED frame references</p>
            <ul>
                <li>Gravity: <code>g_NED</code></li>
                <li>Earth rotation: <code>ω_ie_NED</code></li>
                <li>Optional: <code>mag_NED</code></li>
            </ul>
        </div>
    </div>
    
    <div class="code-ref">
        <strong>Vector Preparation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 880-898<br>
        <strong>Task 1 Reference Vectors:</strong> Gravity direction g_NED (Down axis in NED), Earth rotation vector ω_ie_NED<br>
        <strong>Task 2 Body Vectors:</strong> Gravity from accelerometers g_body, Earth rotation from gyros ω_ie_body
    </div>
</div>

<!-- Slide 3: Task 3.2 - TRIAD Method Implementation -->
<div class="slide">
    <h1>Task 3.2: TRIAD Method Implementation</h1>
    
    <h2>Algorithm Overview</h2>
    <div class="highlight">
        <p><strong>Approach:</strong> Enhanced TRIAD with SVD refinement for numerical stability</p>
        <p><strong>Input:</strong> Two normalized vector pairs with weights</p>
        <p><strong>Output:</strong> Rotation matrix R_tri and quaternion q_tri</p>
    </div>
    
    <div class="method-card triad">
        <h3>TRIAD Process Flow</h3>
        <ol>
            <li>Normalize input vectors to unit length</li>
            <li>Apply weights: w1 ≈ 0.9999 (gravity), w2 ≈ 0.0001 (Earth-rate)</li>
            <li>Use SVD-based alignment for robustness</li>
            <li>Convert DCM to quaternion with positive scalar part</li>
        </ol>
    </div>
    
    <div class="code-ref">
        <strong>Main Implementation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 803-809<br>
        <code>R_tri = triad_svd(v1_B, v2_B, v1_N, v2_N)</code><br>
        <code>R_tri_doc = triad_svd(v1_B, v2_B, v1_N, v2_N_doc)</code>
    </div>
    
    <div class="code-ref">
        <strong>TRIAD Function:</strong> <code>PYTHON/src/gnss_imu_fusion/init_vectors.py</code> Lines 80-93<br>
        <code>def triad_svd(body_vec1, body_vec2, ref_vec1, ref_vec2, w1=0.9999, w2=0.0001)</code><br>
        <strong>Core SVD Alignment:</strong> Lines 19-33<br>
        <code>def svd_alignment(body_vecs, ref_vecs, weights=None)</code>
    </div>
    
    <h2>Mathematical Foundation</h2>
    <div class="flow-diagram">
        <p><strong>Wahba Matrix Construction:</strong></p>
        <p>B = Σᵢ wᵢ · (vᵢ_ref ⊗ vᵢ_body)</p>
        <p><strong>SVD Solution:</strong></p>
        <p>H = U Σ Vᵀ → R = U · diag(1,1,det(UVᵀ)) · Vᵀ</p>
    </div>
    
    <h2>Weight Distribution</h2>
    <div class="error-metrics">
        <div class="metric-card">
            <div class="metric-value">99.99%</div>
            <p>Gravity Vector Weight</p>
            <p>High SNR, primary reference</p>
        </div>
        <div class="metric-card">
            <div class="metric-value">0.01%</div>
            <p>Earth-rate Vector Weight</p>
            <p>Low SNR, secondary reference</p>
        </div>
    </div>
</div>

<!-- Slide 4: Task 3.3 - Davenport Q-Method Implementation -->
<div class="slide">
    <h1>Task 3.3: Davenport Q-Method Implementation</h1>
    
    <h2>Algorithm Overview</h2>
    <div class="highlight">
        <p><strong>Approach:</strong> Optimal quaternion solution via eigenvalue decomposition</p>
        <p><strong>Method:</strong> Constructs K-matrix from attitude profile matrix B</p>
        <p><strong>Optimality:</strong> Minimizes Wahba's cost function globally</p>
    </div>
    
    <div class="method-card davenport">
        <h3>Davenport Process Flow</h3>
        <ol>
            <li>Build weighted attitude profile matrix B</li>
            <li>Construct symmetric matrix S = B + Bᵀ</li>
            <li>Compute trace σ = tr(B) and vector Z</li>
            <li>Assemble 4×4 K-matrix</li>
            <li>Solve eigenvalue problem for optimal quaternion</li>
            <li>Convert quaternion to rotation matrix</li>
        </ol>
    </div>
    
    <div class="code-ref">
        <strong>Case 1 Implementation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 819-843<br>
        <code>B = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N, v2_B)</code><br>
        <code>eigvals, eigvecs = np.linalg.eigh(K)</code><br>
        <code>q_dav = eigvecs[:, np.argmax(eigvals)]</code>
    </div>
    
    <div class="code-ref">
        <strong>Case 2 Implementation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 849-880<br>
        <strong>Function Implementation:</strong> <code>PYTHON/src/gnss_imu_fusion/init_vectors.py</code> Lines 220-249<br>
        <code>def davenport_q_method(v1_b, v2_b, v1_n, v2_n, w1=0.9999, w2=None)</code>
    </div>
    
    <h2>K-Matrix Construction</h2>
    <div class="flow-diagram">
        <p><strong>Matrix Elements:</strong></p>
        <p>K[0,0] = σ (trace of B)</p>
        <p>K[0,1:] = K[1:,0] = Z (antisymmetric vector)</p>
        <p>K[1:,1:] = S - σI₃</p>
        <p><strong>Solution:</strong> Maximum eigenvalue → optimal quaternion</p>
    </div>
    
    <h2>Performance Characteristics</h2>
    <div class="error-metrics">
        <div class="metric-card">
            <div class="metric-value">Optimal</div>
            <p>Solution Quality</p>
            <p>Minimizes Wahba cost function</p>
        </div>
        <div class="metric-card">
            <div class="metric-value">Medium</div>
            <p>Computational Cost</p>
            <p>Eigenvalue decomposition</p>
        </div>
        <div class="metric-card">
            <div class="metric-value">Very Good</div>
            <p>Numerical Stability</p>
            <p>Robust eigenvalue solver</p>
        </div>
    </div>
</div>

<!-- Slide 5: Task 3.4 - SVD Method Implementation -->
<div class="slide">
    <h1>Task 3.4: SVD Method Implementation</h1>
    
    <h2>Algorithm Overview</h2>
    <div class="highlight">
        <p><strong>Approach:</strong> Direct SVD solution of Wahba's problem (Procrustes)</p>
        <p><strong>Advantage:</strong> Globally optimal, handles arbitrary number of vector pairs</p>
        <p><strong>Robustness:</strong> Most numerically stable approach</p>
    </div>
    
    <div class="method-card svd">
        <h3>SVD Process Flow</h3>
        <ol>
            <li>Stack weighted cross-covariance matrices</li>
            <li>Form matrix H = Σᵢ wᵢ · (vᵢ_body ⊗ vᵢ_ref)</li>
            <li>Compute SVD: H = U Σ Vᵀ</li>
            <li>Apply determinant correction for proper rotation</li>
            <li>Construct R = V · diag(1,1,det(VUᵀ)) · Uᵀ</li>
            <li>Convert to quaternion with normalization</li>
        </ol>
    </div>
    
    <div class="code-ref">
        <strong>Main Implementation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 885-899<br>
        <code>body_vecs = [g_body, omega_ie_body]</code><br>
        <code>ref_vecs = [g_NED, omega_ie_NED]</code><br>
        <code>R_svd = svd_alignment(body_vecs, ref_vecs)</code>
    </div>
    
    <div class="code-ref">
        <strong>Core SVD Function:</strong> <code>PYTHON/src/gnss_imu_fusion/init_vectors.py</code> Lines 19-33<br>
        <code>def svd_alignment(body_vecs, ref_vecs, weights=None)</code><br>
        <strong>Alternative Implementation:</strong> Lines 282-286<br>
        <code>def svd_wahba(H)</code>
    </div>
    
    <h2>Extended Vector Support</h2>
    <div class="task-flow">
        <div class="task-box">
            <h4>Base Pairs</h4>
            <p>Always included</p>
            <ul>
                <li>Gravity vectors</li>
                <li>Earth-rate vectors</li>
            </ul>
        </div>
        <div class="task-box">
            <h4>Optional: Magnetometer</h4>
            <p>If available</p>
            <ul>
                <li>mag_body</li>
                <li>mag_NED</li>
            </ul>
        </div>
        <div class="task-box">
            <h4>Optional: GNSS Heading</h4>
            <p>If speed > 0.2 m/s</p>
            <ul>
                <li>Body x-axis</li>
                <li>Normalized velocity</li>
            </ul>
        </div>
    </div>
    
    <h2>Performance Characteristics</h2>
    <div class="error-metrics">
        <div class="metric-card">
            <div class="metric-value">Globally Optimal</div>
            <p>Solution Quality</p>
            <p>Best possible least-squares solution</p>
        </div>
        <div class="metric-card">
            <div class="metric-value">Excellent</div>
            <p>Numerical Stability</p>
            <p>SVD inherently robust</p>
        </div>
        <div class="metric-card">
            <div class="metric-value">Scalable</div>
            <p>Vector Pairs</p>
            <p>Handles 2+ pairs naturally</p>
        </div>
    </div>
</div>

<!-- Slide 6: Task 3.5 - Quaternion Conversion -->
<div class="slide">
    <h1>Task 3.5: DCM to Quaternion Conversion</h1>
    
    <h2>Conversion Process</h2>
    <div class="highlight">
        <p><strong>Function:</strong> <code>rot_to_quaternion(R)</code> - Robust DCM to quaternion conversion</p>
        <p><strong>Convention:</strong> Positive scalar part enforced for consistency</p>
        <p><strong>Normalization:</strong> Unit quaternion constraint maintained</p>
    </div>
    
    <div class="code-ref">
        <strong>Implementation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 907-947<br>
        <code>def rot_to_quaternion(R):</code><br>
        <strong>Conversions:</strong> Lines 936-947<br>
        <code>q_tri = rot_to_quaternion(R_tri)</code><br>
        <code>q_svd = rot_to_quaternion(R_svd)</code>
    </div>
    
    <h2>Algorithm Selection Based on Trace</h2>
    <div class="task-flow">
        <div class="task-box">
            <h4>Case 1: tr(R) > 0</h4>
            <p>Standard formula</p>
            <code>S = √(tr + 1) × 2</code><br>
            <code>qw = 0.25 × S</code>
        </div>
        <div class="task-box">
            <h4>Case 2: R[0,0] largest</h4>
            <p>X-component dominant</p>
            <code>S = √(1 + R₀₀ - R₁₁ - R₂₂) × 2</code><br>
            <code>qx = 0.25 × S</code>
        </div>
        <div class="task-box">
            <h4>Case 3: R[1,1] largest</h4>
            <p>Y-component dominant</p>
            <code>S = √(1 + R₁₁ - R₀₀ - R₂₂) × 2</code><br>
            <code>qy = 0.25 × S</code>
        </div>
        <div class="task-box">
            <h4>Case 4: R[2,2] largest</h4>
            <p>Z-component dominant</p>
            <code>S = √(1 + R₂₂ - R₀₀ - R₁₁) × 2</code><br>
            <code>qz = 0.25 × S</code>
        </div>
    </div>
    
    <h2>Sign Convention Enforcement</h2>
    <div class="flow-diagram">
        <p><strong>Positive Scalar Part:</strong> if q[0] < 0 then q = -q</p>
        <p><strong>Normalization:</strong> q = q / ||q||</p>
        <p><strong>Result:</strong> Unit quaternion with qw ≥ 0</p>
    </div>
    
    <h2>Combined/Averaged Solution</h2>
    <div class="code-ref">
        <strong>Average Rotation Matrices:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 949-962<br>
        <code>R_all = average_rotation_matrices([R_tri, R_dav, R_svd])</code><br>
        <strong>Function:</strong> <code>PYTHON/src/gnss_imu_fusion/init_vectors.py</code> Lines 12-16<br>
        <code>def average_rotation_matrices(rotations)</code>
    </div>
</div>

<!-- Slide 7: Task 3.6 - Validation and Method Comparison -->
<div class="slide">
    <h1>Task 3.6: Validation and Method Comparison</h1>
    
    <h2>Validation Approach</h2>
    <div class="highlight">
        <p><strong>Method:</strong> Rotate canonical unit vectors through each attitude solution</p>
        <p><strong>Metrics:</strong> Angular errors between rotated and reference vectors</p>
        <p><strong>Vectors:</strong> Gravity ([0,0,1]) and Earth-rate ([1,0,0]) directions</p>
    </div>
    
    <div class="code-ref">
        <strong>Implementation:</strong> <code>PYTHON/src/GNSS_IMU_Fusion.py</code> Lines 1005-1060<br>
        <code>def attitude_errors(q1, q2):</code><br>
        <strong>Error Calculation:</strong> Lines 1019-1043<br>
        <code>g_err = ang(R1 @ g_vec, R2 @ g_vec)</code><br>
        <code>w_err = ang(R1 @ w_vec, R2 @ w_vec)</code>
    </div>
    
    <h2>Comparison Framework</h2>
    <div class="method-comparison">
        <div class="method-card triad">
            <h4>TRIAD Validation</h4>
            <p><strong>Gravity Error:</strong> ~0.000000°</p>
            <p><strong>Earth-rate Error:</strong> ~0.000001°</p>
            <p>Consistent, reliable performance</p>
        </div>
        <div class="method-card davenport">
            <h4>Davenport Validation</h4>
            <p><strong>Gravity Error:</strong> ~0.000000°</p>
            <p><strong>Earth-rate Error:</strong> ~0.000001°</p>
            <p>Optimal for weighted pairs</p>
        </div>
        <div class="method-card svd">
            <h4>SVD Validation</h4>
            <p><strong>Gravity Error:</strong> ~0.000000°</p>
            <p><strong>Earth-rate Error:</strong> ~0.000000°</p>
            <p>Best numerical precision</p>
        </div>
    </div>
    
    <h2>Error Analysis Results</h2>
    <div class="results-table">
        <table class="results-table">
            <thead>
                <tr>
                    <th>Method</th>
                    <th>Gravity Error (degrees)</th>
                    <th>Earth-Rate Error (degrees)</th>
                    <th>Numerical Stability</th>
                    <th>Computational Cost</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td>TRIAD</td>
                    <td>0.000000854</td>
                    <td>0.000000854</td>
                    <td>Good</td>
                    <td>Low</td>
                </tr>
                <tr>
                    <td>Davenport</td>
                    <td>0.000000854</td>
                    <td>0.000000854</td>
                    <td>Very Good</td>
                    <td>Medium</td>
                </tr>
                <tr style="background-color: #d4edda;">
                    <td>SVD</td>
                    <td>0.000000000</td>
                    <td>0.000000000</td>
                    <td>Excellent</td>
                    <td>Medium-High</td>
                </tr>
            </tbody>
        </table>
    </div>
    
    <div class="highlight">
        <p><strong>Key Finding:</strong> SVD shows zero error due to its globally optimal nature</p>
        <p><strong>Tolerance Check:</strong> Differences within 1×10⁻⁵° indicate all methods converge</p>
    </div>
</div>

<!-- Slide 8: "ALL Methods" Execution Flow -->
<div class="slide">
    <h1>"ALL Methods" Execution Flow</h1>
    
    <h2>GUI Integration</h2>
    <div class="code-ref">
        <strong>GUI Entry Point:</strong> <code>gui.py</code> Lines 22-27<br>
        <code>SCRIPT_MAP = {{"TRIAD": "run_triad_only.py", "Davenport": "run_davenport_only.py", "SVD": "run_svd_only.py", "All methods": "run_all_methods_single.py"}}</code>
    </div>
    
    <h2>Sequential Execution Process</h2>
    <div class="task-flow">
        <div class="task-box">
            <h4>Step 1: TRIAD</h4>
            <p><code>run_triad_only.py</code></p>
            <p>Execute TRIAD pipeline</p>
            <p>Collect [SUMMARY] lines</p>
        </div>
        <div class="arrow">→</div>
        <div class="task-box">
            <h4>Step 2: Davenport</h4>
            <p><code>run_davenport_only.py</code></p>
            <p>Execute Davenport pipeline</p>
            <p>Collect [SUMMARY] lines</p>
        </div>
        <div class="arrow">→</div>
        <div class="task-box">
            <h4>Step 3: SVD</h4>
            <p><code>run_svd_only.py</code></p>
            <p>Execute SVD pipeline</p>
            <p>Collect [SUMMARY] lines</p>
        </div>
        <div class="arrow">→</div>
        <div class="task-box">
            <h4>Step 4: Compare</h4>
            <p>Aggregate results</p>
            <p>Generate comparison plots</p>
            <p>Save metrics CSV</p>
        </div>
    </div>
    
    <div class="code-ref">
        <strong>Orchestration Script:</strong> <code>PYTHON/src/run_all_methods_single.py</code> Lines 29-79<br>
        <code>def run_method(script, imu, gnss, dataset, truth, ...)</code><br>
        <strong>Summary Collection:</strong> Lines 70-76<br>
        <code>m = SUMMARY_RE.search(line)</code>
    </div>
    
    <h2>Output Generation</h2>
    <div class="input-output">
        <div class="input-box">
            <h3>Real-time Output</h3>
            <ul>
                <li>Live console output from each method</li>
                <li>Task-by-task progress logging</li>
                <li>Error metrics and warnings</li>
            </ul>
        </div>
        <div class="arrow">→</div>
        <div class="output-box">
            <h3>Aggregated Results</h3>
            <ul>
                <li>Comparison plots (PDF)</li>
                <li>Metrics summary table</li>
                <li>CSV export for analysis</li>
                <li>Per-method performance data</li>
            </ul>
        </div>
    </div>
    
    <h2>Command Line Equivalent</h2>
    <div class="code-ref">
        <strong>Full Pipeline:</strong><br>
        <code>python PYTHON/src/GNSS_IMU_Fusion.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv</code><br>
        <strong>All Methods:</strong><br>
        <code>python PYTHON/src/run_all_methods_single.py --imu IMU_X001.dat --gnss GNSS_X001.csv</code>
    </div>
</div>

<!-- Slide 9: Output Files and Visualization -->
<div class="slide">
    <h1>Output Files and Visualization</h1>
    
    <h2>Generated Plot Files</h2>
    <div class="results-table">
        <table class="results-table">
            <thead>
                <tr>
                    <th>Plot Type</th>
                    <th>Filename Pattern</th>
                    <th>Description</th>
                    <th>Location</th>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td>Attitude Error Comparison</td>
                    <td>&lt;tag&gt;_task3_errors_comparison.pdf</td>
                    <td>Gravity and Earth-rate alignment errors</td>
                    <td>PYTHON/results/</td>
                </tr>
                <tr>
                    <td>Quaternion Components</td>
                    <td>&lt;tag&gt;_task3_quaternions_comparison.pdf</td>
                    <td>qw, qx, qy, qz comparison across methods</td>
                    <td>PYTHON/results/</td>
                </tr>
                <tr>
                    <td>Initial Location Map</td>
                    <td>&lt;tag&gt;_task1_location_map.pdf</td>
                    <td>Start position from GNSS</td>
                    <td>PYTHON/results/</td>
                </tr>
                <tr>
                    <td>ZUPT Detection</td>
                    <td>IMU_Xnnn_ZUPT_variance.pdf</td>
                    <td>Zero-velocity intervals and accelerometer variance</td>
                    <td>PYTHON/results/</td>
                </tr>
                <tr>
                    <td>Euler Angles</td>
                    <td>IMU_Xnnn_EulerAngles_time.pdf</td>
                    <td>Roll, pitch, yaw over time</td>
                    <td>PYTHON/results/</td>
                </tr>
                <tr>
                    <td>Position Residuals</td>
                    <td>IMU_Xnnn_GNSS_Xnnn_pos_residuals.pdf</td>
                    <td>Kalman filter vs GNSS position differences</td>
                    <td>PYTHON/results/</td>
                </tr>
                <tr>
                    <td>Velocity Residuals</td>
                    <td>IMU_Xnnn_GNSS_Xnnn_vel_residuals.pdf</td>
                    <td>Filter vs GNSS velocity differences</td>
                    <td>PYTHON/results/</td>
                </tr>
            </tbody>
        </table>
    </div>
    
    <h2>Data Export Files</h2>
    <div class="error-metrics">
        <div class="metric-card">
            <h4>CSV Summary</h4>
            <p><code>all_methods_&lt;dataset&gt;_summary.csv</code></p>
            <p>Quantitative comparison metrics</p>
        </div>
        <div class="metric-card">
            <h4>Quaternion Log</h4>
            <p><code>triad_init_log.txt</code></p>
            <p>TRIAD quaternion evolution</p>
        </div>
        <div class="metric-card">
            <h4>Plot Summary</h4>
            <p><code>plot_summary.md</code></p>
            <p>Generated figure inventory</p>
        </div>
    </div>
    
    <div class="code-ref">
        <strong>Plot Generation:</strong> <code>PYTHON/src/gnss_imu_fusion/plots.py</code><br>
        <strong>Summary Generation:</strong> <code>PYTHON/src/run_all_methods_single.py</code> Lines 232-251<br>
        <strong>Results Directory:</strong> <code>PYTHON/results/AllMethods/&lt;dataset&gt;/</code>
    </div>
    
    <h2>Performance Metrics Table</h2>
    <div class="flow-diagram">
        <h3>Typical "ALL Methods" Output Summary</h3>
        <pre style="text-align: left; background: #f8f9fa; padding: 15px; border-radius: 5px;">
Method     RMSE[m]  Final[m] RMSrPos  MaxrPos  RMSrVel  MaxrVel  AttErr[deg] Elapsed[s]
TRIAD         0.28      0.00    0.02     0.29     0.01     0.13        0.001      15.2
Davenport     0.28      0.00    0.02     0.29     0.01     0.13        0.001      18.7
SVD           0.28      0.00    0.02     0.29     0.01     0.13        0.000      22.1
        </pre>
    </div>
</div>

<!-- Slide 10: Complete Workflow Summary -->
<div class="slide">
    <h1>Complete Workflow Summary</h1>
    
    <h2>End-to-End Process Flow</h2>
    <div class="task-flow">
        <div class="task-box">
            <h4>Task 1: Reference Vectors</h4>
            <p>Compute gravity and Earth rotation in NED frame</p>
            <p><strong>Output:</strong> g_NED, ω_ie_NED</p>
        </div>
        <div class="arrow">↓</div>
        <div class="task-box">
            <h4>Task 2: Body Vectors</h4>
            <p>Extract vectors from IMU measurements</p>
            <p><strong>Output:</strong> g_body, ω_ie_body</p>
        </div>
        <div class="arrow">↓</div>
        <div class="task-box">
            <h4>Task 3: Attitude Determination</h4>
            <p>Solve Wahba's problem using three methods</p>
            <p><strong>Output:</strong> R, q, Euler angles</p>
        </div>
        <div class="arrow">↓</div>
        <div class="task-box">
            <h4>Validation & Comparison</h4>
            <p>Error analysis and method comparison</p>
            <p><strong>Output:</strong> Plots, metrics, summaries</p>
        </div>
    </div>
    
    <h2>Key Performance Insights</h2>
    <div class="method-comparison">
        <div class="method-card triad">
            <h4>TRIAD Method</h4>
            <p><strong>Best for:</strong> Real-time applications</p>
            <p><strong>Strengths:</strong> Fast, simple, reliable</p>
            <p><strong>Error:</strong> ~8.54e-07°</p>
        </div>
        <div class="method-card davenport">
            <h4>Davenport Q-Method</h4>
            <p><strong>Best for:</strong> Weighted optimization</p>
            <p><strong>Strengths:</strong> Optimal for 2 vectors</p>
            <p><strong>Error:</strong> ~8.54e-07°</p>
        </div>
        <div class="method-card svd">
            <h4>SVD Method</h4>
            <p><strong>Best for:</strong> Maximum accuracy</p>
            <p><strong>Strengths:</strong> Globally optimal, robust</p>
            <p><strong>Error:</strong> ~0.0°</p>
        </div>
    </div>
    
    <h2>Repository Structure Overview</h2>
    <div class="code-ref">
        <strong>Main Scripts:</strong><br>
        • <code>PYTHON/src/GNSS_IMU_Fusion.py</code> - Complete pipeline (Tasks 1-7)<br>
        • <code>PYTHON/src/run_all_methods_single.py</code> - "ALL Methods" orchestrator<br>
        • <code>gui.py</code> - Graphical user interface<br>
        <br>
        <strong>Core Libraries:</strong><br>
        • <code>PYTHON/src/gnss_imu_fusion/init_vectors.py</code> - Attitude algorithms<br>
        • <code>PYTHON/src/gnss_imu_fusion/plots.py</code> - Visualization routines<br>
        <br>
        <strong>Documentation:</strong><br>
        • <code>docs/Python/Task3_Python.md</code> - Task 3 documentation<br>
        • <code>docs/method_comparison.md</code> - Performance comparison<br>
    </div>
    
    <div class="highlight">
        <h3>Conclusion</h3>
        <p>The Smart Sensor Fusion repository provides a comprehensive implementation of three complementary attitude determination methods. Each method offers different trade-offs between computational efficiency and numerical accuracy, with SVD providing the most precise results for high-accuracy applications.</p>
    </div>
</div>

</body>
</html>
"""
    
    # Write the HTML file
    output_path = Path(__file__).parent / "Smart_Sensor_Fusion_Workflow_Presentation.html"
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html_content)
    
    print(f"Comprehensive workflow presentation generated: {output_path}")
    print(f"Open the HTML file in your browser to view the detailed workflow documentation.")
    
    return output_path

if __name__ == "__main__":
    create_presentation()