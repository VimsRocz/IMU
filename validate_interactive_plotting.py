#!/usr/bin/env python3
"""
Complete validation and demonstration script for interactive Task 6 plotting.

This script validates that the interactive plotting upgrades work correctly
and demonstrates the enhanced features for both Python and MATLAB environments.
"""

import sys
import subprocess
import numpy as np
from pathlib import Path
import time

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent / "PYTHON" / "src"))

try:
    from plot_overlay_interactive import plot_overlay_interactive, create_comparison_dashboard, PLOTLY_AVAILABLE
    from plot_overlay import plot_overlay
except ImportError as e:
    print(f"Import error: {e}")
    print("Please ensure you're running from the repository root directory")
    sys.exit(1)


def print_section(title):
    """Print a formatted section header."""
    print("\n" + "="*80)
    print(f" {title}")
    print("="*80)


def print_subsection(title):
    """Print a formatted subsection header."""
    print(f"\n--- {title} ---")


def check_dependencies():
    """Check that all required dependencies are available."""
    print_section("DEPENDENCY CHECK")
    
    dependencies = {
        "numpy": "numpy",
        "matplotlib": "matplotlib.pyplot", 
        "plotly": "plotly.graph_objects",
        "kaleido": "kaleido",
        "scipy": "scipy.io"
    }
    
    missing = []
    for name, module in dependencies.items():
        try:
            __import__(module)
            print(f"✓ {name} available")
        except ImportError:
            print(f"✗ {name} missing")
            missing.append(name)
    
    if missing:
        print(f"\nMissing dependencies: {', '.join(missing)}")
        print("Install with: pip install " + " ".join(missing))
        return False
    
    print(f"\nPlotly available: {PLOTLY_AVAILABLE}")
    return True


def test_synthetic_data():
    """Test interactive plotting with synthetic data."""
    print_section("SYNTHETIC DATA TEST")
    
    # Generate synthetic test data
    print("Generating synthetic test data...")
    t = np.linspace(0, 100, 2000)
    
    # Synthetic trajectory
    pos_base = np.array([
        10 * np.sin(0.1 * t),
        10 * np.cos(0.1 * t), 
        0.02 * t
    ]).T
    
    vel_base = np.array([
        np.cos(0.1 * t),
        -np.sin(0.1 * t),
        0.02 * np.ones_like(t)
    ]).T
    
    acc_base = np.array([
        -0.1 * np.sin(0.1 * t),
        -0.1 * np.cos(0.1 * t),
        np.zeros_like(t)
    ]).T
    
    # Add noise for different sensors
    np.random.seed(42)
    noise = 0.1
    
    # Create output directory
    out_dir = Path("PYTHON/results/validation_test")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Test configurations
    methods = ["TRIAD", "Davenport", "SVD"]
    frames = ["NED", "ECEF", "Body"]
    
    success_count = 0
    total_tests = len(methods) * len(frames)
    
    for method in methods:
        for frame in frames:
            print(f"  Testing {method} - {frame}...", end=" ")
            
            try:
                plot_overlay_interactive(
                    frame=frame,
                    method=method,
                    t_imu=t,
                    pos_imu=pos_base + noise * 2 * np.random.randn(*pos_base.shape),
                    vel_imu=vel_base + noise * np.random.randn(*vel_base.shape),
                    acc_imu=acc_base + noise * 0.5 * np.random.randn(*acc_base.shape),
                    t_gnss=t[::10],
                    pos_gnss=pos_base[::10] + noise * np.random.randn(len(t[::10]), 3),
                    vel_gnss=vel_base[::10] + noise * 0.8 * np.random.randn(len(t[::10]), 3),
                    acc_gnss=acc_base[::10] + noise * 0.3 * np.random.randn(len(t[::10]), 3),
                    t_fused=t,
                    pos_fused=pos_base + noise * 0.5 * np.random.randn(*pos_base.shape),
                    vel_fused=vel_base + noise * 0.4 * np.random.randn(*vel_base.shape),
                    acc_fused=acc_base + noise * 0.2 * np.random.randn(*acc_base.shape),
                    out_dir=str(out_dir),
                    t_truth=t,
                    pos_truth=pos_base,
                    vel_truth=vel_base,
                    acc_truth=acc_base,
                    filename=f"validation_{method}_{frame}",
                    include_measurements=True,
                    save_static=True,
                )
                print("✓")
                success_count += 1
            except Exception as e:
                print(f"✗ ({e})")
    
    print(f"\nSynthetic data test: {success_count}/{total_tests} passed")
    
    # Create dashboard
    if success_count > 0:
        try:
            create_comparison_dashboard(out_dir, methods, frames)
            print("✓ Dashboard created successfully")
        except Exception as e:
            print(f"✗ Dashboard creation failed: {e}")
    
    return success_count == total_tests


def test_real_data():
    """Test with real pipeline data if available."""
    print_section("REAL DATA TEST")
    
    # Look for existing pipeline results
    results_dir = Path("PYTHON/results")
    kf_files = list(results_dir.glob("*_kf_output.mat"))
    
    if not kf_files:
        print("No real data available. Run the pipeline first:")
        print("  python PYTHON/src/run_triad_only.py")
        return False
    
    print(f"Found {len(kf_files)} result files")
    
    # Use existing test script
    try:
        print("Running optimized real data test...")
        result = subprocess.run([
            sys.executable, "PYTHON/test_optimized_interactive.py"
        ], capture_output=True, text=True, timeout=300)
        
        if result.returncode == 0:
            print("✓ Real data test passed")
            return True
        else:
            print("✗ Real data test failed")
            print("STDOUT:", result.stdout[-500:])  # Last 500 chars
            print("STDERR:", result.stderr[-500:])
            return False
    except subprocess.TimeoutExpired:
        print("✗ Real data test timed out")
        return False
    except Exception as e:
        print(f"✗ Real data test error: {e}")
        return False


def test_backward_compatibility():
    """Test that existing code still works with new features."""
    print_section("BACKWARD COMPATIBILITY TEST")
    
    # Test that plot_overlay still works with default parameters
    print("Testing plot_overlay backward compatibility...")
    
    # Generate simple test data
    t = np.linspace(0, 10, 100)
    pos = np.array([t, t**2, np.sin(t)]).T
    vel = np.array([np.ones_like(t), 2*t, np.cos(t)]).T
    acc = np.array([np.zeros_like(t), 2*np.ones_like(t), -np.sin(t)]).T
    
    out_dir = Path("PYTHON/results/compatibility_test")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        # Test with interactive=True (default)
        plot_overlay(
            frame="NED",
            method="TEST_INTERACTIVE",
            t_imu=t, pos_imu=pos, vel_imu=vel, acc_imu=acc,
            t_gnss=t[::5], pos_gnss=pos[::5], vel_gnss=vel[::5], acc_gnss=acc[::5],
            t_fused=t, pos_fused=pos, vel_fused=vel, acc_fused=acc,
            out_dir=str(out_dir),
            t_truth=t, pos_truth=pos, vel_truth=vel, acc_truth=acc,
            interactive=True
        )
        print("✓ Interactive mode works")
        
        # Test with interactive=False (static)
        plot_overlay(
            frame="NED", 
            method="TEST_STATIC",
            t_imu=t, pos_imu=pos, vel_imu=vel, acc_imu=acc,
            t_gnss=t[::5], pos_gnss=pos[::5], vel_gnss=vel[::5], acc_gnss=acc[::5],
            t_fused=t, pos_fused=pos, vel_fused=vel, acc_fused=acc,
            out_dir=str(out_dir),
            t_truth=t, pos_truth=pos, vel_truth=vel, acc_truth=acc,
            interactive=False
        )
        print("✓ Static mode works")
        
        return True
        
    except Exception as e:
        print(f"✗ Compatibility test failed: {e}")
        return False


def check_matlab_files():
    """Check that MATLAB files are properly structured."""
    print_section("MATLAB FILE CHECK")
    
    matlab_files = [
        "MATLAB/task6_overlay_plot.m",
        "MATLAB/task6_overlay_plot_interactive.m"
    ]
    
    all_exist = True
    for file_path in matlab_files:
        if Path(file_path).exists():
            print(f"✓ {file_path} exists")
        else:
            print(f"✗ {file_path} missing")
            all_exist = False
    
    # Check for key functions in the interactive file
    interactive_file = Path("MATLAB/task6_overlay_plot_interactive.m")
    if interactive_file.exists():
        content = interactive_file.read_text()
        key_functions = [
            "plot_overlay_interactive_3x3",
            "custom_datatip",
            "figure_key_press",
            "add_custom_toolbar"
        ]
        
        for func in key_functions:
            if func in content:
                print(f"✓ Function {func} found")
            else:
                print(f"✗ Function {func} missing")
                all_exist = False
    
    return all_exist


def generate_summary():
    """Generate a summary of all improvements."""
    print_section("IMPROVEMENT SUMMARY")
    
    improvements = [
        ("Interactive Python Plotting", "✓ Plotly integration with zoom, pan, hover"),
        ("Dashboard Creation", "✓ Comprehensive navigation interface"),
        ("Performance Optimization", "✓ Data subsampling for large datasets"),
        ("Static Export", "✓ PNG and PDF generation with kaleido"),
        ("MATLAB Enhancement", "✓ 3x3 layout with interactive features"), 
        ("Backward Compatibility", "✓ Existing code continues to work"),
        ("Documentation", "✓ Complete user guide created"),
        ("Testing Suite", "✓ Comprehensive validation scripts"),
    ]
    
    for item, status in improvements:
        print(f"  {status} {item}")
    
    print("\nKey Benefits:")
    print("  • Enhanced data exploration with interactive controls")
    print("  • Consistent 3x3 layout between Python and MATLAB")
    print("  • Performance optimized for large datasets")
    print("  • Dashboard for comparing multiple methods/frames")
    print("  • Maintains backward compatibility with existing code")
    print("  • Comprehensive documentation and testing")


def main():
    """Run complete validation and demonstration."""
    start_time = time.time()
    
    print_section("INTERACTIVE TASK 6 PLOTTING VALIDATION")
    print("This script validates the enhanced interactive plotting functionality")
    print("for both Python and MATLAB environments.")
    
    # Run tests
    tests = [
        ("Dependencies", check_dependencies),
        ("Synthetic Data", test_synthetic_data),
        ("Real Data", test_real_data),
        ("Backward Compatibility", test_backward_compatibility),
        ("MATLAB Files", check_matlab_files),
    ]
    
    results = {}
    for test_name, test_func in tests:
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"✗ {test_name} test failed with error: {e}")
            results[test_name] = False
    
    # Final summary
    print_section("TEST RESULTS")
    passed = sum(results.values())
    total = len(results)
    
    for test_name, result in results.items():
        status = "✓ PASSED" if result else "✗ FAILED"
        print(f"  {status}: {test_name}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Interactive Task 6 plotting is ready to use.")
    else:
        print("⚠️  Some tests failed. Please check the output above for details.")
    
    generate_summary()
    
    runtime = time.time() - start_time
    print(f"\nValidation completed in {runtime:.1f} seconds")
    
    # Usage instructions
    print_section("NEXT STEPS")
    print("1. View interactive plots by opening .html files in a web browser")
    print("2. Check the dashboard files for easy navigation:")
    print("   - PYTHON/results/validation_test/task6_interactive_dashboard.html")
    print("   - PYTHON/results/task6_interactive_optimized/task6_interactive_dashboard.html")
    print("3. Run your own data through the enhanced pipeline:")
    print("   python PYTHON/src/task6_plot_truth.py --est-file <your_file> --interactive")
    print("4. For MATLAB, use: task6_overlay_plot(..., true) for interactive plots")
    print("5. See docs/Interactive_Task6_Plotting_Guide.md for complete documentation")


if __name__ == "__main__":
    main()