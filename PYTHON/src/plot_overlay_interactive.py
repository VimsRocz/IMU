"""Interactive plotting module using Plotly for Task 6 overlay plots.

This module provides interactive plotting capabilities for IMU/GNSS fusion results,
allowing users to zoom, pan, and explore data with hover tooltips.
"""

from pathlib import Path
from typing import Optional, Tuple, Union
import numpy as np

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    import plotly.io as pio
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False


def plot_overlay_interactive(
    frame: str,
    method: str,
    t_imu: np.ndarray,
    pos_imu: np.ndarray,
    vel_imu: np.ndarray,
    acc_imu: np.ndarray,
    t_gnss: np.ndarray,
    pos_gnss: np.ndarray,
    vel_gnss: np.ndarray,
    acc_gnss: np.ndarray,
    t_fused: np.ndarray,
    pos_fused: np.ndarray,
    vel_fused: np.ndarray,
    acc_fused: np.ndarray,
    out_dir: str,
    truth: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None,
    *,
    t_truth: Optional[np.ndarray] = None,
    pos_truth: Optional[np.ndarray] = None,
    vel_truth: Optional[np.ndarray] = None,
    acc_truth: Optional[np.ndarray] = None,
    filename: Optional[str] = None,
    include_measurements: bool = True,
    save_static: bool = True,
) -> None:
    """Create interactive overlay plot using Plotly.
    
    Parameters
    ----------
    frame : str
        Coordinate frame ("NED", "ECEF", or "Body")
    method : str
        Attitude estimation method name
    t_imu, pos_imu, vel_imu, acc_imu : np.ndarray
        IMU time and position/velocity/acceleration data
    t_gnss, pos_gnss, vel_gnss, acc_gnss : np.ndarray
        GNSS time and position/velocity/acceleration data
    t_fused, pos_fused, vel_fused, acc_fused : np.ndarray
        Fused estimate time and position/velocity/acceleration data
    out_dir : str
        Output directory for saved plots
    truth : tuple, optional
        Deprecated, use t_truth, pos_truth, etc. instead
    t_truth, pos_truth, vel_truth, acc_truth : np.ndarray, optional
        Ground truth time and position/velocity/acceleration data
    filename : str, optional
        Custom filename for the saved plot
    include_measurements : bool, optional
        Whether to include IMU and GNSS measurements (default True)
    save_static : bool, optional
        Whether to also save static PNG/PDF versions (default True)
    """
    if not PLOTLY_AVAILABLE:
        raise ImportError(
            "Plotly not available. Install with: pip install plotly>=5.0"
        )
    
    if truth is not None:
        t_truth, pos_truth, vel_truth, acc_truth = truth

    # Define axis labels for different coordinate frames
    axis_labels = {
        "NED": ["ΔN [m]", "ΔE [m]", "ΔD [m]"],
        "ECEF": ["X [m]", "Y [m]", "Z [m]"],
        "Body": ["X [m]", "Y [m]", "Z [m]"],
    }
    cols = axis_labels.get(frame, ["X [m]", "Y [m]", "Z [m]"])

    # Create subplot figure with 3x3 layout
    subplot_titles = [
        f"Position {cols[0]}", f"Position {cols[1]}", f"Position {cols[2]}",
        f"Velocity {cols[0]}", f"Velocity {cols[1]}", f"Velocity {cols[2]}",
        f"Acceleration {cols[0]}", f"Acceleration {cols[1]}", f"Acceleration {cols[2]}"
    ]
    
    fig = make_subplots(
        rows=3, cols=3,
        subplot_titles=subplot_titles,
        shared_xaxes=True,
        vertical_spacing=0.08,
        horizontal_spacing=0.06
    )

    # Data arrays for the three rows (position, velocity, acceleration)
    datasets = [
        (pos_imu, pos_gnss, pos_fused, pos_truth, "Position"),
        (vel_imu, vel_gnss, vel_fused, vel_truth, "Velocity"),
        (acc_imu, acc_gnss, acc_fused, acc_truth, "Acceleration"),
    ]

    # Color scheme matching the matplotlib version
    colors = {
        "Truth": "black",
        "Fused": "#1f77b4",  # tab:blue
        "GNSS": "#2ca02c",   # tab:green
        "IMU": "#ff7f0e",    # tab:orange
    }

    # Track if acceleration truth data is available
    has_acc_truth = acc_truth is not None and len(acc_truth) > 0

    # Add traces for each subplot
    for row, (imu, gnss, fused, truth_data, data_type) in enumerate(datasets):
        for col in range(3):
            subplot_row = row + 1
            subplot_col = col + 1
            
            # Determine units for hover template
            if data_type == "Position":
                units = "m"
            elif data_type == "Velocity":
                units = "m/s"
            else:  # Acceleration
                units = "m/s²"
            
            # Add measured data if requested
            if include_measurements:
                # GNSS measurements
                fig.add_trace(
                    go.Scatter(
                        x=t_gnss,
                        y=gnss[:, col],
                        mode='lines',
                        name='Measured GNSS',
                        line=dict(color=colors["GNSS"]),
                        hovertemplate=f'<b>GNSS {data_type}</b><br>' +
                                     f'Time: %{{x:.2f}} s<br>' +
                                     f'Value: %{{y:.3f}} {units}<extra></extra>',
                        showlegend=(row == 0 and col == 0),
                    ),
                    row=subplot_row, col=subplot_col
                )
                
                # IMU measurements
                fig.add_trace(
                    go.Scatter(
                        x=t_imu,
                        y=imu[:, col],
                        mode='lines',
                        name='Measured IMU',
                        line=dict(color=colors["IMU"], dash='dash'),
                        hovertemplate=f'<b>IMU {data_type}</b><br>' +
                                     f'Time: %{{x:.2f}} s<br>' +
                                     f'Value: %{{y:.3f}} {units}<extra></extra>',
                        showlegend=(row == 0 and col == 0),
                    ),
                    row=subplot_row, col=subplot_col
                )
            
            # Add truth data if available (skip acceleration if no truth data)
            if (truth_data is not None and 
                not (row == 2 and not has_acc_truth)):
                fig.add_trace(
                    go.Scatter(
                        x=t_truth,
                        y=truth_data[:, col],
                        mode='lines',
                        name='Truth',
                        line=dict(color=colors["Truth"]),
                        hovertemplate=f'<b>Truth {data_type}</b><br>' +
                                     f'Time: %{{x:.2f}} s<br>' +
                                     f'Value: %{{y:.3f}} {units}<extra></extra>',
                        showlegend=(row == 0 and col == 0),
                    ),
                    row=subplot_row, col=subplot_col
                )
            
            # Add fused estimate
            fig.add_trace(
                go.Scatter(
                    x=t_fused,
                    y=fused[:, col],
                    mode='lines',
                    name=f'Fused GNSS+IMU ({method})',
                    line=dict(color=colors["Fused"], dash='dot'),
                    hovertemplate=f'<b>Fused {data_type}</b><br>' +
                                 f'Time: %{{x:.2f}} s<br>' +
                                 f'Value: %{{y:.3f}} {units}<br>' +
                                 f'Method: {method}<extra></extra>',
                    showlegend=(row == 0 and col == 0),
                ),
                row=subplot_row, col=subplot_col
            )

    # Update layout for better interactivity
    title_text = f"Task 6 – {method} – {frame} Frame"
    if t_truth is not None:
        title_text += " (Interactive: Fused vs. Truth)"
    elif include_measurements:
        title_text += " (Interactive: Fused vs. Measured)"
    else:
        title_text += " (Interactive: Fused)"

    fig.update_layout(
        title=dict(
            text=title_text,
            x=0.5,
            font=dict(size=16)
        ),
        height=800,
        width=1200,
        hovermode='closest',
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        ),
        # Enable zoom and pan
        dragmode='zoom',
    )

    # Add axis labels for bottom row (time)
    for col in range(1, 4):
        fig.update_xaxes(title_text="Time [s]", row=3, col=col)

    # Add y-axis labels for leftmost column
    fig.update_yaxes(title_text="Position [m]", row=1, col=1)
    fig.update_yaxes(title_text="Velocity [m/s]", row=2, col=1)
    fig.update_yaxes(title_text="Acceleration [m/s²]", row=3, col=1)

    # Configure default plotly settings for interactivity
    config = {
        'displayModeBar': True,
        'displaylogo': False,
        'modeBarButtonsToAdd': ['pan2d', 'select2d', 'lasso2d', 'resetScale2d'],
        'toImageButtonOptions': {
            'format': 'png',
            'filename': f'{method}_{frame}_task6_interactive',
            'height': 800,
            'width': 1200,
            'scale': 2
        }
    }

    # Save interactive HTML
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    
    if filename is not None:
        html_file = out_path / f"{filename}.html"
    else:
        html_file = out_path / f"{method}_{frame}_task6_interactive.html"
    
    fig.write_html(str(html_file), config=config)
    print(f"Saved interactive plot: {html_file}")

    # Optionally save static versions for compatibility
    if save_static:
        try:
            # Save as PNG (requires kaleido)
            png_file = html_file.with_suffix(".png")
            fig.write_image(str(png_file), width=1200, height=800, scale=2)
            print(f"Saved static PNG: {png_file}")
        except Exception as e:
            print(f"Note: Could not save PNG (install kaleido for static export): {e}")
        


def create_comparison_dashboard(
    results_dir: Union[str, Path],
    methods: list = None,
    frames: list = None
) -> None:
    """Create a comprehensive dashboard comparing multiple methods and frames.
    
    Parameters
    ----------
    results_dir : str or Path
        Directory containing the individual interactive plots
    methods : list, optional
        List of methods to include (default: ["TRIAD", "Davenport", "SVD"])
    frames : list, optional
        List of frames to include (default: ["NED", "ECEF", "Body"])
    """
    if not PLOTLY_AVAILABLE:
        print("Plotly not available for dashboard creation")
        return

    if methods is None:
        methods = ["TRIAD", "Davenport", "SVD"]
    if frames is None:
        frames = ["NED", "ECEF", "Body"]

    results_dir = Path(results_dir)
    
    # Create simple HTML dashboard with links to individual plots
    dashboard_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>IMU/GNSS Fusion - Interactive Task 6 Results Dashboard</title>
        <style>
            body {{ font-family: Arial, sans-serif; margin: 40px; }}
            .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }}
            .card {{ border: 1px solid #ddd; border-radius: 8px; padding: 20px; background: #f9f9f9; }}
            .card h3 {{ margin-top: 0; color: #333; }}
            .card a {{ display: inline-block; padding: 10px 15px; background: #007bff; color: white; 
                      text-decoration: none; border-radius: 4px; margin: 5px 0; }}
            .card a:hover {{ background: #0056b3; }}
            h1 {{ text-align: center; color: #333; }}
            .subtitle {{ text-align: center; color: #666; margin-bottom: 30px; }}
        </style>
    </head>
    <body>
        <h1>Interactive Task 6 Results Dashboard</h1>
        <p class="subtitle">IMU/GNSS Fusion - Interactive Overlay Plots</p>
        
        <div class="grid">
    """
    
    for method in methods:
        for frame in frames:
            html_file = results_dir / f"{method}_{frame}_task6_interactive.html"
            if html_file.exists():
                dashboard_html += f"""
                <div class="card">
                    <h3>{method} - {frame} Frame</h3>
                    <p>Interactive overlay plot showing fused IMU/GNSS trajectory compared with truth data.</p>
                    <a href="{html_file.name}" target="_blank">Open Interactive Plot</a>
                </div>
                """
    
    dashboard_html += """
        </div>
        <br>
        <div style="text-align: center; color: #666; margin-top: 40px;">
            <p>Each plot supports zooming, panning, and hover tooltips for detailed data exploration.</p>
        </div>
    </body>
    </html>
    """
    
    dashboard_file = results_dir / "task6_interactive_dashboard.html"
    with open(dashboard_file, 'w') as f:
        f.write(dashboard_html)
    
    print(f"Created interactive dashboard: {dashboard_file}")