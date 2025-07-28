"""Placeholder for the MATLAB ``plot_xyz_timeseries`` helper.

The MATLAB implementation plots fused and optional GNSS position, velocity
and acceleration timeseries in a 3x3 grid.  This Python stub mirrors the
function signature so that scripts can import it, but no equivalent
functionality is provided here yet.
"""


def plot_xyz_timeseries(
    time_fused,
    pos_fused,
    vel_fused,
    acc_fused,
    time_gnss,
    pos_gnss,
    vel_gnss,
    acc_gnss,
    fig_title,
    out_prefix,
    axis_labels=None,
    method=None,
):
    """Raise :class:`NotImplementedError` when called.

    Parameters match the MATLAB helper: all position, velocity and
    acceleration arrays are expected to be ``3xN``.  The GNSS arrays may be
    empty to omit the overlay.
    """

    raise NotImplementedError(
        "plot_xyz_timeseries is implemented in MATLAB only"
    )
