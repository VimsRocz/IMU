"""Expose helpers from the sibling :mod:`utils` module.

The project ships a top-level ``utils.py`` module providing common helper
functions such as ``compute_C_ECEF_to_NED`` and ``save_mat``.  A package with
additional utilities lives next to it under ``utils/`` which shadows the module
name.  Importing ``from utils import ...`` would therefore resolve to this
package and hide the functions defined in ``utils.py``.

To maintain backwards compatibility we explicitly load the sibling module under
an alias and re-export the expected public helpers.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

# Load the sibling ``utils.py`` module (two levels up from this file) under the
# name ``base_utils`` to avoid the package name collision.
_utils_path = pathlib.Path(__file__).resolve().parent.parent / "utils.py"
_spec = importlib.util.spec_from_file_location("base_utils", _utils_path)
base_utils = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None  # for type checkers
_spec.loader.exec_module(base_utils)

# Re-export commonly used helpers so that ``from utils import foo`` works.
compute_C_ECEF_to_NED = base_utils.compute_C_ECEF_to_NED
ensure_dependencies = base_utils.ensure_dependencies
interpolate_series = base_utils.interpolate_series
is_static = base_utils.is_static
ecef_to_geodetic = base_utils.ecef_to_geodetic
ecef_to_ned = base_utils.ecef_to_ned
save_mat = base_utils.save_mat
save_plot_fig = base_utils.save_plot_fig
save_plot_mat = base_utils.save_plot_mat
zero_base_time = base_utils.zero_base_time

__all__ = [
    "compute_C_ECEF_to_NED",
    "ensure_dependencies",
    "interpolate_series",
    "is_static",
    "ecef_to_geodetic",
    "ecef_to_ned",
    "save_mat",
    "save_plot_fig",
    "save_plot_mat",
    "zero_base_time",
]
