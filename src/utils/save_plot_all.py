"""Utility to save matplotlib figures in multiple formats.

The helper mirrors the MATLAB implementation and ensures a figure is written
as ``.png`` and ``.pickle`` by default. Additional formats can be requested via
the ``formats`` argument. The function always returns a list of written
file paths so callers can index artefacts easily.
"""

from __future__ import annotations

import pickle
from pathlib import Path
from typing import Iterable, List

try:  # pragma: no cover - matplotlib optional in tests
    import matplotlib.pyplot as plt
    import matplotlib.figure
except Exception:  # pragma: no cover
    plt = None  # type: ignore
    matplotlib = None  # type: ignore


def save_plot_all(
    fig: "matplotlib.figure.Figure",
    basepath: str,
    formats: Iterable[str] | None = None,
    *,
    show_plot: bool = False,
    logger=None,
) -> List[str]:
    """Save ``fig`` to all ``formats`` and return list of written paths."""
    base = Path(basepath)
    base.parent.mkdir(parents=True, exist_ok=True)
    fmts = [f.lower() for f in (formats or ["png", "pickle"])]
    written: List[str] = []
    for fmt in fmts:
        suffix = "." + fmt.lstrip(".")
        out = base.with_suffix(suffix)
        if fmt in {"png", "pdf"}:
            fig.savefig(out, dpi=300, bbox_inches="tight")
        elif fmt == "pickle":
            with open(out, "wb") as fh:
                pickle.dump(fig, fh)
        else:  # unsupported format
            continue
        written.append(str(out))
        if logger:
            logger.debug("Saved %s", out)
        else:  # pragma: no cover
            print(f"Saved -> {out}")
    if show_plot and plt is not None:
        plt.show()
    return written


__all__ = ["save_plot_all"]
