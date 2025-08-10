import subprocess
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.io import savemat


try:  # Optional import; matplotlib might not be installed for tests
    import matplotlib.figure
except Exception:  # pragma: no cover - handled at runtime
    matplotlib = None  # type: ignore


def _extract_axes_schema(fig: "matplotlib.figure.Figure") -> dict:
    """Extract basic line plot info from the first axes of ``fig``."""
    ax = fig.gca()
    lines_out = []
    for ln in ax.get_lines():
        x = ln.get_xdata(orig=False)
        y = ln.get_ydata(orig=False)
        label = ln.get_label()
        if label and label.startswith("_"):
            label = ""
        lines_out.append(
            dict(
                x=np.asarray(x, dtype=float),
                y=np.asarray(y, dtype=float),
                label=label or "",
                color=ln.get_color(),
                linestyle=ln.get_linestyle(),
                linewidth=float(ln.get_linewidth()),
                marker=ln.get_marker(),
                markersize=float(ln.get_markersize()),
            )
        )

    schema = dict(
        title=ax.get_title() or "",
        xlabel=ax.get_xlabel() or "",
        ylabel=ax.get_ylabel() or "",
        lines=lines_out,
        legend=dict(
            loc="best",
            visible=any(l.get("label") for l in lines_out),
        ),
    )
    return schema


def _ensure_dir(p: str) -> None:
    Path(p).parent.mkdir(parents=True, exist_ok=True)


def _try_matlab_engine(mat_path: str, fig_out: str) -> bool:
    """Attempt to use MATLAB Engine to save ``fig_out`` from ``mat_path``."""
    try:
        import matlab.engine  # type: ignore
    except Exception:
        return False
    try:  # pragma: no cover - requires MATLAB
        eng = matlab.engine.start_matlab()
        eng.rebuild_and_save_fig(mat_path, fig_out, nargout=0)
        eng.quit()
        return True
    except Exception:
        return False


def _try_matlab_cli(mat_path: str, fig_out: str) -> bool:
    """Attempt to call MATLAB from the command line to save ``fig_out``."""
    cmd = [
        "matlab",
        "-batch",
        f"rebuild_and_save_fig('{mat_path.replace('\\', '/')}', '{fig_out.replace('\\', '/')}')",
    ]
    try:  # pragma: no cover - requires MATLAB
        subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return True
    except Exception:
        return False


def save_plot_all(
    fig: "matplotlib.figure.Figure",
    basepath: str,
    formats: Iterable[str] = (".png", ".pdf", ".svg", ".fig"),
) -> None:
    """Save a matplotlib ``fig`` to multiple ``formats``.

    Parameters
    ----------
    fig : matplotlib.figure.Figure
        The figure to save.
    basepath : str
        Path without extension where files will be written.
    formats : Iterable[str], optional
        Iterable of extensions (e.g. ``(".png", ".pdf")``). ``".fig"``
        triggers MATLAB export using the engine or command line.
    """
    _ensure_dir(basepath)

    # raster/vector exports
    for ext in (e.lower() for e in formats if e.lower() in {".png", ".pdf", ".svg"}):
        fig.savefig(basepath + ext, dpi=300, bbox_inches="tight")
        print(f"Saved -> {basepath + ext}")

    if ".fig" not in [e.lower() for e in formats]:
        return

    schema = _extract_axes_schema(fig)
    mat_payload = dict(
        title=schema["title"],
        xlabel=schema["xlabel"],
        ylabel=schema["ylabel"],
        legend_loc=schema["legend"]["loc"],
        legend_visible=int(bool(schema["legend"]["visible"])),
        n_lines=len(schema["lines"]),
    )

    lines = schema["lines"]
    mat_payload["line_x"] = [np.asarray(l["x"], dtype=float) for l in lines]
    mat_payload["line_y"] = [np.asarray(l["y"], dtype=float) for l in lines]
    mat_payload["line_label"] = np.array([l["label"] for l in lines], dtype=object)
    mat_payload["line_color"] = np.array([l["color"] for l in lines], dtype=object)
    mat_payload["line_ls"] = np.array([l["linestyle"] for l in lines], dtype=object)
    mat_payload["line_lw"] = np.array([float(l["linewidth"]) for l in lines], dtype=float)
    mat_payload["line_marker"] = np.array([l["marker"] for l in lines], dtype=object)
    mat_payload["line_mks"] = np.array([float(l["markersize"]) for l in lines], dtype=float)

    mat_path = basepath + "_pyplot_dump.mat"
    savemat(mat_path, mat_payload)
    fig_out = basepath + ".fig"

    if _try_matlab_engine(mat_path, fig_out) or _try_matlab_cli(mat_path, fig_out):
        print(f"Saved -> {fig_out}")
    else:
        print(
            "WARNING: MATLAB not available; skipped .fig export. Kept .mat for later conversion:",
            mat_path,
        )


if __name__ == "__main__":
    import argparse
    import numpy as np
    import matplotlib.pyplot as plt

    ap = argparse.ArgumentParser(description="Demo plot export utility")
    ap.add_argument("--demo", type=str, default="results/demo_plot", help="Output basepath")
    args = ap.parse_args()

    x = np.linspace(0, 2 * np.pi, 400)
    fig = plt.figure()
    ax = fig.gca()
    ax.plot(x, np.sin(x), label="sin")
    ax.plot(x, np.cos(x), "--", label="cos")
    ax.set_title("Demo")
    ax.set_xlabel("rad")
    ax.set_ylabel("value")
    ax.legend()

    save_plot_all(fig, args.demo)
