"""Tracing and debug helpers for tasks.

This module provides decorators and utilities that enable rich debug logging
across the pipeline.  When the ``DEBUG`` environment variable is set to one of
``1``, ``true`` or ``yes`` the utilities automatically bootstrap and emit logs
under ``results/debug_log.txt``.

Usage
-----
>>> from utils.trace_utils import trace_task, dump_structure, log

The helpers mirror the MATLAB implementation found in ``MATLAB/src/utils`` and
are kept lightweight so they can be imported in any entry point without extra
setup.
"""

from __future__ import annotations

import functools
import inspect
import os
import pickle
import sys
import traceback
from typing import Any, Dict

import numpy as np
from utils.print_task_start import print_task_start


DEBUG = os.environ.get("DEBUG", "0").lower() in ("1", "true", "yes")


def set_debug(val: bool) -> None:
    """Enable or disable debug logging."""

    global DEBUG
    DEBUG = bool(val)
    state = "enabled" if DEBUG else "disabled"
    print(f"[DEBUG] Debug logging {state}")


def log_msg(msg: str, *args) -> None:
    """Log a debug message when ``DEBUG`` is active."""

    if DEBUG:
        text = msg % args if args else msg
        print(f"[DEBUG] {text}")


LOG_DIR = os.path.join(os.getcwd(), "results")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_PATH = os.path.join(LOG_DIR, "debug_log.txt")


def log(msg: str) -> None:
    """Write a debug message when DEBUG is enabled."""

    if not DEBUG:
        return
    from datetime import datetime

    line = f"[{datetime.now():%Y-%m-%d %H:%M:%S}] {msg}"
    print(line)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def _preview(value: Any) -> str:
    """Return a short textual preview of ``value`` for logs."""

    try:
        if isinstance(value, np.ndarray):
            return f"ndarray shape={value.shape} dtype={value.dtype}, head={value.reshape(-1)[:5]}"
        if isinstance(value, (list, tuple)):
            return f"{type(value).__name__} len={len(value)}, head={value[:5]}"
        if isinstance(value, dict):
            return f"dict keys(sample)={list(value.keys())[:5]}"
        s = str(value)
        return s if len(s) <= 200 else s[:200] + "…"
    except Exception as e:  # pragma: no cover - defensive
        return f"<preview failed: {e}>"


def dump_structure(
    name: str,
    obj: Any,
    depth: int = 0,
    max_depth: int = 5,
    visited: set[int] | None = None,
) -> None:
    """Recursively log the structure of complex objects."""

    if not DEBUG:
        return
    if visited is None:
        visited = set()
    indent = "  " * depth
    try:
        oid = id(obj)
        if oid in visited:
            log(f"{indent}{name}: <visited>")
            return
        visited.add(oid)

        tname = type(obj).__name__
        if isinstance(obj, np.ndarray):
            log(f"{indent}{name}: {tname} shape={obj.shape} dtype={obj.dtype}")
        elif isinstance(obj, (list, tuple, set)):
            log(f"{indent}{name}: {tname} len={len(obj)}")
            if depth < max_depth:
                for i, it in enumerate(list(obj)[:10]):
                    dump_structure(f"{name}[{i}]", it, depth + 1, max_depth, visited)
        elif isinstance(obj, dict):
            log(
                f"{indent}{name}: dict len={len(obj)} keys(sample)={list(obj.keys())[:10]}"
            )
            if depth < max_depth:
                for k in list(obj.keys())[:20]:
                    dump_structure(f"{name}.{k}", obj[k], depth + 1, max_depth, visited)
        elif hasattr(obj, "__dict__"):
            log(f"{indent}{name}: {tname} (object) attrs={list(vars(obj).keys())[:10]}")
            if depth < max_depth:
                for k, v in list(vars(obj).items())[:20]:
                    dump_structure(f"{name}.{k}", v, depth + 1, max_depth, visited)
        else:
            log(f"{indent}{name}: {tname} value={_preview(obj)}")
    except Exception as e:  # pragma: no cover - defensive
        log(f"{indent}{name}: <dump failed: {e}>")


def dump_locals_npz(task_name: str, local_vars: Dict[str, Any]) -> None:
    """Persist simple local variables to ``results/{task_name}_locals.npz``."""

    try:
        safe: Dict[str, Any] = {}
        for k, v in local_vars.items():
            if k.startswith("_"):
                continue
            try:
                if isinstance(v, np.ndarray):
                    safe[k] = v
                elif isinstance(v, (int, float, str, bool)):
                    safe[k] = v
                elif isinstance(v, (list, tuple)):
                    safe[k] = np.array(v, dtype=object)
                elif isinstance(v, dict):
                    safe[k] = np.array([("keys", list(v.keys()))], dtype=object)
                else:
                    continue
            except Exception:
                continue
        np.savez(os.path.join(LOG_DIR, f"{task_name}_locals.npz"), **safe)
        log(f"[DUMP] {task_name}: saved locals NPZ with {len(safe)} items")
    except Exception as e:  # pragma: no cover - debug helper
        log(f"[ERROR] {task_name}: npz dump failed: {e}")


def try_task(func_or_name, name_or_func: str, *args, **kwargs) -> None:
    """Run ``func`` and report errors without stopping execution.

    Usage
    -----
    try_task(func, "Task 1", arg1, ...)
    or
    try_task("Task 1", func, arg1, ...)
    """

    func = func_or_name
    task_name = name_or_func
    if isinstance(func_or_name, str) and callable(name_or_func):
        func, task_name = name_or_func, func_or_name

    print_task_start(task_name)
    try:
        func(*args, **kwargs)
        print(f"\u2713 {task_name} completed successfully.")
    except Exception as e:
        print(f"\u274c Error in {task_name}: {e}")
        if DEBUG:
            traceback.print_exc(file=sys.stdout)
            snapshot = {f"arg{i}": a for i, a in enumerate(args)}
            snapshot.update(kwargs)
            dump_locals_npz(task_name, snapshot)


def trace_task(task_name: str):
    """Decorator: log start/end and capture structure/locals on failure."""

    def deco(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            log("=" * 80)
            if DEBUG:
                print_task_start(task_name)
            log(f"START {task_name}")
            for i, a in enumerate(args):
                dump_structure(f"{task_name}.arg{i}", a)
            for k, v in kwargs.items():
                dump_structure(f"{task_name}.kw.{k}", v)
            try:
                out = fn(*args, **kwargs)
                dump_structure(f"{task_name}.return", out)
                log(f"✓ DONE {task_name}")
                return out
            except Exception as e:  # pragma: no cover - debug helper
                log(f"[ERROR] {task_name} failed: {e}")
                log(traceback.format_exc())
                snapshot = {f"arg{i}": a for i, a in enumerate(args)}
                snapshot.update({f"kw_{k}": v for k, v in kwargs.items()})
                dump_locals_npz(task_name, snapshot)
                return None
            finally:
                log("=" * 80)

        return wrapper

    return deco


def save_plot_interactive(
    fig, outbase: str, formats: list[str] | None = None, show: bool = True
) -> None:
    """Save ``fig`` in multiple formats while keeping interactive display."""

    import matplotlib.pyplot as plt

    if fig is None:
        fig = plt.gcf()
    if formats is None:
        formats = [".png", ".pickle"]
    os.makedirs(os.path.dirname(outbase), exist_ok=True)
    try:
        for ext in formats:
            if ext == ".pickle":
                with open(outbase + ".pickle", "wb") as f:
                    pickle.dump(fig, f)
            else:
                fig.savefig(outbase + ext, dpi=300, bbox_inches="tight")
    except Exception as e:  # pragma: no cover - debug helper
        log(f"[WARN] save_plot_interactive failed for {outbase}: {e}")
    if show:
        try:
            plt.show()
        except Exception:
            pass


def enable_if_env() -> None:
    """Auto-bootstrap debug logging when ``DEBUG`` is set."""

    if DEBUG:
        log("[DEBUG] tracing enabled via DEBUG env var")


enable_if_env()
