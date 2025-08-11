"""Utilities for task-level tracing and debug logging.

This module provides decorators and helpers to wrap task execution with
logging, variable inspection and error capture.

Usage
-----
from utils.trace_utils import trace_task, log, debug_var

Environment
-----------
DEBUG environment variable enables logging when set to 1/true/yes.
All logs and dumps are stored under ``./results``.
"""

import os
import sys
import pickle
import functools
import inspect
import traceback
import numpy as np

DEBUG = os.environ.get("DEBUG", "0").lower() in ("1", "true", "yes")

LOG_DIR = os.path.join(os.getcwd(), "results")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_PATH = os.path.join(LOG_DIR, "debug_log.txt")

def log(msg: str) -> None:
    """Write a debug message when DEBUG is enabled."""
    if not DEBUG:
        return
    from datetime import datetime
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line + "\n")

def debug_var(name: str, val, n: int = 5) -> None:
    """Log basic information about a variable.

    Parameters
    ----------
    name : str
        Variable name to display in the log.
    val : Any
        The variable to inspect.
    n : int, optional
        Number of items to preview for arrays or tables, by default 5.
    """
    if not DEBUG:
        return
    try:
        import pandas as pd  # lazy import
        if hasattr(val, "shape"):
            log(f"{name}: type={type(val).__name__}, shape={val.shape}")
        elif isinstance(val, (list, tuple, dict)):
            log(f"{name}: type={type(val).__name__}, len={len(val)}")
        else:
            log(f"{name}: type={type(val).__name__}")
        if isinstance(val, np.ndarray):
            log(f"  preview: {val[:n]}")
        elif isinstance(val, pd.DataFrame):
            log("  preview:\n" + str(val.head(n)))
    except Exception as e:  # pragma: no cover - debug helper
        log(f"[WARN] debug_var failed for {name}: {e}")

def dump_vars(filename: str, local_vars: dict) -> None:
    """Save all non-private locals to NPZ for post-mortem analysis."""
    try:
        safe_vars = {k: v for k, v in local_vars.items() if not k.startswith("_")}
        np.savez(os.path.join(LOG_DIR, filename), **safe_vars)
        log(f"[DUMP] Saved variables to {filename}")
    except Exception as e:  # pragma: no cover - debug helper
        log(f"[ERROR] Failed to dump vars: {e}")

def trace_task(task_name: str):
    """Decorator for task entry points with error capture and continuation."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            log(f"▶ START {task_name}")
            try:
                out = fn(*args, **kwargs)
                log(f"✓ DONE {task_name}")
                return out
            except Exception as e:  # pragma: no cover - debug helper
                log(f"[ERROR] {task_name} failed: {e}")
                log(traceback.format_exc())
                dump_vars(f"{task_name}_locals.npz", locals())
                # Continue execution by returning None
                return None
        return wrapper
    return deco
