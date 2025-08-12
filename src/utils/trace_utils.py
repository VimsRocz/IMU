"""Logging and task execution helpers.

This module centralises logging configuration and provides a decorator for
robust task execution.  It intentionally mirrors the MATLAB utilities while
using Pythonic interfaces.
"""

from __future__ import annotations

import functools
import inspect
import logging
import os
import time
from pathlib import Path
from typing import Any, Callable, Dict

import numpy as np


def get_logger(run_dir: str | Path, debug: bool = False) -> logging.Logger:
    """Return a configured :class:`logging.Logger` instance."""
    run_path = Path(run_dir)
    log_dir = run_path / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    logger = logging.getLogger("imu_pipeline")
    logger.setLevel(logging.DEBUG)
    for h in list(logger.handlers):
        logger.removeHandler(h)

    fmt = logging.Formatter("[%(asctime)s] %(levelname)s %(message)s", "%H:%M:%S")
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG if debug else logging.INFO)
    ch.setFormatter(fmt)
    fh = logging.FileHandler(log_dir / f"run_{time.strftime('%Y%m%d_%H%M%S')}.log")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    logger.addHandler(ch)
    logger.addHandler(fh)

    np.set_printoptions(precision=4, suppress=True)
    os.environ["DEBUG"] = "1" if debug else "0"
    np.random.seed(0)
    return logger


def try_task(name: str) -> Callable[[Callable[..., Dict[str, Any]]], Callable[..., Dict[str, Any]]]:
    """Decorator to execute a task with start/end logging and error capture."""

    def decorator(func: Callable[..., Dict[str, Any]]) -> Callable[..., Dict[str, Any]]:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            logger: logging.Logger | None = None
            if args and isinstance(args[0], dict) and "logger" in args[0]:
                logger = args[0]["logger"]
            else:
                logger = kwargs.get("logger", logging.getLogger("imu_pipeline"))

            logger.info(f"\u25B6 {name} START")
            t0 = time.time()
            try:
                result = func(*args, **kwargs)
                dt = time.time() - t0
                logger.info(f"\u2714 {name} OK in {dt:.2f}s")
                return result
            except Exception as exc:  # pragma: no cover - defensive
                dt = time.time() - t0
                logger.exception(f"\u2716 {name} FAILED after {dt:.2f}s: {exc}")
                run_dir = None
                if args and isinstance(args[0], dict):
                    run_dir = args[0].get("run_dir")
                run_dir = Path(run_dir or Path.cwd())
                dbg_dir = run_dir / "debug"
                dbg_dir.mkdir(parents=True, exist_ok=True)
                dump_path = dbg_dir / f"{name.replace(' ', '_')}_locals.npz"
                try:
                    frame = inspect.trace()[-1][0]
                    local_vars = frame.f_locals
                    safe: Dict[str, Any] = {}
                    for k, v in local_vars.items():
                        if k.startswith("_"):
                            continue
                        if isinstance(v, (int, float, str, bool, np.ndarray)):
                            safe[k] = v
                    np.savez_compressed(dump_path, **safe)
                    logger.debug("Dumped locals -> %s", dump_path)
                except Exception:
                    logger.debug("Failed to dump locals to %s", dump_path)
                raise

        return wrapper

    return decorator


__all__ = ["get_logger", "try_task"]
