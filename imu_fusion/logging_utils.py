import logging
import os
from datetime import datetime
from typing import Iterable

import numpy as np


def setup_logging(log_dir: str = "logs") -> str:
    """Configure logging to console and a file in *log_dir*."""
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, "run.log")
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
        handlers=[logging.FileHandler(log_file, mode="a"), logging.StreamHandler()],
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    return log_file


def log_static_validation(acc: np.ndarray, gyro: np.ndarray) -> None:
    """Log static mean/variance and warn if values look suspicious."""
    mean_acc = np.mean(acc, axis=0)
    mean_gyro = np.mean(gyro, axis=0)
    logging.info("Static acc mean: %s", mean_acc)
    logging.info("Static gyro mean: %s", mean_gyro)
    logging.info("Static acc variance: %s", np.var(acc, axis=0))
    logging.info("Static gyro variance: %s", np.var(gyro, axis=0))
    norm_g = np.linalg.norm(-mean_acc)
    if abs(norm_g - 9.81) > 0.3:
        logging.warning(
            "Accelerometer gravity norm suspicious: %.3f m/s² (should be ~9.81).", norm_g
        )
    if np.linalg.norm(mean_gyro) > 1e-3:
        logging.warning("Gyro shows non-zero static mean: check for bias!")


def append_summary(log_dir: str, lines: Iterable[str]) -> str:
    """Append a short summary section to ``run_summary.txt`` in *log_dir*."""
    os.makedirs(log_dir, exist_ok=True)
    path = os.path.join(log_dir, "run_summary.txt")
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"=== RUN at {datetime.utcnow().isoformat()} ===\n")
        for line in lines:
            f.write(line.rstrip() + "\n")
    return path

