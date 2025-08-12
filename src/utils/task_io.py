"""Lightweight Python analogue of the MATLAB ``TaskIO`` utility."""
from __future__ import annotations
from pathlib import Path
from typing import Any
import scipy.io as sio


class TaskIO:
    """Save and load task structs using ``scipy.io``."""

    @staticmethod
    def save(task_name: str, data: dict, path: str) -> str:
        """Save *data* under ``task_name`` to *path*."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        sio.savemat(p, {task_name: data})
        return str(p)

    @staticmethod
    def load(task_name: str, path: str) -> Any:
        """Load *task_name* struct from *path*."""
        data = sio.loadmat(path, squeeze_me=True, struct_as_record=False)
        return data.get(task_name, data)
