"""Task 1 placeholder for the unified pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

from utils.trace_utils import try_task


@try_task("Task 1")
def run(state: Dict[str, Any], *args, **kwargs) -> Dict[str, Any]:
    """Placeholder implementation for Task 1.

    The real pipeline will populate this with actual computation.
    """
    logger = state.get("logger")
    task_dir = Path(state["run_dir"]) / f"Task1"
    task_dir.mkdir(parents=True, exist_ok=True)
    if logger:
        logger.debug("Task 1 stub executed")
    return {"status": "ok", "inputs": {}, "outputs": {}, "artifacts": {}, "meta": {"duration_s": 0.0, "notes": "stub"}}
