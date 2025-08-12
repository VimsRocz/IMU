import os
from pathlib import Path

ALLOWED_SUBDIRS = {"legacy", "archival"}

def test_results_flat_structure(tmp_path):
    results = tmp_path / "results"
    results.mkdir()
    # create sample files
    (results / "IMU_X001_GNSS_X001_TRIAD_task6_demo.pdf").touch()
    (results / "sample_task7_plot.png").touch()
    (results / "legacy").mkdir()
    (results / "legacy" / "old.txt").touch()

    # scan directory
    for p in results.iterdir():
        if p.is_dir():
            assert p.name in ALLOWED_SUBDIRS, f"Unexpected subdirectory {p.name}"
        else:
            assert "task" in p.name, f"Missing task number in {p.name}"
