import pytest
from src.naming import make_tag, script_name, output_dir, plot_filename


def test_make_tag():
    assert (
        make_tag("IMU_X001.dat", "GNSS_X001.csv", "TRIAD") == "IMU_X001_GNSS_X001_TRIAD"
    )


def test_script_name():
    assert script_name("IMU_X001.dat", "TRIAD", 5) == "IMU_X001_TRIAD_task5.py"


def test_output_dir(tmp_path):
    d = output_dir(5, "IMU_X001.dat", "GNSS_X001.csv", "TRIAD", tmp_path)
    assert d == tmp_path


def test_plot_filename():
    name = plot_filename("IMU_X001.dat", "GNSS_X001.csv", "TRIAD", 7, "3", "residuals")
    assert name == "IMU_X001_GNSS_X001_TRIAD_task7_3_residuals.pdf"
