#!/usr/bin/env python3
"""Thin wrapper script invoking :mod:`fusion_single`.

This keeps ``GNSS_IMU_Fusion.py`` as the recommended entry point
mentioned in the README while delegating all real work to
``fusion_single.main``.  Any command line arguments are passed through
unchanged.
"""

from fusion_single import main


if __name__ == "__main__":
    main()

