#!/usr/bin/env python3
"""Thin wrapper CLI for Task 6 overlay plots across frames."""

import argparse
from pathlib import Path
from typing import Dict, Tuple, Optional

try:  # pragma: no cover - optional helper
    from utils.run_id import run_id as build_run_id
except Exception:  # pragma: no cover
    build_run_id = None  # type: ignore


def parse_qbody(qstr: str | None) -> Tuple[float, float, float, float] | None:
    if not qstr:
        return None
    parts = [float(p) for p in qstr.split(",")]
    if len(parts) != 4:
        raise ValueError("--qbody must provide four comma-separated values")
    return tuple(parts)  # type: ignore


def main() -> None:
    parser = argparse.ArgumentParser(description="Task 6 overlay helper")
    parser.add_argument("--est-file", required=True, help="KF output .mat/.npz file")
    parser.add_argument("--truth-file", required=True, help="STATE_X file")
    parser.add_argument("--gnss-file", help="GNSS CSV for lat/lon fallback")
    parser.add_argument("--lat", type=float, help="Latitude in degrees")
    parser.add_argument("--lon", type=float, help="Longitude in degrees")
    parser.add_argument("--qbody", type=str, help="Body->NED quaternion qw,qx,qy,qz")
    parser.add_argument("--triad-file", help="TRIAD estimator output for comparison")
    parser.add_argument("--davenport-file", help="Davenport estimator output")
    parser.add_argument("--svd-file", help="SVD estimator output")
    parser.add_argument("--run-id", help="Override run identifier")
    args = parser.parse_args()

    est_path = Path(args.est_file)
    run_id_str: Optional[str] = args.run_id
    if run_id_str is None and build_run_id is not None:
        try:
            run_id_str = build_run_id(est_path.stem)
        except Exception:
            run_id_str = None
    if run_id_str is None:
        run_id_str = est_path.stem

    output_dir = Path("results") / run_id_str / "task6"
    output_dir.mkdir(parents=True, exist_ok=True)

    qbody_tuple = parse_qbody(args.qbody)

    from task6_overlay_all_frames import (
        run_task6_overlay_all_frames,
        run_task6_compare_methods_all_frames,
    )

    run_task6_overlay_all_frames(
        est_file=str(est_path),
        truth_file=args.truth_file,
        output_dir=str(output_dir),
        lat_deg=args.lat,
        lon_deg=args.lon,
        gnss_file=args.gnss_file,
        q_b2n_const=qbody_tuple,
    )

    # Build method file mapping
    method_files: Dict[str, str] = {}
    if args.triad_file:
        method_files["TRIAD"] = args.triad_file
    if args.davenport_file:
        method_files["Davenport"] = args.davenport_file
    if args.svd_file:
        method_files["SVD"] = args.svd_file

    if not method_files:
        triad_path = Path(args.est_file)
        davenport_path = triad_path.with_name(triad_path.name.replace("TRIAD", "Davenport"))
        svd_path = triad_path.with_name(triad_path.name.replace("TRIAD", "SVD"))
        for name, p in [("TRIAD", triad_path), ("Davenport", davenport_path), ("SVD", svd_path)]:
            if p.exists():
                method_files[name] = str(p)

    if method_files:
        run_task6_compare_methods_all_frames(
            method_files=method_files,
            truth_file=args.truth_file,
            output_dir=str(output_dir),
            lat_deg=args.lat,
            lon_deg=args.lon,
            gnss_file=args.gnss_file,
            q_b2n_const=qbody_tuple,
        )

    print(output_dir / "task6_overlay_NED.png")
    print(output_dir / "task6_overlay_ECEF.png")
    print(output_dir / "task6_overlay_BODY.png")
    if method_files:
        print(output_dir / "task6_methods_overlay_NED.png")
        print(output_dir / "task6_methods_overlay_ECEF.png")
        print(output_dir / "task6_methods_overlay_BODY.png")
    print(output_dir / "task6_overlay_manifest.json")


if __name__ == "__main__":  # pragma: no cover
    main()
