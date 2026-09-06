#!/usr/bin/env python3
"""Summarize multi-run evo results and copy the best covered runs."""

import argparse
import csv
import json
import shutil
import zipfile
from pathlib import Path



def read_stats(path: Path):
    with zipfile.ZipFile(path) as archive:
        return json.loads(archive.read("stats.json"))


def count_poses(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return sum(1 for line in stream if line.strip() and not line.startswith("#"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_root", type=Path)
    parser.add_argument("best_root", type=Path)
    parser.add_argument(
        "--expected-poses", type=Path, required=True,
        help="JSON object mapping sequence names to expected pose counts",
    )
    args = parser.parse_args()
    expected_poses = json.loads(args.expected_poses.read_text(encoding="utf-8"))
    if not isinstance(expected_poses, dict) or not expected_poses:
        parser.error("expected-poses must be a non-empty JSON object")
    for name, count in expected_poses.items():
        if not name or name in {".", ".."} or "/" in name or "\\" in name:
            parser.error("sequence names must be single directory names")
        if type(count) is not int or count <= 0:
            parser.error("expected pose counts must be positive integers")
    source_root = args.run_root.resolve()
    output_root = args.best_root.resolve()
    if (source_root == output_root or source_root in output_root.parents
            or output_root in source_root.parents):
        parser.error("run_root and best_root must be separate, non-nested directories")
    if args.best_root.exists():
        parser.error("best_root must not already exist; choose a new output directory")

    rows = []
    for sequence, expected in expected_poses.items():
        for run_dir in sorted((args.run_root / sequence).glob("run_*")):
            trajectory = run_dir / "CameraTrajectory.txt"
            if not trajectory.exists():
                continue
            ape = read_stats(run_dir / "ape.zip")
            rpe = read_stats(run_dir / "rpe.zip")
            point_rpe_path = run_dir / "rpe_point_distance.zip"
            point_rpe = read_stats(point_rpe_path) if point_rpe_path.exists() else None
            poses = count_poses(trajectory)
            rows.append({
                "sequence": sequence,
                "run": run_dir.name,
                "expected_poses": expected,
                "poses": poses,
                "coverage": poses / expected,
                "ate_rmse_m": ape["rmse"],
                "rpe_rmse_m": rpe["rmse"],
                "rpe_point_distance_rmse_m": point_rpe["rmse"] if point_rpe else None,
                "run_dir": run_dir,
            })

    args.best_root.mkdir(parents=True, exist_ok=True)
    best_records = []
    for sequence in expected_poses:
        sequence_rows = [row for row in rows if row["sequence"] == sequence]
        if not sequence_rows:
            print(f"Skipping {sequence}: no runs with trajectories")
            continue
        max_poses = max(row["poses"] for row in sequence_rows)
        covered_rows = [row for row in sequence_rows if row["poses"] == max_poses]
        best_ate = min(covered_rows, key=lambda row: row["ate_rmse_m"])
        best_rpe = min(covered_rows, key=lambda row: row["rpe_rmse_m"])

        sequence_root = args.best_root / sequence
        shutil.copytree(best_ate["run_dir"], sequence_root / "best_ate", dirs_exist_ok=True)
        shutil.copytree(best_rpe["run_dir"], sequence_root / "best_rpe", dirs_exist_ok=True)
        best_records.append((sequence, best_ate, best_rpe))

    csv_path = args.best_root / "all_runs.csv"
    fieldnames = [
        "sequence", "run", "expected_poses", "poses", "coverage",
        "ate_rmse_m", "rpe_rmse_m", "rpe_point_distance_rmse_m",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in fieldnames})

    markdown = [
        "# Best evo results",
        "",
        "Selection rule: maximize valid trajectory coverage first, then minimize RMSE.",
        "Metrics are read from input archives; alignment, units, and RPE settings must be consistent across runs.",
        "",
        "| Sequence | Best ATE run | Poses | Coverage | ATE RMSE (m) | RPE RMSE (m) | Best RPE run | Best RPE RMSE (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for sequence, best_ate, best_rpe in best_records:
        markdown.append(
            f"| {sequence} | {best_ate['run']} | {best_ate['poses']}/{best_ate['expected_poses']} | "
            f"{best_ate['coverage']:.2%} | {best_ate['ate_rmse_m']:.6f} | "
            f"{best_ate['rpe_rmse_m']:.6f} | {best_rpe['run']} | {best_rpe['rpe_rmse_m']:.6f} |"
        )
    (args.best_root / "README.md").write_text("\n".join(markdown) + "\n", encoding="utf-8")

    print(f"all_runs={csv_path}")
    print(f"summary={args.best_root / 'README.md'}")
    for sequence, best_ate, best_rpe in best_records:
        print(
            f"{sequence}: best_ate={best_ate['run']} poses={best_ate['poses']}/{best_ate['expected_poses']} "
            f"ate={best_ate['ate_rmse_m']:.6f} rpe={best_ate['rpe_rmse_m']:.6f}; "
            f"best_rpe={best_rpe['run']} rpe={best_rpe['rpe_rmse_m']:.6f}"
        )


if __name__ == "__main__":
    main()
