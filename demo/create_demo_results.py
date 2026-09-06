#!/usr/bin/env python3
"""Create synthetic input to exercise the public evaluator, not SLAM results."""
import argparse
import json
import zipfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; choose a new directory")
    args.output.mkdir(parents=True)
    (args.output / "expected_poses.json").write_text(
        json.dumps({"synthetic_sequence": 4}) + "\n", encoding="utf-8"
    )
    # An incomplete run deliberately has lower error: coverage takes priority.
    runs = (("run_1", 4, 0.03, 0.02), ("run_2", 4, 0.04, 0.01),
            ("run_3", 2, 0.001, 0.001))
    for name, poses, ate, rpe in runs:
        run = args.output / "runs" / "synthetic_sequence" / name
        run.mkdir(parents=True)
        (run / "CameraTrajectory.txt").write_text(
            "# Synthetic fixture, not a measured SLAM trajectory\n" +
            "".join(f"{i}.0 {i * 0.1:.1f} 0 0 0 0 0 1\n" for i in range(poses)),
            encoding="utf-8",
        )
        for filename, rmse in (("ape.zip", ate), ("rpe.zip", rpe)):
            with zipfile.ZipFile(run / filename, "w") as archive:
                archive.writestr("stats.json", json.dumps({"rmse": rmse}))
    print(f"Synthetic fixtures created in {args.output}")


if __name__ == "__main__":
    main()
