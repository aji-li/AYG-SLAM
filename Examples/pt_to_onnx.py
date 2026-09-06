#!/usr/bin/env python3
"""Export a user-supplied YOLO model; adapted from AYG-SLAM's export helper."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, help="Path to your trained .pt model")
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error("model file does not exist")
    from ultralytics import YOLO
    model = YOLO(str(args.model))
    model.export(format="onnx")


if __name__ == "__main__":
    main()
