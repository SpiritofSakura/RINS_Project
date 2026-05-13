"""
Reorganise the RINS dataset into the folder structure expected by anomalib's
Folder datamodule (as used by train.py / evaluate.py).

Source layout (data/):
  train/images/  -- okay_*  (normal) and damaged_* (anomalous) images
  train/gt/      -- masks for every train image
  test/images/   -- damaged_* images only
  test/gt/       -- masks for every test image

Target layout (data/anomalib/):
  train/good/              ← okay_* images from train/images
  test/<damage_type>/      ← damaged images (test set first, then train set)
  ground_truth/<damage_type>/  ← matching GT masks

Uses symlinks so no data is duplicated.  Re-running is safe — existing links
are silently replaced.

Usage:
    python prepare_data.py [--data-dir data] [--out-dir data/anomalib]
"""

import argparse
import os
from pathlib import Path

DAMAGE_TYPES = ["damaged_0", "damaged_1", "damaged_2", "damaged_3"]


def symlink(src: Path, dst: Path):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.is_symlink() or dst.exists():
        dst.unlink()
    dst.symlink_to(src.resolve())


def damage_type(filename: str) -> str | None:
    """Return e.g. 'damaged_0' from 'damaged_0_0001_ls3_camera0.png', or None."""
    for dt in DAMAGE_TYPES:
        if filename.startswith(dt + "_"):
            return dt
    return None


def prepare(data_dir: Path, out_dir: Path):
    train_images = data_dir / "train" / "images"
    train_gt = data_dir / "train" / "gt"
    test_images = data_dir / "test" / "images"
    test_gt = data_dir / "test" / "gt"

    counts = {"good": 0, "test": 0, "gt": 0}

    # 1. okay_* train images → train/good/
    good_dir = out_dir / "train" / "good"
    for img in sorted(train_images.glob("okay_*.png")):
        symlink(img, good_dir / img.name)
        counts["good"] += 1

    # 2. test/images → test/<damage_type>/  and  test/gt → ground_truth/<damage_type>/
    for img in sorted(test_images.glob("*.png")):
        dt = damage_type(img.name)
        if dt is None:
            print(f"  [skip] unrecognised prefix: {img.name}")
            continue
        symlink(img, out_dir / "test" / dt / img.name)
        counts["test"] += 1

        mask = test_gt / img.name
        if mask.exists():
            symlink(mask, out_dir / "ground_truth" / dt / img.name)
            counts["gt"] += 1

    # 3. damaged_* train images → also add to test/<damage_type>/
    #    (anomaly detection trains on normal only; these are extra anomalous samples)
    for img in sorted(train_images.glob("damaged_*.png")):
        dt = damage_type(img.name)
        if dt is None:
            continue
        # Prefix with "train_" to avoid filename collisions with test set
        dst_name = "train_" + img.name
        symlink(img, out_dir / "test" / dt / dst_name)
        counts["test"] += 1

        mask = train_gt / img.name
        if mask.exists():
            symlink(mask, out_dir / "ground_truth" / dt / dst_name)
            counts["gt"] += 1

    print(f"Done.")
    print(f"  train/good/        : {counts['good']} symlinks")
    print(f"  test/<type>/       : {counts['test']} symlinks")
    print(f"  ground_truth/<type>: {counts['gt']} symlinks")
    print(f"  Output dir         : {out_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path,
                        default=Path(__file__).parent / "data",
                        help="Root of the source data (default: data/)")
    parser.add_argument("--out-dir", type=Path,
                        default=Path(__file__).parent / "data" / "anomalib",
                        help="Output root for the anomalib layout (default: data/anomalib/)")
    args = parser.parse_args()

    if not args.data_dir.exists():
        raise SystemExit(f"data-dir not found: {args.data_dir}")

    prepare(args.data_dir, args.out_dir)


if __name__ == "__main__":
    main()
