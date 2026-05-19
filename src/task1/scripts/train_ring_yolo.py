#!/usr/bin/env python3
"""
Train YOLOv8-nano to detect coloured rings.

Usage:
    python3 train_ring_yolo.py

Expects ~/ring_dataset/ to exist (run generate_ring_data.py first).
Model saved to ~/ring_yolo/weights/best.pt
"""

from pathlib import Path
from ultralytics import YOLO

WORKSPACE = Path(__file__).resolve().parents[3]
DATASET_YAML = str(Path.home() / "ring_dataset" / "ring_dataset.yaml")
OUTPUT_DIR   = str(Path.home() / "ring_yolo")
LOCAL_BASE_MODEL = WORKSPACE / "yolov8n.pt"
BASE_MODEL = str(LOCAL_BASE_MODEL) if LOCAL_BASE_MODEL.is_file() else "yolov8n.pt"

EPOCHS    = 100
IMG_SIZE  = 416
BATCH     = 16
PATIENCE  = 20   # early stop if no improvement for 20 epochs


def main():
    if not Path(DATASET_YAML).is_file():
        raise FileNotFoundError(
            f"Dataset not found: {DATASET_YAML}. Run generate_ring_data.py first."
        )

    print(f"Base model: {BASE_MODEL}")
    print(f"Dataset:    {DATASET_YAML}")
    print(f"Output:     {OUTPUT_DIR}/ring_det")

    model = YOLO(BASE_MODEL)

    results = model.train(
        data      = DATASET_YAML,
        epochs    = EPOCHS,
        imgsz     = IMG_SIZE,
        batch     = BATCH,
        patience  = PATIENCE,
        device    = 0,            # GPU 0
        project   = OUTPUT_DIR,
        name      = "ring_det",
        exist_ok  = True,
        # Augmentation — keep colour jitter mild so ring colours stay recognisable
        hsv_h     = 0.01,
        hsv_s     = 0.3,
        hsv_v     = 0.3,
        fliplr    = 0.5,
        flipud    = 0.2,
        scale     = 0.5,
        translate = 0.1,
        mosaic    = 1.0,
        degrees   = 30,
        verbose   = True,
    )

    best = Path(OUTPUT_DIR) / "ring_det" / "weights" / "best.pt"
    print(f"\nTraining done. Best model: {best}")
    print("Use this with: ros2 launch task1 rings_test.launch.py ring_model:=" + str(best))


if __name__ == "__main__":
    main()
