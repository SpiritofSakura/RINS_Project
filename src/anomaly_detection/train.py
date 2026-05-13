"""
Train a surface anomaly detection model on the RINS dataset.

Usage:
    .venv/bin/python train.py [--model patchcore|padim|fastflow] [--damage-types damaged_0 ...]

Supports:
  - patchcore  : k-NN on deep features, strong baseline, 1 epoch
  - padim      : Gaussian distribution on patch features (PCA-based), 1 epoch
  - fastflow   : normalising flow on features, needs more epochs

Results and checkpoints are saved under ./results/<model>/<damage_type>/.
"""

import argparse
from pathlib import Path

from anomalib.data import Folder
from anomalib.engine import Engine
from anomalib.models import Fastflow, Padim, Patchcore
from torchvision.transforms import v2 as T

DATA_ROOT = Path(__file__).parent / "data" / "anomalib"
RESULTS_DIR = Path(__file__).parent / "results"

DAMAGE_TYPES = ["damaged_0", "damaged_1", "damaged_3"]

MODELS = {
    "patchcore": Patchcore,
    "padim": Padim,
    "fastflow": Fastflow,
}

# Memory-based models only need 1 pass; flow models benefit from more.
EPOCHS = {
    "patchcore": 1,
    "padim": 1,
    "fastflow": 30,
}


def build_datamodule(damage_type: str, image_size: int) -> Folder:
    augmentations = T.Compose([T.Resize((image_size, image_size))])
    return Folder(
        name=f"rins_{damage_type}",
        root=DATA_ROOT,
        normal_dir="train/good",
        abnormal_dir=f"test/{damage_type}",
        mask_dir=f"ground_truth/{damage_type}",
        normal_split_ratio=0.2,
        augmentations=augmentations,
        train_batch_size=32,
        eval_batch_size=32,
        num_workers=4,
    )


def train_one(model_name: str, damage_type: str, image_size: int) -> dict:
    print(f"\n{'='*60}")
    print(f"  Model: {model_name}  |  Damage type: {damage_type}")
    print(f"{'='*60}")

    model = MODELS[model_name]()
    datamodule = build_datamodule(damage_type, image_size)
    output_dir = RESULTS_DIR / model_name / damage_type

    engine = Engine(
        default_root_dir=str(output_dir),
        max_epochs=EPOCHS[model_name],
        accelerator="auto",
        devices=1,
    )

    engine.fit(model=model, datamodule=datamodule)
    results = engine.test(model=model, datamodule=datamodule)

    metrics = results[0] if results else {}
    print(f"\nResults for {model_name} / {damage_type}:")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}" if isinstance(v, float) else f"  {k}: {v}")

    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=list(MODELS), default="patchcore")
    parser.add_argument(
        "--damage-types", nargs="+", default=DAMAGE_TYPES, choices=DAMAGE_TYPES,
        help="Which test damage categories to train/evaluate against",
    )
    parser.add_argument(
        "--image-size", type=int, default=256,
        help="Square image size fed to the model (default: 256)",
    )
    args = parser.parse_args()

    all_results = {}
    for dtype in args.damage_types:
        all_results[dtype] = train_one(args.model, dtype, args.image_size)

    print(f"\n{'='*60}")
    print("  Summary")
    print(f"{'='*60}")
    for dtype, res in all_results.items():
        # Anomalib v2 logs metrics with various key names depending on model
        pixel = next((v for k, v in res.items() if "pixel" in k.lower() and "auroc" in k.lower()), "N/A")
        image = next((v for k, v in res.items() if "image" in k.lower() and "auroc" in k.lower()), "N/A")
        print(f"  {dtype:12s}  pixel-AUROC={pixel}  image-AUROC={image}")


if __name__ == "__main__":
    main()
