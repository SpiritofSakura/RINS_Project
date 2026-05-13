"""
Evaluate a trained Anomalib model and save visualisations.

Usage:
    .venv/bin/python evaluate.py \
        --checkpoint results/patchcore/damaged_0/Patchcore/v0/weights/lightning/model.ckpt \
        --model patchcore \
        --damage-type damaged_0 \
        [--image-size 256] [--threshold 0.5] [--save-dir eval_output]

Outputs:
  - Console: mean pixel IoU per image
  - eval_output/<damage_type>/  side-by-side PNGs:
      [input | anomaly heatmap | predicted mask (red) | GT mask (green)]
"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from anomalib.data import Folder
from anomalib.engine import Engine
from anomalib.models import Fastflow, Padim, Patchcore
from torchvision.transforms import v2 as T

DATA_ROOT = Path(__file__).parent / "data" / "anomalib"
DAMAGE_TYPES = ["damaged_0", "damaged_1", "damaged_3"]

MODELS = {
    "patchcore": Patchcore,
    "padim": Padim,
    "fastflow": Fastflow,
}


def build_datamodule(damage_type: str, image_size: int) -> Folder:
    augmentations = T.Compose([T.Resize((image_size, image_size))])
    return Folder(
        name=f"rins_{damage_type}",
        root=DATA_ROOT,
        normal_dir="train/good",
        abnormal_dir=f"test/{damage_type}",
        mask_dir=f"ground_truth/{damage_type}",
        augmentations=augmentations,
        eval_batch_size=1,
        num_workers=2,
    )


def overlay_mask(image: np.ndarray, mask: np.ndarray, color=(0, 0, 255), alpha=0.5) -> np.ndarray:
    out = image.copy()
    where = mask.astype(bool)
    out[where] = ((1 - alpha) * out[where] + alpha * np.array(color)).clip(0, 255).astype(np.uint8)
    return out


def pixel_iou(pred: np.ndarray, gt: np.ndarray) -> float:
    intersection = (pred & gt).sum()
    union = (pred | gt).sum()
    return float(intersection) / float(union + 1e-8)


def save_vis(image_t, amap_t, gt_mask_t, pred_mask_t, out_path: Path):
    # Convert tensors to numpy (CHW → HWC, denormalise)
    img = image_t.permute(1, 2, 0).cpu().numpy()
    img = ((img * 0.5 + 0.5) * 255).clip(0, 255).astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    amap = amap_t.squeeze().cpu().numpy()
    amap_norm = ((amap - amap.min()) / (amap.max() - amap.min() + 1e-8) * 255).astype(np.uint8)
    heatmap = cv2.applyColorMap(amap_norm, cv2.COLORMAP_JET)

    gt   = gt_mask_t.squeeze().cpu().numpy().astype(np.uint8)
    pred = pred_mask_t.squeeze().cpu().numpy().astype(np.uint8)

    pred_vis = overlay_mask(img, pred, color=(0, 0, 255), alpha=0.5)
    gt_vis   = overlay_mask(img, gt,   color=(0, 200, 0), alpha=0.5)

    h, w = img.shape[:2]
    panel = np.concatenate([img, heatmap, pred_vis, gt_vis], axis=1)
    for i, label in enumerate(["Input", "Anomaly map", "Pred (blue)", "GT (green)"]):
        cv2.putText(panel, label, (i * w + 5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), panel)


def evaluate(checkpoint: str, model_name: str, damage_type: str,
             image_size: int, threshold: float, save_dir: Path):
    datamodule = build_datamodule(damage_type, image_size)
    model = MODELS[model_name]()

    engine = Engine(accelerator="auto", devices=1)
    predictions = engine.predict(
        model=model,
        datamodule=datamodule,
        ckpt_path=checkpoint,
        return_predictions=True,
    )

    ious = []
    out_dir = save_dir / damage_type
    out_dir.mkdir(parents=True, exist_ok=True)

    for batch in predictions:
        # batch is an ImageBatch; iterate items for per-image processing
        for item in batch:
            if item.anomaly_map is None or item.gt_mask is None:
                continue

            amap  = item.anomaly_map.squeeze()
            gt    = item.gt_mask.squeeze()
            pred  = (amap > threshold).to(gt.dtype)

            ious.append(pixel_iou(pred.cpu().numpy().astype(bool),
                                  gt.cpu().numpy().astype(bool)))

            stem = Path(item.image_path).stem
            save_vis(item.image, item.anomaly_map, item.gt_mask, pred,
                     out_dir / f"{stem}.png")

    mean_iou = float(np.mean(ious)) if ious else 0.0
    print(f"\n{damage_type}:")
    print(f"  Images evaluated : {len(ious)}")
    print(f"  Mean pixel IoU   : {mean_iou:.4f}  (threshold={threshold})")
    print(f"  Visualisations   : {out_dir}/")
    return mean_iou


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True,
                        help="Path to the .ckpt file produced by train.py")
    parser.add_argument("--model", choices=list(MODELS), default="patchcore")
    parser.add_argument("--damage-type", choices=DAMAGE_TYPES, default="damaged_0")
    parser.add_argument("--image-size", type=int, default=256)
    parser.add_argument("--threshold", type=float, default=0.5,
                        help="Anomaly score threshold for binary mask (default 0.5)")
    parser.add_argument("--save-dir", type=Path,
                        default=Path(__file__).parent / "eval_output")
    args = parser.parse_args()

    evaluate(args.checkpoint, args.model, args.damage_type,
             args.image_size, args.threshold, args.save_dir)


if __name__ == "__main__":
    main()
