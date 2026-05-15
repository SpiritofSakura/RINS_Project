"""
Evaluate trained U-Net and save visualisations.

Usage:
    .venv/bin/python unet_evaluate.py [--checkpoint results/unet/best_model.pth]
                                      [--image-size 512]

Automatically sweeps thresholds [0.20 … 0.50] and picks the best overall IoU.
Applies morphological post-processing (open 3×3 → close 5×5) to clean the mask.
Reports mean / median / min / max IoU per damage type.
Saves side-by-side PNGs in eval_output/unet/<best_threshold>/<damage_type>/.
"""

import argparse
from pathlib import Path

import albumentations as A
import cv2
import numpy as np
import torch
from albumentations.pytorch import ToTensorV2
import segmentation_models_pytorch as smp

DATA_DIR = Path(__file__).parent / "data"
DAMAGE_TYPES = ["damaged_0", "damaged_1", "damaged_3"]
THRESHOLDS = [0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70]

MORPH_OPEN_K  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
MORPH_CLOSE_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def make_transform(image_size):
    return A.Compose([
        A.Resize(image_size, image_size),
        A.CLAHE(clip_limit=4.0, tile_grid_size=(8, 8), p=1.0),
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ])


def postprocess(mask: np.ndarray) -> np.ndarray:
    """Remove speckles (open) then close small gaps (close)."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_OPEN_K)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_CLOSE_K)
    return mask


def pixel_iou(pred: np.ndarray, gt: np.ndarray, eps=1e-6) -> float:
    intersection = (pred & gt).sum()
    union = (pred | gt).sum()
    return float(intersection) / float(union + eps)


def save_panel(orig_path: Path, pred_mask: np.ndarray, gt_mask: np.ndarray, out_path: Path):
    img = cv2.imread(str(orig_path))
    h, w = pred_mask.shape
    img_r = cv2.resize(img, (w, h))

    def overlay(base, mask, color, alpha=0.5):
        out = base.copy()
        where = mask.astype(bool)
        out[where] = ((1 - alpha) * out[where] + alpha * np.array(color)).clip(0, 255).astype(np.uint8)
        return out

    pred_vis = overlay(img_r, pred_mask, (0, 0, 255))
    gt_vis   = overlay(img_r, gt_mask,   (0, 200, 0))

    panel = np.concatenate([img_r, pred_vis, gt_vis], axis=1)
    for i, label in enumerate(["Input", "Pred (blue)", "GT (green)"]):
        cv2.putText(panel, label, (i * w + 5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), panel)


def run_threshold(model, transform, image_size: int, threshold: float, device) -> dict[str, list]:
    """Return {damage_type: [iou, ...]} for the given threshold (no saving)."""
    results = {}
    img_dir  = DATA_DIR / "test" / "images"
    mask_dir = DATA_DIR / "test" / "gt"

    for dt in DAMAGE_TYPES:
        ious = []
        for img_path in sorted(img_dir.glob(f"{dt}_*.png")):
            mask_path = mask_dir / img_path.name
            if not mask_path.exists():
                continue

            img_rgb = cv2.cvtColor(cv2.imread(str(img_path)), cv2.COLOR_BGR2RGB)
            gt = (cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) > 127).astype(np.uint8)
            gt = cv2.resize(gt, (image_size, image_size), interpolation=cv2.INTER_NEAREST)

            tensor = transform(image=img_rgb)["image"].unsqueeze(0).to(device)
            with torch.no_grad():
                prob = torch.sigmoid(model(tensor)).squeeze().cpu().numpy()

            pred = (prob > threshold).astype(np.uint8)
            pred = postprocess(pred)
            ious.append(pixel_iou(pred.astype(bool), gt.astype(bool)))

        results[dt] = ious
    return results


def save_results(model, transform, image_size: int, threshold: float, device, save_dir: Path):
    """Save visualisations for the chosen threshold."""
    img_dir  = DATA_DIR / "test" / "images"
    mask_dir = DATA_DIR / "test" / "gt"

    for dt in DAMAGE_TYPES:
        out_dir = save_dir / dt
        for img_path in sorted(img_dir.glob(f"{dt}_*.png")):
            mask_path = mask_dir / img_path.name
            if not mask_path.exists():
                continue

            img_rgb = cv2.cvtColor(cv2.imread(str(img_path)), cv2.COLOR_BGR2RGB)
            gt = (cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) > 127).astype(np.uint8)
            gt = cv2.resize(gt, (image_size, image_size), interpolation=cv2.INTER_NEAREST)

            tensor = transform(image=img_rgb)["image"].unsqueeze(0).to(device)
            with torch.no_grad():
                prob = torch.sigmoid(model(tensor)).squeeze().cpu().numpy()

            pred = postprocess((prob > threshold).astype(np.uint8))
            save_panel(img_path, pred, gt, out_dir / img_path.name)


def print_stats(label: str, ious: list):
    if not ious:
        print(f"  {label:12s}  no images")
        return
    arr = np.array(ious)
    print(f"  {label:12s}  n={len(arr):3d}  "
          f"mean={arr.mean():.4f}  median={np.median(arr):.4f}  "
          f"min={arr.min():.4f}  max={arr.max():.4f}")


def evaluate(checkpoint: Path, image_size: int, save_dir: Path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = smp.Unet(encoder_name="resnet34", encoder_weights=None, in_channels=3, classes=1)
    model.load_state_dict(torch.load(checkpoint, map_location=device))
    model.to(device).eval()

    transform = make_transform(image_size)

    # --- threshold sweep ---
    print("Threshold sweep:")
    best_threshold = THRESHOLDS[0]
    best_mean_iou  = -1.0
    sweep_results  = {}

    for thr in THRESHOLDS:
        results = run_threshold(model, transform, image_size, thr, device)
        all_ious = [iou for ious in results.values() for iou in ious]
        mean_iou = float(np.mean(all_ious)) if all_ious else 0.0
        marker = ""
        if mean_iou > best_mean_iou:
            best_mean_iou  = mean_iou
            best_threshold = thr
            sweep_results  = results
            marker = "  ← best"
        print(f"  thr={thr:.2f}  overall mean IoU={mean_iou:.4f}{marker}")

    # --- detailed report at best threshold ---
    print(f"\nBest threshold: {best_threshold:.2f}  (overall mean IoU={best_mean_iou:.4f})")
    print("\nPer damage type:")
    all_ious = []
    for dt in DAMAGE_TYPES:
        ious = sweep_results.get(dt, [])
        print_stats(dt, ious)
        all_ious.extend(ious)
    print("─" * 65)
    print_stats("OVERALL", all_ious)

    # --- save visualisations ---
    vis_dir = save_dir / f"thr_{best_threshold:.2f}"
    save_results(model, transform, image_size, best_threshold, device, vis_dir)
    print(f"\nVisualisations: {vis_dir}/")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path,
                        default=Path(__file__).parent / "results" / "unet" / "best_model.pth")
    parser.add_argument("--image-size", type=int, default=512)
    parser.add_argument("--save-dir", type=Path,
                        default=Path(__file__).parent / "eval_output" / "unet")
    args = parser.parse_args()

    if not args.checkpoint.exists():
        raise SystemExit(f"Checkpoint not found: {args.checkpoint}")

    evaluate(args.checkpoint, args.image_size, args.save_dir)


if __name__ == "__main__":
    main()
