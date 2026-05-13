"""
Evaluate trained U-Net and save visualisations.

Usage:
    .venv/bin/python unet_evaluate.py [--checkpoint results/unet/best_model.pth]
                                      [--threshold 0.5] [--image-size 512]

Outputs per-damage-type IoU and side-by-side PNGs in eval_output/unet/<damage_type>/.
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


def make_transform(image_size):
    return A.Compose([
        A.Resize(image_size, image_size),
        A.CLAHE(clip_limit=4.0, tile_grid_size=(8, 8), p=1.0),
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ])


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


def evaluate(checkpoint: Path, image_size: int, threshold: float, save_dir: Path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    model = smp.Unet(encoder_name="resnet34", encoder_weights=None, in_channels=3, classes=1)
    model.load_state_dict(torch.load(checkpoint, map_location=device))
    model.to(device).eval()

    transform = make_transform(image_size)

    all_ious = []
    for damage_type in DAMAGE_TYPES:
        img_dir  = DATA_DIR / "test" / "images"
        mask_dir = DATA_DIR / "test" / "gt"
        out_dir  = save_dir / damage_type

        images = sorted(img_dir.glob(f"{damage_type}_*.png"))
        if not images:
            print(f"No test images found for {damage_type}, skipping.")
            continue

        ious = []
        for img_path in images:
            mask_path = mask_dir / img_path.name
            if not mask_path.exists():
                continue

            img_bgr = cv2.imread(str(img_path))
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            gt = (cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) > 127).astype(np.uint8)
            gt_resized = cv2.resize(gt, (image_size, image_size), interpolation=cv2.INTER_NEAREST)

            tensor = transform(image=img_rgb)["image"].unsqueeze(0).to(device)
            with torch.no_grad():
                logit = model(tensor)
            pred = (torch.sigmoid(logit).squeeze().cpu().numpy() > threshold).astype(np.uint8)

            iou = pixel_iou(pred.astype(bool), gt_resized.astype(bool))
            ious.append(iou)

            save_panel(img_path, pred, gt_resized, out_dir / img_path.name)

        mean_iou = float(np.mean(ious)) if ious else 0.0
        all_ious.extend(ious)
        print(f"  {damage_type:12s}  images={len(ious):3d}  mean IoU={mean_iou:.4f}")

    print(f"\n  Overall mean IoU: {float(np.mean(all_ious)):.4f}  (threshold={threshold})")
    print(f"  Visualisations:   {save_dir}/")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path,
                        default=Path(__file__).parent / "results" / "unet" / "best_model.pth")
    parser.add_argument("--image-size", type=int, default=512)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--save-dir", type=Path,
                        default=Path(__file__).parent / "eval_output" / "unet")
    args = parser.parse_args()

    if not args.checkpoint.exists():
        raise SystemExit(f"Checkpoint not found: {args.checkpoint}")

    evaluate(args.checkpoint, args.image_size, args.threshold, args.save_dir)


if __name__ == "__main__":
    main()
