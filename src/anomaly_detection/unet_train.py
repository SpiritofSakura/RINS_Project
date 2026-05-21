"""
Train a U-Net segmentation model for crack detection.

Usage:
    .venv/bin/python unet_train.py [--epochs 50] [--image-size 512] [--batch-size 8]

Checkpoint saved to results/unet/best_model.pth (best validation Dice).
"""

import argparse
import random
from pathlib import Path

import albumentations as A
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from albumentations.pytorch import ToTensorV2
from torch.utils.data import DataLoader, Dataset
import segmentation_models_pytorch as smp

DATA_DIR = Path(__file__).parent / "data"
RESULTS_DIR = Path(__file__).parent / "results" / "unet"


def _soft_erode(img):
    p1 = -F.max_pool2d(-img, (3, 1), (1, 1), (1, 0))
    p2 = -F.max_pool2d(-img, (1, 3), (1, 1), (0, 1))
    return torch.min(p1, p2)


def _soft_skel(img, iters=3):
    skel = F.relu(img - _soft_erode(img))
    for _ in range(iters):
        img = _soft_erode(img)
        skel = skel + F.relu(img - _soft_erode(img))
    return skel


class ClDiceLoss(nn.Module):
    """Centerline Dice — penalises missing crack tips/skeletons."""
    def __init__(self, iters=3, smooth=1.0):
        super().__init__()
        self.iters = iters
        self.smooth = smooth

    def forward(self, logits, targets):
        pred = torch.sigmoid(logits)
        skel_pred = _soft_skel(pred,    self.iters)
        skel_true = _soft_skel(targets, self.iters)
        s = self.smooth
        tprec = ((skel_pred * targets).sum() + s) / (skel_pred.sum() + s)
        tsens = ((skel_true * pred).sum()    + s) / (skel_true.sum() + s)
        return 1.0 - 2.0 * tprec * tsens / (tprec + tsens + 1e-8)


class CrackDataset(Dataset):
    def __init__(self, image_paths, mask_paths, transform=None):
        self.image_paths = image_paths
        self.mask_paths = mask_paths
        self.transform = transform

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        image = cv2.cvtColor(cv2.imread(str(self.image_paths[idx])), cv2.COLOR_BGR2RGB)
        mask = cv2.imread(str(self.mask_paths[idx]), cv2.IMREAD_GRAYSCALE)
        mask = (mask > 127).astype(np.float32)

        if self.transform:
            out = self.transform(image=image, mask=mask)
            image = out["image"]
            mask = out["mask"].unsqueeze(0)

        return image, mask


def make_transforms(image_size: int, train: bool):
    base = [
        A.Resize(image_size, image_size),
        A.CLAHE(clip_limit=4.0, tile_grid_size=(8, 8), p=1.0),
    ]
    if train:
        base += [
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.5),
            A.RandomRotate90(p=0.5),
            A.ShiftScaleRotate(shift_limit=0.05, scale_limit=0.1, rotate_limit=45, p=0.5),
            A.ElasticTransform(alpha=120, sigma=6, p=0.3),
            A.GridDistortion(num_steps=5, distort_limit=0.3, p=0.3),
            A.RandomBrightnessContrast(p=0.4),
            A.RandomGamma(gamma_limit=(70, 130), p=0.3),
            A.GaussNoise(p=0.2),
        ]
    base += [
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ]
    return A.Compose(base)


def compute_pos_weight(mask_paths: list) -> float:
    """Ratio of negative to positive pixels across all masks."""
    total_pos = 0
    total_neg = 0
    for p in mask_paths:
        m = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if m is None:
            continue
        pos = int((m > 127).sum())
        total_pos += pos
        total_neg += m.size - pos
    weight = min(total_neg / max(total_pos, 1), 10.0)  # cap to avoid over-correction with Tversky
    print(f"pos_weight={weight:.1f}  (neg={total_neg}, pos={total_pos})")
    return weight


def stratified_split(damaged_pairs: list, normal_pairs: list, val_split: float, seed: int = 42):
    """Split each group separately so the damaged/okay ratio is preserved in both splits."""
    rng = random.Random(seed)

    def split(pairs):
        pairs = list(pairs)
        rng.shuffle(pairs)
        n = int(len(pairs) * val_split)
        return pairs[:n], pairs[n:]

    val_d, train_d = split(damaged_pairs)
    val_n, train_n = split(normal_pairs)
    return train_d + train_n, val_d + val_n


def dice_score(logits, targets, threshold=0.5, eps=1e-6):
    preds = (torch.sigmoid(logits) > threshold).float()
    intersection = (preds * targets).sum()
    return (2 * intersection + eps) / (preds.sum() + targets.sum() + eps)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-size", type=int, default=512)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--val-split", type=float, default=0.2)
    parser.add_argument("--early-stopping", type=int, default=10,
                        help="Stop if val_dice does not improve for this many epochs (0=off)")
    parser.add_argument("--no-normal", action="store_true",
                        help="Exclude okay_* images from training")
    args = parser.parse_args()

    image_dir = DATA_DIR / "train" / "images"
    mask_dir  = DATA_DIR / "train" / "gt"

    damaged_pairs = [(p, mask_dir / p.name) for p in sorted(image_dir.glob("damaged_*.png"))
                     if (mask_dir / p.name).exists()]
    normal_pairs  = [] if args.no_normal else [
        (p, mask_dir / p.name) for p in sorted(image_dir.glob("okay_*.png"))
        if (mask_dir / p.name).exists()
    ]

    train_pairs, val_pairs = stratified_split(damaged_pairs, normal_pairs, args.val_split)
    print(f"Train: {len(train_pairs)}  Val: {len(val_pairs)}  "
          f"(damaged={len(damaged_pairs)}, normal={len(normal_pairs)})")

    train_imgs, train_msks = zip(*train_pairs)
    val_imgs,   val_msks   = zip(*val_pairs)

    pos_weight = compute_pos_weight(list(train_msks))

    train_ds = CrackDataset(train_imgs, train_msks, make_transforms(args.image_size, train=True))
    val_ds   = CrackDataset(val_imgs,   val_msks,   make_transforms(args.image_size, train=False))

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,  num_workers=4, pin_memory=True)
    val_loader   = DataLoader(val_ds,   batch_size=args.batch_size, shuffle=False, num_workers=4, pin_memory=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    model = smp.Unet(
        encoder_name="resnet34",
        encoder_weights="imagenet",
        in_channels=3,
        classes=1,
    ).to(device)

    bce     = nn.BCEWithLogitsLoss(pos_weight=torch.tensor([pos_weight]).to(device))
    tversky = smp.losses.TverskyLoss(mode="binary", alpha=0.2, beta=0.8)
    cldice  = ClDiceLoss(iters=3)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=50)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    best_val_dice  = 0.0
    epochs_no_improve = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for imgs, msks in train_loader:
            imgs, msks = imgs.to(device), msks.to(device)
            optimizer.zero_grad()
            preds = model(imgs)
            loss = 0.35 * bce(preds, msks) + 0.35 * tversky(preds, msks) + 0.3 * cldice(preds, msks)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        model.eval()
        val_dice = 0.0
        with torch.no_grad():
            for imgs, msks in val_loader:
                imgs, msks = imgs.to(device), msks.to(device)
                val_dice += dice_score(model(imgs), msks).item()

        train_loss /= len(train_loader)
        val_dice   /= len(val_loader)
        scheduler.step()

        print(f"Epoch {epoch:3d}/{args.epochs}  loss={train_loss:.4f}  val_dice={val_dice:.4f}")

        if val_dice > best_val_dice:
            best_val_dice = val_dice
            epochs_no_improve = 0
            torch.save(model.state_dict(), RESULTS_DIR / "best_model.pth")
            print(f"           → best saved (dice={val_dice:.4f})")
        else:
            epochs_no_improve += 1
            if args.early_stopping > 0 and epochs_no_improve >= args.early_stopping:
                print(f"\nEarly stopping: no improvement for {args.early_stopping} epochs.")
                break

    print(f"\nDone. Best val Dice: {best_val_dice:.4f}")
    print(f"Checkpoint: {RESULTS_DIR / 'best_model.pth'}")


if __name__ == "__main__":
    main()
