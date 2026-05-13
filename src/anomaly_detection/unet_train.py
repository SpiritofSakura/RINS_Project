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
from albumentations.pytorch import ToTensorV2
from torch.utils.data import DataLoader, Dataset
import segmentation_models_pytorch as smp

DATA_DIR = Path(__file__).parent / "data"
RESULTS_DIR = Path(__file__).parent / "results" / "unet"


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
        # Boost local contrast so cracks are visible even in dark/flat images
        A.CLAHE(clip_limit=4.0, tile_grid_size=(8, 8), p=1.0),
    ]
    if train:
        base += [
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.5),
            A.RandomRotate90(p=0.5),
            A.RandomBrightnessContrast(p=0.4),
            A.GaussNoise(p=0.2),
        ]
    base += [
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ]
    return A.Compose(base)


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
    parser.add_argument("--no-normal", action="store_true",
                        help="Exclude okay_* images from training")
    args = parser.parse_args()

    image_dir = DATA_DIR / "train" / "images"
    mask_dir = DATA_DIR / "train" / "gt"

    images = sorted(image_dir.glob("damaged_*.png"))
    if not args.no_normal:
        images += sorted(image_dir.glob("okay_*.png"))

    pairs = [(p, mask_dir / p.name) for p in images if (mask_dir / p.name).exists()]
    print(f"Training pairs: {len(pairs)}")

    random.seed(42)
    random.shuffle(pairs)
    n_val = int(len(pairs) * args.val_split)
    val_pairs, train_pairs = pairs[:n_val], pairs[n_val:]

    train_imgs, train_msks = zip(*train_pairs)
    val_imgs, val_msks = zip(*val_pairs)

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

    bce    = nn.BCEWithLogitsLoss(pos_weight=torch.tensor([10.0]).to(device))
    tversky = smp.losses.TverskyLoss(mode="binary", alpha=0.2, beta=0.8)
    # alpha=FP penalty, beta=FN penalty — high beta forces model to find thin/faint crack ends

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    best_val_dice = 0.0

    for epoch in range(1, args.epochs + 1):
        model.train()
        train_loss = 0.0
        for imgs, msks in train_loader:
            imgs, msks = imgs.to(device), msks.to(device)
            optimizer.zero_grad()
            preds = model(imgs)
            loss = 0.5 * bce(preds, msks) + 0.5 * tversky(preds, msks)
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
            torch.save(model.state_dict(), RESULTS_DIR / "best_model.pth")
            print(f"           → best saved (dice={val_dice:.4f})")

    print(f"\nDone. Best val Dice: {best_val_dice:.4f}")
    print(f"Checkpoint: {RESULTS_DIR / 'best_model.pth'}")


if __name__ == "__main__":
    main()
