#!/usr/bin/env python3
"""
Synthetic ring training data generator for YOLOv8.

Generates colored annuli (hollow circles / tilted ellipses) on varied backgrounds,
plus hard-negative images with fake circles, filled discs, shadows, and flat
ring-like wall marks that must not be annotated.

Classes: 0=red  1=green  2=blue  3=black
Output:  ~/ring_dataset/{images,labels}/{train,val}/
"""

import math
import random
from pathlib import Path

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
IMG_SIZE   = 416
N_TRAIN    = 3000
N_VAL      = 600
OUT_DIR    = Path.home() / "ring_dataset"
SEED       = 42
NEGATIVE_PROB = 0.25

# BGR colors matched to sim HSV ranges in detect_rings_v2.py
RING_BGR = {
    0: [(0, 0, 200), (30, 30, 240), (0, 0, 255)],          # red
    1: [(0, 180, 0), (0, 210, 40), (20, 255, 20)],          # green
    2: [(200, 60, 0), (230, 80, 10), (255, 100, 30)],       # blue
    3: [(10, 10, 10), (25, 25, 25), (40, 40, 40)],          # black
}


# ── Background generators ─────────────────────────────────────────────────────
def _bg_solid(h, w):
    v = random.randint(60, 210)
    jitter = [random.randint(-15, 15) for _ in range(3)]
    return np.full((h, w, 3), [max(0, min(255, v + j)) for j in jitter], dtype=np.uint8)


def _bg_gradient(h, w):
    c1 = np.array([random.randint(60, 180)] * 3, dtype=np.float32)
    c2 = np.array([random.randint(60, 180)] * 3, dtype=np.float32)
    t = np.linspace(0, 1, w if random.random() < 0.5 else h, dtype=np.float32)
    bg = np.zeros((h, w, 3), dtype=np.uint8)
    if t.shape[0] == w:
        for x, alpha in enumerate(t):
            bg[:, x] = (c1 * (1 - alpha) + c2 * alpha).astype(np.uint8)
    else:
        for y, alpha in enumerate(t):
            bg[y, :] = (c1 * (1 - alpha) + c2 * alpha).astype(np.uint8)
    return bg


def _bg_tile(h, w):
    bg = np.zeros((h, w, 3), dtype=np.uint8)
    tile = random.randint(30, 70)
    base = random.randint(90, 170)
    for ty in range(0, h, tile):
        for tx in range(0, w, tile):
            v = base + random.randint(-20, 20)
            bg[ty:ty + tile, tx:tx + tile] = [v, v, v]
    return bg


def random_background(h, w):
    fn = random.choice([_bg_solid, _bg_solid, _bg_gradient, _bg_tile])
    bg = fn(h, w)
    # Subtle noise so the network generalises
    noise = np.random.randint(-8, 8, (h, w, 3), dtype=np.int16)
    return np.clip(bg.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def add_distractors(img, max_items=5):
    """Draw non-ring circular clutter. These are intentionally unlabeled."""
    h, w = img.shape[:2]
    for _ in range(random.randint(1, max_items)):
        cx = random.randint(20, w - 20)
        cy = random.randint(20, h - 20)
        rx = random.randint(8, 80)
        ry = random.randint(8, 80)
        angle = random.randint(0, 180)
        colour = random.choice([
            (random.randint(50, 210),) * 3,
            (0, 0, random.randint(90, 180)),
            (0, random.randint(90, 180), 0),
            (random.randint(90, 180), 50, 0),
            (20, 20, 20),
        ])
        kind = random.choice(["filled", "outline", "arc", "shadow"])
        if kind == "filled":
            cv2.ellipse(img, (cx, cy), (rx, ry), angle, 0, 360, colour, -1)
        elif kind == "outline":
            cv2.ellipse(img, (cx, cy), (rx, ry), angle, 0, 360, colour, random.randint(2, 6))
        elif kind == "arc":
            start = random.randint(0, 180)
            cv2.ellipse(img, (cx, cy), (rx, ry), angle, start, start + random.randint(70, 240),
                        colour, random.randint(2, 7))
        else:
            overlay = img.copy()
            cv2.ellipse(overlay, (cx, cy), (rx, ry), angle, 0, 360, (0, 0, 0), -1)
            img[:] = cv2.addWeighted(overlay, random.uniform(0.08, 0.22), img, 0.9, 0)
    return img


# ── Ring drawing ──────────────────────────────────────────────────────────────
def draw_ring(img, cx, cy, rx, ry, angle_deg, inner_ratio, class_id):
    """
    Draw a (possibly elliptical) ring using mask compositing so the hole
    shows the background correctly.
    """
    h, w = img.shape[:2]
    color = random.choice(RING_BGR[class_id])

    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.ellipse(mask, (cx, cy), (rx, ry), angle_deg, 0, 360, 255, -1)
    irx = max(1, int(rx * inner_ratio))
    iry = max(1, int(ry * inner_ratio))
    cv2.ellipse(mask, (cx, cy), (irx, iry), angle_deg, 0, 360, 0, -1)

    # Slight color variation across the ring (simulate 3-D shading)
    color_layer = np.full_like(img, color, dtype=np.uint8)
    shade = np.zeros((h, w), dtype=np.float32)
    cv2.ellipse(shade, (cx, cy), (rx, ry), angle_deg, 0, 360, 1.0, -1)
    shade = cv2.GaussianBlur(shade, (max(3, rx | 1), max(3, ry | 1)), rx / 3)
    shade = (shade * 0.3 + 0.85).clip(0.7, 1.15)
    color_layer = np.clip(
        color_layer.astype(np.float32) * shade[:, :, np.newaxis], 0, 255
    ).astype(np.uint8)

    img[mask > 0] = color_layer[mask > 0]
    return img


def ring_bbox(cx, cy, rx, ry, angle_deg, img_w, img_h):
    """Axis-aligned bounding box of a rotated ellipse, normalised to [0,1]."""
    a = math.radians(angle_deg)
    hw = math.sqrt((rx * math.cos(a)) ** 2 + (ry * math.sin(a)) ** 2)
    hh = math.sqrt((rx * math.sin(a)) ** 2 + (ry * math.cos(a)) ** 2)
    bx = cx / img_w
    by = cy / img_h
    bw = min(1.0, 2 * hw / img_w)
    bh = min(1.0, 2 * hh / img_h)
    return bx, by, bw, bh


# ── Image generation ──────────────────────────────────────────────────────────
def generate_image():
    h = w = IMG_SIZE
    img = random_background(h, w)
    annotations = []

    if random.random() < NEGATIVE_PROB:
        return add_distractors(img, max_items=7), annotations

    if random.random() < 0.45:
        img = add_distractors(img, max_items=3)

    n_rings = random.randint(1, 3)
    placed = []  # (cx, cy, rx, ry)

    for _ in range(n_rings):
        class_id = random.randint(0, 3)

        # Size: outer radius 15–90 px, perspective tilt gives ellipse
        rx = random.randint(15, 90)
        tilt = random.uniform(0.35, 1.0)   # 1.0 = front-on circle, 0.35 = steep tilt
        ry = max(4, int(rx * tilt))
        angle_deg = random.randint(0, 180)
        inner_ratio = random.uniform(0.40, 0.65)

        margin = rx + 4
        cx = random.randint(margin, w - margin)
        cy = random.randint(margin, h - margin)

        # Reject if overlaps an existing ring
        overlap = any(
            math.hypot(cx - px, cy - py) < (rx + prx) * 0.8
            for px, py, prx, _ in placed
        )
        if overlap:
            continue
        placed.append((cx, cy, rx, ry))

        img = draw_ring(img, cx, cy, rx, ry, angle_deg, inner_ratio, class_id)

        # Optional distance blur
        if random.random() < 0.25:
            k = random.choice([3, 5])
            img = cv2.GaussianBlur(img, (k, k), 0)

        bx, by, bw, bh = ring_bbox(cx, cy, rx, ry, angle_deg, w, h)
        annotations.append(f"{class_id} {bx:.6f} {by:.6f} {bw:.6f} {bh:.6f}")

    return img, annotations


# ── Dataset creation ──────────────────────────────────────────────────────────
def create_split(split_name, n):
    img_dir = OUT_DIR / "images" / split_name
    lbl_dir = OUT_DIR / "labels" / split_name
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    for i in range(n):
        img, anns = generate_image()
        stem = f"{split_name}_{i:05d}"
        cv2.imwrite(str(img_dir / f"{stem}.jpg"), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(lbl_dir / f"{stem}.txt", "w") as f:
            f.write("\n".join(anns))
        if (i + 1) % 500 == 0:
            print(f"  {split_name}: {i+1}/{n}")


def main():
    random.seed(SEED)
    np.random.seed(SEED)

    print(f"Generating {N_TRAIN} train + {N_VAL} val images → {OUT_DIR}")
    create_split("train", N_TRAIN)
    create_split("val",   N_VAL)

    yaml_path = OUT_DIR / "ring_dataset.yaml"
    yaml_path.write_text(
        f"path: {OUT_DIR}\n"
        "train: images/train\n"
        "val:   images/val\n"
        "nc: 4\n"
        "names: ['red', 'green', 'blue', 'black']\n"
    )
    print(f"Done. Dataset YAML: {yaml_path}")


if __name__ == "__main__":
    main()
