#!/usr/bin/env python3
"""
Burn yellow-line forbidden zones into map2.pgm as occupied (black) cells.

Run once after mapping. Re-run whenever you refine the coordinates.
The original map is backed up to map2.pgm.bak on first run.

  python3 make_yellow_layer.py

Coordinates are defined in yellow_lines_coords.yaml in world-frame metres.
To find coordinates in RViz: enable the "Publish Point" tool then
  ros2 topic echo /clicked_point
and click on the yellow lines on the map display.
"""

import shutil
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

MAP_PGM    = Path(__file__).parent / "map2.pgm"
MAP_YAML   = Path(__file__).parent / "map2.yaml"
COORDS_YAML = Path(__file__).parent / "yellow_lines_coords.yaml"


def world_to_px(wx, wy, origin, resolution, img_height):
    """Convert world-frame metres to image pixel (col, row)."""
    col = int(round((wx - origin[0]) / resolution))
    row = int(round(img_height - (wy - origin[1]) / resolution))
    return col, row


def main():
    # ── load map metadata ───────────────────────────────────────────────────
    with open(MAP_YAML) as f:
        cfg = yaml.safe_load(f)
    resolution = float(cfg["resolution"])
    origin     = cfg["origin"][:2]

    img = Image.open(MAP_PGM).convert("L")
    h, w = img.size[1], img.size[0]

    # ── backup original (only once) ─────────────────────────────────────────
    backup = MAP_PGM.with_suffix(".pgm.bak")
    if not backup.exists():
        shutil.copy(MAP_PGM, backup)
        print(f"Original backed up → {backup}")
    else:
        # Start from the clean backup each run so lines don't accumulate
        img = Image.open(backup).convert("L")

    draw = ImageDraw.Draw(img)

    with open(COORDS_YAML) as f:
        defs = yaml.safe_load(f)

    # ── filled polygons ─────────────────────────────────────────────────────
    for name, verts in (defs.get("yellow_polygons") or {}).items():
        px_verts = [world_to_px(x, y, origin, resolution, h) for x, y in verts]
        draw.polygon(px_verts, fill=0)
        print(f"  polygon '{name}'  ({len(px_verts)} vertices)")

    # ── line segments with explicit width ───────────────────────────────────
    for name, seg in (defs.get("yellow_lines") or {}).items():
        pts = seg["points"]
        width_px = max(1, int(round(seg.get("width_m", 0.15) / resolution)))
        px_pts   = [world_to_px(x, y, origin, resolution, h) for x, y in pts]
        draw.line(px_pts, fill=0, width=width_px)
        print(f"  line    '{name}'  width={width_px} px")

    img.save(MAP_PGM)
    print(f"\nSaved → {MAP_PGM}")
    print("Restart Nav2 (or the whole simulation) for the new map to take effect.")


if __name__ == "__main__":
    main()
