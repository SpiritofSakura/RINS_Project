#!/bin/bash
# Install all Python dependencies required by task1.
set -e

PIP="pip3 install --break-system-packages"

# NOTE: do NOT install opencv-python via pip — the system opencv has QT5 GUI
# support (imshow works), pip version does not.

echo "=== Installing face recognition ==="
$PIP face_recognition

echo "=== Installing gender/face attribute analysis (DeepFace) ==="
$PIP deepface

echo "=== Installing PDF generation ==="
$PIP fpdf2

echo "=== Installing image augmentation ==="
$PIP albumentations

echo "=== Installing PyTorch + torchvision ==="
$PIP torch torchvision

echo "=== Installing segmentation models (UNet) ==="
$PIP segmentation-models-pytorch

echo "=== Pinning numpy to 1.x (cv_bridge requires numpy < 2) ==="
$PIP "numpy<2"

echo ""
echo "All dependencies installed."
