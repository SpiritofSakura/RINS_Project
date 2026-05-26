import sys
from pathlib import Path

import albumentations as A
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from albumentations.pytorch import ToTensorV2
from cv_bridge import CvBridge
import torch
import segmentation_models_pytorch as smp

_root = Path(__file__).resolve()
while not (_root / "src" / "anomaly_detection").exists():
    _root = _root.parent
CHECKPOINT = _root / "src" / "anomaly_detection" / "results" / "unet" / "best_model.pth"
IMAGE_SIZE = 512
THRESHOLD = 0.20
MIN_DEFECT_RATIO = 0.002

MORPH_OPEN_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
MORPH_CLOSE_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def make_transform(image_size):
    return A.Compose([
        A.Resize(image_size, image_size),
        A.CLAHE(clip_limit=4.0, tile_grid_size=(8, 8), p=1.0),
        A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
        ToTensorV2(),
    ])


def postprocess(mask: np.ndarray) -> np.ndarray:
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_OPEN_K)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_CLOSE_K)
    return mask


class TileClassifier(Node):
    def __init__(self):
        super().__init__("tile_classifier")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/tile_warped", self._warped_cb, 10)
        self.result_pub = self.create_publisher(String, "/tile_classification", 10)
        self.heatmap_pub = self.create_publisher(Image, "/tile_heatmap", 10)
        self._model = None
        self._transform = make_transform(IMAGE_SIZE)
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self._load_model()
        self.get_logger().info(
            f"TileClassifier ready (device={self._device}, model={'loaded' if self._model else 'N/A'})"
        )

    def _load_model(self):
        if not CHECKPOINT.exists():
            self.get_logger().warn(f"Checkpoint not found: {CHECKPOINT} — running in demo mode")
            return
        try:
            model = smp.Unet(encoder_name="resnet34", encoder_weights=None, in_channels=3, classes=1)
            model.load_state_dict(torch.load(CHECKPOINT, map_location=self._device, weights_only=True))
            model.to(self._device).eval()
            self._model = model
            self.get_logger().info(f"Loaded checkpoint: {CHECKPOINT}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def _classify(self, bgr: np.ndarray) -> tuple[str, float, np.ndarray, np.ndarray]:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h_orig, w_orig = rgb.shape[:2]

        if self._model is None:
            prob = np.zeros((IMAGE_SIZE, IMAGE_SIZE), dtype=np.float32)
        else:
            tensor = self._transform(image=rgb)["image"].unsqueeze(0).to(self._device)
            with torch.no_grad():
                prob = torch.sigmoid(self._model(tensor)).squeeze().cpu().numpy()
            prob = cv2.resize(prob, (w_orig, h_orig), interpolation=cv2.INTER_LINEAR)

        pred = postprocess((prob > THRESHOLD).astype(np.uint8))
        defect_ratio = float(pred.sum()) / max(pred.size, 1)
        label = "DEFECT" if defect_ratio >= MIN_DEFECT_RATIO else "OK"
        return label, defect_ratio, pred, prob

    def _draw_visualization(self, bgr: np.ndarray, label: str, defect_ratio: float,
                            pred: np.ndarray, prob: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        scale = min(600 / w, 600 / h, 1.0)
        if scale < 1.0:
            disp_w, disp_h = int(w * scale), int(h * scale)
            disp = cv2.resize(bgr, (disp_w, disp_h))
            prob_disp = cv2.resize(prob, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)
        else:
            disp, prob_disp = bgr.copy(), prob.copy()
            disp_h, disp_w = h, w

        heatmap = cv2.applyColorMap((prob_disp * 255).astype(np.uint8), cv2.COLORMAP_JET)
        blended = cv2.addWeighted(disp, 0.5, heatmap, 0.5, 0)

        panel = np.concatenate([disp, blended], axis=1)

        color = (0, 255, 0) if label == "OK" else (0, 0, 255)
        cv2.putText(panel, f"{label} ({defect_ratio*100:.2f}%)", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
        return panel

    def _warped_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Bridge error: {e}")
            return

        self.get_logger().info(f"Received warped tile ({bgr.shape[1]}x{bgr.shape[0]})")
        try:
            label, defect_ratio, pred, prob = self._classify(bgr)
            self.get_logger().info(f"Classification: {label}  (defect={defect_ratio*100:.2f}%)")

            tile_id = msg.header.frame_id
            heatmap = cv2.applyColorMap((prob * 255).astype(np.uint8), cv2.COLORMAP_JET)
            blended = cv2.addWeighted(bgr, 0.5, heatmap, 0.5, 0)
            heatmap_msg = self.bridge.cv2_to_imgmsg(blended, "bgr8")
            heatmap_msg.header.frame_id = tile_id
            self.heatmap_pub.publish(heatmap_msg)

            self.result_pub.publish(String(data=f"{label}:{tile_id}"))

            panel = self._draw_visualization(bgr, label, defect_ratio, pred, prob)
            cv2.imshow("analysis", panel)
        except Exception as e:
            self.get_logger().error(f"Classification failed: {e}")
            import traceback
            traceback.print_exc()

        cv2.waitKey(1)


def main():
    rclpy.init()
    node = TileClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()