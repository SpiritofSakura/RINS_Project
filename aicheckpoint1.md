# AI Session Checkpoint

## Project: RINS — Line Detection & Workstation Localization

### Stack
- ROS2 Jazzy, Ubuntu 24.04, Gazebo Sim 8, TurtleBot4, Python-only (`rclpy`)
- Workspace: `/home/zeta/RINS_Project` (colcon, `--merge-install`)

---

## Files Built/Modified

### `src/task1/task1/line_localizator.py`
**Purpose:** Detects colored lines (red/green/blue/yellow) on the floor from OAK-D camera and publishes 3D LINE_STRIP markers.

**Pipeline:**
- HSV masking per color (ranges hardcoded, params for tuning)
- Only processes bottom half of image (floor region)
- Morphological close → findContours → filter by area (≥80px) + aspect ratio (≥3:1)
- `_skeletonize()` — iterative thinning to 1-pixel centerline
- `_split_by_skeleton()` — `HoughLinesP` on skeleton → each straight leg → sub-mask → PCA fit → 3D points from point cloud → flatness filter (3rd PCA std < 0.03m)
- Publishes `LINE_STRIP` Marker on `/line_markers`

**Parameters:** `max_flat_std` (0.03), `{color}_lo1/hi1` etc.

**Subscribes to:**
- `/oakd/rgb/preview/image_raw`
- `/oakd/rgb/preview/depth`
- `/oakd/rgb/preview/depth/points`

### `src/task1/task1/workstation_recorder.py`
**Purpose:** Listens to `/line_markers`, finds red/green lines, computes approach point 0.7m from the line toward the robot, locks after 10 confirms.

**Logic:**
- Gets robot position from TF (`base_footprint` → `base_link` fallback)
- Direction = line_center → robot (map frame)
- Marker point = line_center + `stop_distance` * normalized(direction toward robot) (always 0.7m fixed)
- Yaw = atan2 toward the line
- Accumulates 10 detections, takes median position + yaw, locks
- Publishes CYLINDER + TEXT_VIEW_FACING ("WS") MarkerArray on `/workstation_markers`

**Parameters:** `stop_distance` (0.7)

### `src/task1/task1/color_mask_viewer.py`
Unchanged. Debug viewer that shows HSV color masks in OpenCV windows.

### `src/task1/setup.py`
Entry points added: `line_localizator`, `workstation_recorder`

### `src/task1/launch/anomaly_detection_flow.launch.py`
Launches: `color_mask_viewer` + `line_localizator` + `workstation_recorder`

### `task1.rviz`
For RViz: add MarkerArray display for `/workstation_markers` with Namespace `workstations` enabled.

---

## Topics Created

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/line_markers` | Marker | `line_localizator` | LINE_STRIP per detected line segment |
| `/workstation_markers` | MarkerArray | `workstation_recorder` | CYLINDER + "WS" text for red/green workstations |

## Verification

```bash
# Launch all three nodes:
ros2 launch task1 anomaly_detection_flow.launch.py

# Check topics:
ros2 topic list
ros2 topic echo /line_markers
ros2 topic echo /workstation_markers

# Check params:
ros2 param list /line_localizator
ros2 param list /workstation_recorder

# Tune workstation distance:
ros2 param set /workstation_recorder stop_distance 1.0
```

## Known Behaviors
- Bend splitting (90° corners): works via skeleton + HoughLinesP
- Barrel rejection: flatness filter (3rd PCA std < 0.03m)
- Workstation lock: 10 detections → median, then ignores further detections
- Marker always 1m from line toward robot; if robot is within 1m, marker goes past line

