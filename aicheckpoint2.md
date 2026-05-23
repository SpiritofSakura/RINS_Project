# AI Session Checkpoint 2

## Project: RINS — Workstation Anomaly Inspection

### Stack
- ROS2 Jazzy, Ubuntu 24.04, Gazebo Sim 8, TurtleBot4, Python-only (`rclpy`)
- Workspace: `/home/zeta/RINS_Project` (colcon, `--merge-install`)
- RMW: `rmw_zenoh_cpp`

---

## Files Modified (since Checkpoint 1)

### `src/task1/task1/station_inspector.py`
**Purpose:** Navigate to workstation, fine-position against wall, scan tiles for defects.

**Current state:** Stripped down to only `INSPECTOR_INACTIVE` → `LOAD_YAML` → `NAV_TO_WS`. FINE_POSITION is a placeholder (pass). The full FINE_POSITION flow needs to be implemented next.

**Key details:**

- State machine: `INSPECTOR_INACTIVE`, `LOAD_YAML`, `NAV_TO_WS`, `FINE_POSITION`, `EXTEND_ARM`, `SCAN_TILE`, `TILE_FOUND`, `MOVE_NEXT`, `FINISHED`
- Parameter `workstation` (green/red), `use_yaml` (bool)

### `src/task1/task1/robot_state_overlay.py`
- Added all inspector states to `DOVOLJENA_STANJA` with distinct colors
- States: `INSPECTOR_INACTIVE` (gray), `LOAD_YAML` (cyan), `NAV_TO_WS` (white), `FINE_POSITION` (blue), `EXTEND_ARM` (purple), `SCAN_TILE` (yellow-green), `TILE_FOUND` (green), `MOVE_NEXT` (orange), `FINISHED` (mint)

### `src/task1/launch/anomaly_inspection.launch.py`
- Launches: `line_localizator`, `station_inspector`, `arm_camera_viewer`, `robot_state_overlay`, `oakd_camera_viewer`
- No workstation_recorder (YAML is prepared separately)
- Argument: `workstation:=green|red`

### `src/task1/launch/capture_workspace_locations.launch.py`
- Standalone launch for YAML capture: `color_mask_viewer`, `line_localizator`, `workstation_recorder` (toYAML), `arm_camera_viewer`
- Use once to generate `test_workstation_locations.yaml`

### `src/task1/task1/line_localizator.py`
- Line detection logging changed from `.info()` to `.debug()` to reduce terminal clutter

### `src/task1/task1/workstation_recorder.py`
- Added `--mode` argument (`normal`/`toYAML`)
- Stores both approach poses AND line endpoints (p1, p2 in map frame)
- In `toYAML` mode, writes `config/test_workstation_locations.yaml` on lock
- YAML yaw fixed to point TOWARD the line center (`atan2(-dy, -dx)`)

### `src/task1/task1/oakd_camera_viewer.py`
- New. Opens OpenCV window showing `/oakd/rgb/preview/image_raw`.

### `src/task1/task1/arm_camera_viewer.py`
- New. Opens OpenCV window showing `/top_camera/rgb/preview/image_raw`.

---

## Key Facts Learned


### LiDAR
- Topic: `/scan` (`sensor_msgs/LaserScan`)
- 360°, 640 samples, 0.164–12.0m range, 62 Hz
- Mounted with `rpy=(0, 0, pi/2)`:
  - LiDAR angle **0** = robot **left**
  - LiDAR angle **π/2** = robot **forward**
  - LiDAR angle **π** = robot **right**

### Cameras
| Camera | Topic | Resolution |
|--------|-------|------------|
| OAK-D (front body) | `/oakd/rgb/preview/image_raw` | 320×240 |
| Arm camera (wrist) | `/top_camera/rgb/preview/image_raw` | 320×240 |

### Arm
- Controlled via `/arm_command` (`std_msgs/String`)
- Needs `ros2 run dis_tutorial7 arm_mover_actions.py` running
- Poses: `garage`, `look_at_belt_left`, `look_at_belt_right`, `look_for_qr`, `up`
- Arm joint order: `[arm_base_joint, arm_shoulder_joint, arm_elbow_joint, arm_wrist_joint]`

### Robot Dimensions
- Left wall distance in current sim setup: ~0.30m (lidar angle 0)
- Forward obstacle distance from approach point: varies

---

## Planned FINE_POSITION Flow (NOT YET IMPLEMENTED)

### Phase 1 — APPROACH_WALL
- Drive forward (any orientation after Nav2)
- Read lidar → min distance in **±45° forward cone** (lidar angles π/2 ± 0.785 rad)
- Fast **0.15 m/s** if > 0.40m, slow **0.03 m/s** if 0.30–0.40m
- **Stop** when ≤ **0.30m**
- Timeout 30s → FINISHED

### Phase 2 — SWEEP_NORMAL
- Rotate slowly, record (yaw, forward_distance) every 0.5s
- After ~30° rotation, find yaw with minimum forward_distance → **wall normal**
- `parallel_yaw = wall_normal + 90°`

### Phase 3 — ALIGN_WALL
- Rotate to `parallel_yaw` (parallel to wall, left side at ~30cm)

### Phase 4 — FIND_START
- Back up with yaw correction
- Stop when: yellow pixels in OAK-D bottom-left (line end) OR rear lidar ≤ 0.32m

---

## Scripts Available

| Script | Purpose |
|--------|---------|
| `./sim_run.sh` | Launch Gazebo + Nav2 |
| `./inspector.sh <green\|red>` | Read YAML → Nav2 → FINE_POSITION → scan |
| `./init_arm.sh` | Start arm command subscriber (needed for arm) |
| `./arm.sh <pose>` | Send arm command (garage/left/right/qr/up) |
| `./raw_camera.sh` | Open OAK-D camera window |
| `./arm_camera.sh` | Open arm camera window |

---

## Launch Files

```bash
# Capture workstation locations (one-time setup):
ros2 launch task1 capture_workspace_locations.launch.py mode:=toYAML

# Run inspection:
ros2 launch task1 anomaly_inspection.launch.py workstation:=green
```

