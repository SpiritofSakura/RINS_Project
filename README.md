# RINS Task 1 — Inspection Robot

## Running

```bash
# 1. Start the simulator
./server_sim.sh

# 2. Launch the robot stack
./run_sim.sh        # opens RViz, loads maps

# 3. Start the task
ros2 launch task1 task1.launch.py

# 4. Kill everything
./kill_ros_processes.sh
```

Build:
```bash
colcon build --merge-install
# or a single package:
colcon build --packages-select task1 --merge-install
```

---

## Architecture

| Node | Package | Purpose |
|------|---------|---------|
| `detect_people.py` | `dis_tutorial3` | YOLO person detection → `/people_marker` |
| `face_localizator.py` | `task1` | Clusters face detections, confirms unique locations, publishes sphere markers for navigation |
| `face_recognizer.py` | `task1` | Runs `face_recognition` for ID, DeepFace for gender, publishes live debug window |
| `detect_rings_v2.py` | `task1` | OpenCV Hough circle detection → ring markers |
| `ring_localizator.py` | `task1` | Clusters ring detections, confirms unique locations |
| `cylinder_segmentation` | `dis_tutorial5` | PCL RANSAC on point cloud → cylinder markers with saturation filter |
| `cylinder_localizator.py` | `task1` | Clusters cylinder detections, handles spill check state machine, saves report |
| `cylinder_debug_view.py` | `task1` | Live debug overlay for both bottom and top cameras |
| `behavior_manager.py` | `task1` | Central state machine: patrol → approach → interact → return |
| `waypoint_navigator.py` | `task1` | Executes predefined waypoint patrol via Nav2 |
| `robot_state_overlay.py` | `task1` | RViz 2D overlay showing current robot state |

**Robot states:** `IDLE` → `PATROL` → `APPROACH_FACE / RING / CYLINDER` → `INTERACT_*` → `RETURN_TO_PATROL`

---

## What is implemented

### Must do

- **Goal-based navigation** ✅  
  Nav2 `NavigateToPose` action. Predefined waypoints in `config/waypoints.yaml`. Robot patrols in a loop, interrupts for detections, returns to saved patrol pose after each interaction.

- **Face detection** ✅  
  YOLO-based detector (`detect_people.py`) publishes person bounding boxes and 3D position. `face_localizator` confirms locations after 10 detections within 1 m radius.

- **Ring detection** ✅  
  `detect_rings_v2.py` uses OpenCV Hough circle transform on the depth-registered RGB image. HSV-based color classification (red, green, blue, yellow, orange, black). `ring_localizator` confirms and deduplicates.

- **Cylinder detection** ✅  
  `cylinder_segmentation` (C++) runs RANSAC on the OAKD point cloud, checks radius (0.11 m ± 0.025), height spread, and **saturation filter** (rejects grey/beige boxes with low HSV saturation). Orientation (vertical/horizontal) is detected from point cloud spread along the cylinder axis.

- **Colour recognition** ✅  
  All detectors use HSV hue-based classification instead of RGB Euclidean distance. Handles red (wraps around 0°/360°), orange, yellow, green, blue, black. Tuned on simulator barrel images.

- **Approaching faces** ✅  
  `behavior_manager` enters `APPROACH_FACE` state, computes a standoff approach point 1 m in front of the face position, navigates there via Nav2.

- **Speech synthesis** ✅  
  Uses `espeak`. Robot speaks randomised lines when greeting faces, rings, and cylinders. Falls back silently if `espeak` is not installed.

### Should do

- **Face recognition** ✅  
  `face_recognizer.py` loads reference images from `config/personnel/` (filename format: `firstname_pronoun1_pronoun2_role_title.png`). Uses the `face_recognition` library (dlib) at tolerance 0.5. Publishes name + role on `/recognized_person`.

- **Gender recognition** ✅  
  DeepFace (`dominant_gender`) runs on the face crop extracted from `face_recognition` bounding boxes. Result is visual only — not derived from pronouns/filename. Shown live in the "Face Recognition" debug window as `Name (man/woman)` with a green bounding box and role label below.

- **Autonomous space exploration** ⚠️  
  Waypoint patrol covers the map. Waypoints defined in `config/waypoints.yaml`. Robot resumes patrol from saved pose after each interaction. **Missing:** advanced fine manoeuvring and intelligent exploration (dynamic replanning, frontier-based exploration, obstacle-aware fine positioning).

- **Creating the inspection report** ✅  
  `cylinder_localizator` writes `barrell_detection/barrel_report.json` with barrel ID, colour, orientation, leak flag, map coordinates, and image path. A camera snapshot is saved for every confirmed barrel.

### Needs improvement

- **Barrel / spill detection** ⚠️  
  Horizontal barrels are flagged as potential leaks. When the robot arrives (`INTERACT_CYLINDER`), `behavior_manager` signals `/start_spill_check`. The arm moves to look-down position, the top camera captures a frame, and HSV color masking checks if liquid of the barrel's colour is present on the floor (`SPILL_PIXEL_THRESHOLD = 500 px`).  
  **Known issues:**
  - The top camera field of view may not cover the spill area depending on arm position.
  - HSV ranges may need tuning per environment lighting — current ranges are conservative.
  - The `ARM_MOVE_WAIT_SECS = 4` delay may not be enough for the arm to fully settle before capture.
  - The pixel threshold is a fixed count, not relative to image area — may false-positive on large uniform floors.

---

## What is NOT implemented

- **Not crossing yellow line** ❌  
  No yellow line detection or constraint in navigation. Nav2 costmaps would need a custom layer or the path planner would need a forbidden-zone polygon.

- **Follow blue line** ❌  
  No line-following mode. Would require a separate controller node using camera-based lane detection.

- **Correct cell detection** ❌  
  Not started.

- **Tile detection** ❌  
  Not started.

- **Defect detection** ❌  
  Training code exists in `src/anomaly_detection/` (PatchCore / PaDiM / FastFlow via `anomalib`). Models have been trained offline on the RINS dataset. **Not integrated as a ROS2 node** — no live inference pipeline. Would need a node that subscribes to camera images, runs inference, and publishes results.

- **Advanced fine manoeuvring and mobile manipulation** ❌  
  No fine positioning control. The arm moves to fixed poses (look-down for spill check) but has no closed-loop manipulation. Robot navigation stops at Nav2 goal poses without precision alignment to targets.

- **Intelligent navigation and exploration** ❌  
  No frontier-based or coverage-based exploration. Patrol follows a fixed waypoint list — no dynamic replanning based on what has or hasn't been seen.

- **Dialogue with ASR** ❌  
  No microphone input or speech recognition. `espeak` is one-way only.

---

## Debug topics

| Topic | Content |
|-------|---------|
| `/face_debug_image` | Live camera with green bounding box, name, gender, role |
| `/cylinder_debug_image` | Bottom camera with projected cylinder circles + colour labels |
| `/cylinder_top_debug_image` | Top camera with spill warning banner when horizontal barrel detected |

View with: `rqt_image_view /face_debug_image`

---

## Dependencies

```bash
pip3 install face_recognition --break-system-packages
pip3 install deepface tf-keras --break-system-packages
pip3 install "numpy<2" --break-system-packages   # keep numpy 1.x for cv_bridge

sudo apt install ros-jazzy-rviz-2d-overlay-msgs ros-jazzy-rviz-2d-overlay-plugins
sudo apt install espeak
```

Personnel images: `src/task1/config/personnel/firstname_he_him_job_title.png`

---

## Known build notes

- Always use `--merge-install` flag: `colcon build --merge-install`
- C++ package (`dis_tutorial5`) needs: `colcon build --packages-select dis_tutorial5 --merge-install`
