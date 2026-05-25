# RINS Task 1 — Inspection Robot (anomaly-flow)

## Running

### Task 1 — Person/ring/cylinder inspection

```bash
# 1. Start the simulator
./server.sh

# 2. Launch the robot stack
./sim_run.sh        # opens RViz, loads maps

# 3. Start the task
ros2 launch task1 task1.launch.py

# 4. Kill everything
./kill_ros_processes.sh
```


Build:
```bash
./colcon_build.sh

# the above script contains
colcon build --merge-install
# or a single package:
colcon build --packages-select task1 --merge-install

# the above script contains
colcon build --merge-install
# or a single package:
colcon build --packages-select task1 --merge-install

```


# Blue Line Explorer -SpiritofSakura

ROS2 node (`task1/blue_line_explorer.py`) that follows a blue floor line using a top-down camera, always choosing left at intersections.

---

## How It Works

### 1. Color Detection

Each frame from the top camera is converted to HSV. A mask is built from two combined methods:

- **HSV range** — hue 82–102 (blue/cyan), saturation ≥ 120, value ≥ 60
- **Cyan dominance check** — blue and green channels both ≥ 80, and both at least 40 above red

The two masks are OR-ed together, then cleaned with morphological open+close to remove noise.

The total number of lit pixels is compared against `min_blue_pixels` (150). Below that threshold the line is considered not visible.

### 2. Centroid Steering (primary)

The full-image blob centroid is computed with `cv2.moments`. The normalised horizontal error is:

```
cx_norm = (centroid_x - image_width/2) / (image_width/2)
```

`cx_norm = 0` means the line is centered, `+1` means far right, `−1` far left.

The raw angular command is:

```
angular_raw = clamp(-kp * steer, -max_angular_speed, max_angular_speed)
```

An **EMA filter** (`angular_smoothing = 0.35`) prevents step-changes from causing jerks:

```
angular = alpha * angular_raw + (1 - alpha) * prev_angular
```

### 3. Direction ROIs (auxiliary)

Three rectangular regions are checked for blue pixels to classify what's ahead:

| Region   | X span       | Y span (of ROI height) |
|----------|-------------|------------------------|
| Left     | 0 – 33%     | 42 – 90%               |
| Straight | 34 – 66%    | 20 – 65%               |
| Right    | 67 – 100%   | 42 – 90%               |

A region is "active" when it contains ≥ 50 pixels and ≥ 0.8% coverage. These fire independently of the centroid and drive two decisions:

- **Speed**: fast (`0.40 m/s`) when only straight is active (clear corridor); slow (`0.15 m/s`) when split mode is active
- **Left preference**: `left_bias` and the clamp described below

### 4. Intersection Handling (`split_active` state machine)

**Entering split mode** — any tick where left or right ROI fires sets `split_active = True`.

**Steering at the intersection**:
- If `"left" in directions` (genuine choice): `angular_raw = clamp(-kp * steer + left_bias, ...)`. The constant `left_bias = 0.25` adds a leftward nudge on top of the centroid, so the robot prefers the left branch.
- If `"left"` is gone (robot has taken the turn, residual right pixels still visible): **pure centroid**, no bias. This is critical — applying the bias during re-centering fights the correction and keeps the robot off-center.
- If neither `"left"` nor `"straight"` is visible (only a right-side residual): `steer = min(steer, 0.0)` clamps rightward pull to zero.

**Exiting split mode** — the robot is considered re-centered when:
- `"straight" in directions` AND `abs(cx_norm) < 0.30`
- Held for `split_exit_hold = 0.5 s`

Once the timer expires, `split_active` clears and the robot returns to fast straight-line speed.

> **Why the exit timer resets only on `"left"`**: a residual right branch after the turn would otherwise restart the re-centering countdown every tick, keeping the robot in split mode forever. The exit timer is only cancelled when a genuine new left branch appears.

### 5. Recovery (U-turn)

- **Line lost**: if no line is detected for `line_lost_timeout = 2.0 s`, a U-turn starts.
- **Bump**: if the bump sensor fires (`HazardDetectionVector`, type `BUMP`), a U-turn starts immediately.

The U-turn rotates left at `uturn_angular_speed = 0.5 rad/s` for `uturn_duration = 6.5 s` (~180°), then enters SEARCH mode and slowly rotates left until the line reappears.

---

## State Machine

```
IDLE ──enable──► SEARCH ──line found──► FOLLOW ──line lost 2s──► UTURN ──done──► SEARCH
                                           │                        ▲
                                           └──bump──────────────────┘
```

States published on `/robot_state`: `BLUE_LINE_SEARCH`, `BLUE_LINE_FOLLOW`, `BLUE_LINE_DEAD_END`.

---

## Key Parameters

| Parameter | Default | Description |
|---|---|---|
| `linear_speed` | 0.15 | Speed during split/intersection (m/s) |
| `fast_linear_speed` | 0.40 | Speed on clear straight sections (m/s) |
| `kp_steer` | 1.2 | Proportional gain on centroid error |
| `max_angular_speed` | 0.7 | Angular clamp (rad/s) |
| `left_bias` | 0.25 | Constant leftward offset at intersections (rad/s) |
| `angular_smoothing` | 0.35 | EMA alpha (0 = frozen, 1 = no smoothing) |
| `split_exit_hold` | 0.5 | Seconds of straight-zone hold before exiting split mode |
| `min_blue_pixels` | 150 | Minimum pixels to consider line visible |
| `line_lost_timeout` | 2.0 | Seconds before U-turn on line loss |
| `uturn_duration` | 6.5 | Duration of U-turn rotation (s) |
| `branch_roi_top_fraction` | 0.42 | Top of left/right ROI bands |
| `branch_roi_bottom_fraction` | 0.90 | Bottom of left/right ROI bands |

All parameters can be changed at runtime with `ros2 param set /blue_line_explorer <param> <value>`.

---

## Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/top_camera/rgb/preview/image_raw` | `sensor_msgs/Image` | sub | Camera feed |
| `/cmd_vel_unstamped` | `geometry_msgs/Twist` | pub | Drive commands |
| `/blue_line_enabled` | `std_msgs/Bool` | sub | Enable/disable node |
| `/patrol_finished` | `std_msgs/Bool` | sub | Auto-start trigger |
| `/blue_line/debug_image` | `sensor_msgs/Image` | pub | Annotated debug view |
| `/blue_line/status` | `std_msgs/String` | pub | Per-tick status string |
| `/robot_state` | `std_msgs/String` | pub | State machine label |

---

## Quick Test

```bash
# From repo root — arm forward, debug view on
./blue_line_test.sh qr

# Check debug image in RViz or:
ros2 run rqt_image_view rqt_image_view /blue_line/debug_image

# Watch status
ros2 topic echo /blue_line/status
```

The debug image shows the blue mask overlay, the centroid dot, the three ROI boxes (green = active), and a status line with current mode, pixel count, centroid error, and active directions.


## 3stan tips for others to jumpstart

### The inspector and integration 

**Workspace localization**
- there is a **color_mask_viewer** and **line_localizator** node - both must run together to get published markers to **/line_markers** topic 
- to get location markers/waypoints of workstations for the inspector node (anomalies), run also **workstation_recorder** node
- these three nodes are alredy together in a laucnh file ros2 launch task1 anomaly_detection_flow.launch.py
 and can be run with 
```bash
 ./workspace_marker.sh
```
**How to integrate the inspector**
- there is a node called **orchestrator** that listens to these waypoints and remembers them in runtime
- to get the waypoint manually write "get_red_waypoint" or "get_green_waypoint" to topic **/orchestrator_in**
- response will be sent to topic **/orchestrator_out** with the waypoint (PoseStamped) and color of workstation in header.frame_id
- the inspector node and its sub nodes are called dynamically when needed to avoid wasting resources - so when he gets that task from a person 
- **this is my suggested flow - NOT IMPLEMENTED YET** - when person gives instruction to go to anomaly inspection, remember that in the orchestrator node, then just start the inspector node, wait a few seconds for it to inti and publish the correct message to /orchestrator_out based on given color from the human. The rest is aready implemented. This is how to open the inspector node from the orchestrator - make sure to run it in the "use_orchestrator:=true" mode
``` py
from ament_index_python.packages import get_package_prefix                  
     _prefix = get_package_prefix("task1")                                       
     _lib = os.path.join(_prefix, "lib", "task1")


subprocess.Popen(                                                                                  
         [sys.executable, os.path.join(_lib, "station_inspector"),                                                         
          "--ros-args", "-p", f"workstation:={color}",                                           
          "-p", "use_yaml:=false",                                                
          "-p", "use_orchestrator:=true"],                                           
         preexec_fn=os.setpgrp,                                                  
     )
```
- make sure to disable any other functionaliy at this time so it does not interfere.
- I highly reccomend using the orchestrator for this human interaction -> task run flow, here you can save intermediary states and requests and then decide how and when to execute them.
- please for the love of God do not touch the inspector implementation - if something seems wrong CONTACT ME!!!
- The only thing i can think of that could go wrong is if the robot when it arrives at the waypoint that was provided by the yaml of the orchestrator - that he is not facing towards the green line. If that happens, the way we remember the waypont will have to be tweaked.

**Report node**
- we have a report node - for testing it by hand you can call **./report.sh make** and **./report.sh clear** 
- to see checkpoints, wire it so that if you call **./report.sh make --no-increment** so you can see intermediary states - this is implemented for the inspector, but not yet for others.
- currently, ony the inspection is wired inside of it, nothing else. 
- to see how the report is implemented for the inspection, ask ai - it is slick and i suggest that the other tasks are done in a similar way. Do not touch the inspection part of the report.


**Final general tips**
- we have a ./robot_init.sh script that launches all the nodes that the robot needs to operate normally, like arm init, orchestrator, behaivor manager, report etc. Here add any nodes that need to be called on startup - no actions, just initialization.
- for anomaly inspection, run ./workspace_marker.sh, this will publish the markers to the orchestrator - the orchestrator itself is responsible for running the inspection itself by calling station_inspector node.
- similarly, other nodes that have similar functionality should also be put in ther respective launch files, like for instance ring detection nodes should be in a rins.launch.py file, similar for faces etc -- **NOT IMPLEMENTED YET**
- for not crossing yellow line and follow blue line, i strongly suggest that we use the arm camera, because localisation will be too noisy
- Tjas, when you will be making the waypoints, i suggest is sees the workstations for the first timefrom the left, so it will take less time for the inspector to set up his fine tuned position.

### Other useful commands
```bash
./robot_init # here there should be all the nodes that are required for the robot to even function  - untested, the launch file may need to be changed

./init_arm.sh # inits the arm subscriber - not neccesary if you already call the robot init

./arm.sh <arg> # takes one argument based on how you want to position the arm - the init arm must be running
# possible args are: garage (default - same as no arg), left, right, qr, spill, up.

./inspector.sh <arg> --yaml # arg = green or red -- optional yaml flag so it reads from the yaml i captured instead of the orchestartor - waypoints are in root folder in file test_workstation_waypoints.yaml - copy it to install/share/task1/config/ and it will work.

./report make --no-increment # generates report with optional no increment flag, used to see snapshots and not break the setup if you want to generate the report from the same run data again - ask ai why its there

./report clear # clears the report folder and the nodes memory

./report launch # not necessary if you already call the robot init
```


---

## Architecture

### Task 1 nodes

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



### Anomaly Detection Flow nodes (Task 2)

| Node | Package | Purpose |
|------|---------|---------|
| `line_localizator.py` | `task1` | Detects colored floor lines (red/green/blue/yellow) via HSV masking in OAK-D camera; fits 3D line segments from point cloud → `/line_markers` |
| `workstation_recorder.py` | `task1` | Records red/green workstation approach poses by accumulating line detections, median-filtering, and writing `test_workstation_locations.yaml` |
| `station_inspector.py` | `task1` | Central anomaly inspection state machine: navigates to workstation, fine-positions (LiDAR wall stop, yaw align, Hough tilt correction, yellow-line creep), scans tiles, executes escape maneuver |
| `tile_detect.py` | `task1` | Detects tiles on conveyor belt via Otsu thresholding on arm camera; computes perspective warp and publishes tile image → `/tile_warped` |
| `tile_classifier.py` | `task1` | Runs PyTorch U-Net (ResNet34 encoder) trained on RINS defect dataset; publishes OK/DEFECT → `/tile_classification` and probability heatmap → `/tile_heatmap` |
| `report.py` | `task1` | Subscribes to all tile topics; generates PDF and Markdown inspection reports with defect textures and heatmap overlays |
| `parallel_align.py` | `task1` | Standalone Hough-line-based parallel alignment using arm camera; P-controller for tilt correction (independent test/calibration tool) |
| `arm_camera_viewer.py` | `task1` | Diagnostic viewer for arm-mounted RGB camera (`/top_camera`) |
| `oakd_camera_viewer.py` | `task1` | Diagnostic viewer for OAK-D RGB camera |
| `color_mask_viewer.py` | `task1` | Diagnostic viewer showing HSV color masking output (red/green/blue/yellow) |

**Inspector states:** `INSPECTOR_INACTIVE` → `LOAD_YAML` → `NAV_TO_WS` → `FINE_POSITION` (phases 0-5) → `EXTEND_ARM` → `SCAN_TILE` → `TILE_FOUND` → `MOVE_NEXT` → `FINISHED`

### Task 2 — Anomaly detection flow

**Calibration** (drive robot manually near workstation to record approach poses):
```bash
ros2 launch task1 anomaly_detection_flow.launch.py
# or, to save to YAML:
ros2 launch task1 capture_workspace_locations.launch.py
```

**Inspection** (automatic run):
```bash
./inspector.sh red
# or:
/inspector.sh green
```
currently the inspector reads from yaml - in the actual implementation, those waypoints will be dynamically detected during patrol.


**Manual tile detection** (outside launch system):
```bash
./tile_detect.sh
```

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

### Anomaly Detection Flow (Task 2)

- **Colored floor line detection** ✅  
  `line_localizator.py` detects red/green/blue/yellow lines in the OAK-D camera feed via HSV color masking, contour filtering, and PCA-based 3D line fitting from the point cloud. Used to identify workstation locations.

- **Workstation calibration** ✅  
  `workstation_recorder.py` accumulates line detections across 10 frames, median-filters the approach pose, and writes `config/test_workstation_locations.yaml`. Supports red and green workstations.

- **Workstation navigation** ✅  
  `station_inspector.py` loads YAML waypoints, sends Nav2 `NavigateToPose` goals, then executes a multi-phase fine-positioning routine:
  - **Phase 0:** Drive forward until LiDAR reads 0.30 m from wall.
  - **Phase 1:** Yaw-align to `π` (red) or `π/2` (green).
  - **Phase 2:** Hough-line-based tilt correction using arm camera (P-controller).
  - **Phase 3:** Reverse until yellow line detected in OAK-D bottom-left ROI or rear obstacle at 0.40 m.
  - **Phase 4:** Creep forward at 0.08 m/s scanning for tiles.
  - **Phase 5:** Turn 130° CW and drive forward to escape.

- **Tile detection** ✅  
  `tile_detect.py` uses Otsu thresholding on the arm camera feed, contour filtering by area (5-50%) and aspect ratio (< 2.0), and brightness triggering. Publishes perspective-warped tile images on `/tile_warped` for classification.

- **Tile defect classification** ✅  
  `tile_classifier.py` runs a PyTorch U-Net (ResNet34 encoder via `smp`) trained on the RINS tile dataset. CLAHE + ImageNet normalization preprocessing. Threshold at 0.20; ≥ 0.1% defect pixels → DEFECT label. Publishes result on `/tile_classification` and a blended probability heatmap on `/tile_heatmap`.

- **Inspection report generation** ⚠️  -- for now its only wired with the inspector, not yet with faces. 
  `report.py` subscribes to all inspection topics and generates PDF + Markdown reports in `reports/` with per-tile ID, OK/DEFECT status, defect texture crops, and heatmap overlays.

- **U-Net defect segmentation model** ✅  
  Trained model (`best_model.pth`, 98 MB) stored in `src/anomaly_detection/results/unet/`. Training data includes okay, damaged_0, damaged_1, and damaged_3 classes.

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

- **Advanced fine manoeuvring and mobile manipulation** ❌  
  No fine positioning control. The arm moves to fixed poses (look-down for spill check) but has no closed-loop manipulation. Robot navigation stops at Nav2 goal poses without precision alignment to targets.
  TUKAJ KOMENTAR - kje tocno bi to rabl? V inspectorju je fine manouvering odlicen.


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
| `/line_markers` | Colored floor line segments from `line_localizator` |
| `/workstation_markers` | Workstation approach pose markers from `workstation_recorder` |
| `/tile_warped` | Perspective-warped tile image sent to classifier |
| `/tile_heatmap` | U-Net probability heatmap overlay from `tile_classifier` |
| `/tile_status` | Tile detection state (`TILE_FOUND` / `TILE_LEFT`) |
| `/tile_classification` | Classification result (`OK` / `DEFECT`) |
| `/inspector_phase` | Current fine-positioning phase (0-5) |
| `/inspector_station` | Current workstation name (green/red) |
| `/parallel_align/debug/derivative` | Vertical derivative image used for Hough line alignment |
| `/parallel_align/debug/binary` | Otsu-thresholded binary image |
| `/parallel_align/debug/hough` | Hough line overlay on arm camera feed |

View with: `rqt_image_view /face_debug_image`

---

## Dependencies

```bash
pip3 install face_recognition --break-system-packages
pip3 install deepface tf-keras --break-system-packages
pip3 install "numpy<2" --break-system-packages   # keep numpy 1.x for cv_bridge

# Anomaly detection / tile classifier
pip3 install torch torchvision --break-system-packages
pip3 install segmentation-models-pytorch albumentations --break-system-packages
pip3 install fpdf2 --break-system-packages  # report PDF generation

sudo apt install ros-jazzy-rviz-2d-overlay-msgs ros-jazzy-rviz-2d-overlay-plugins
sudo apt install espeak
```

Personnel images: `src/task1/config/personnel/firstname_he_him_job_title.png`

Tile defect model: `src/anomaly_detection/results/unet/best_model.pth`

Workstation locations: `src/task1/config/test_workstation_locations.yaml` (generated by `capture_workspace_locations.launch.py`)

Task 2 specification: `task2.pdf`

---

## Known build notes

- Always use `--merge-install` flag: `colcon build --merge-install`
- C++ package (`dis_tutorial5`) needs: `colcon build --packages-select dis_tutorial5 --merge-install`
