**TODO LIST (Task1):**
- Razmislit o pathu, ki ga bomo naredili pri detektiranju obrazov.
- Face Detection tako, da ko enkrat obraz "pravilno" zazna, ga naslednjič ignorira.
- Ring Detection


## Simulation

Run in this order:
- `server_sim.sh`
- `run_sim.sh`

Opens RViz and loads all maps.

To stop:
- `kill_ros_processes.sh`

To scan the map manually:
- `start_slam.sh`

---

## Real World (IRL)

### Requirements
- TurtleBot4 powered on and connected to the same network
- Map already built and saved at `src/dis_tutorial3/maps/map_name.yaml`
- Robot placed at the designated starting position

### First-time setup
Record the robot's starting position once (after doing a manual 2D Pose Estimate in RViz):
```bash
ros2 topic echo /amcl_pose --once
```
Update `src/dis_tutorial3/config/localization_irl.yaml` with the `x`, `y` and `yaw` values.
Yaw is computed from the quaternion: `yaw = 2 * arctan2(z, w)`.

### Running

**Terminal 1 — full stack (Nav2 + localization + RViz + task1):**
```bash
./real.sh
```
This opens a tmux session with 4 panes. Each pane starts with a delay so services come up in the right order. Wait until all panes are stable before starting patrol.

Tmux tips: `Ctrl+B` + arrow keys to switch panes, `Ctrl+B` + `D` to detach without killing.

**Terminal 2 — start patrol:**
```bash
./patrol.sh
```

### Adding/editing IRL waypoints
Waypoints are stored in `src/task1/config/irl-waypoints.yaml`. Each entry:
```yaml
- x: 1.23
  y: -0.45
  yaw: -1.57   # 2 * arctan2(qz, qw) from /amcl_pose
```
Drive the robot to a position with teleop, then read the pose:
```bash
ros2 topic echo /amcl_pose --once
```

**DODAJANJE PAKETOV:**
ros2 pkg create <naslov-paketa> --build-type ament_python --dependencies rclpy

V skripto build.sh v root directorju nato dodaj:
colcon build --packages-select <naslov-paketa> --symlink-install

**NEW IMPLEMENTATIONS**
- face_localizator.py 
  - listens on /people_marker and if detection fires 20 times in 1m radius, a persistent marker is published
  - later this will be used to visit the face
- robot_state_overlay.py
  - simple display in rviz that represents the robot current state.
  - Must install: sudo apt install ros-jazzy-rviz-2d-overlay-msgs ros-jazzy-rviz-2d-overlay-plugins
 




**Robot States are:**
IDLE:
If the robot doesn't do anything ("stays still"), it will be in state "idle".

MANUAL_CONTROL:
If you use teleop_key node where you control the robot using your keyboard, it will display "manual_control"
    
PATROL:
Robot goes into this state when it's following the "main" waypoints that were set across the map, or manually exploring the map in task2.

APPROACH_FACE:
When a robot detects a face during patrol, it will exit out of that state and move to approach_face, coming closer to the face.

INTERACT_FACE:
When the robot comes close to the face and finishes it's route from "approach_state", it will greet the person.

APPROACH_RING:
Same logic as in APPROACH_FACE

INTERACT_RING:
Same logic as in INTERACT_FACE

RETURN_TO_PATROL:
When the robot interatct succesfully, it will continue patrolling.

**FIXES**:
- If command "colcon build" doesn't work, add prefix --merge-install


