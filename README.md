**TODO LIST (Task1):**
- Razmislit o pathu, ki ga bomo naredili pri detektiranju obrazov.
- Face Detection tako, da ko enkrat obraz "pravilno" zazna, ga naslednjič ignorira.
- Ring Detection


Za delovanje programa poženeš skripti v tem vrstnem redu:
- server_sim.sh
- run_sim.sh

odpre rviz in naloži vse mape.

Z skripto:
- kill_ros_processes.sh
ustrezno zapremo delovanje programa.

start_slam.sh je da samo sam od sebe skeniraš mapo.

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


