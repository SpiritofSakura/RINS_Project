**TODO LIST:**
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
