# Slop-Bot – Startbefehle

Überblick über Pakete, Hardware und Aufbau: [README.md](README.md)

Jedes neue Terminal zuerst (und nach jedem Bauen in offenen Terminals neu):
source ~/slop-bot_ws/install/setup.bash

## Bauen – immer über das Skript (saubere Umgebung, ohne ~/ros2_ws und venv-Python)
cd ~/slop-bot_ws && ./colcon_build.sh && source install/setup.bash

## Einzelne Pakete bauen
./colcon_build.sh --packages-select slop_base slop_safety slop_teleop slop_sensors

## Tests (ohne Hardware)
colcon test --packages-select slop_base slop_safety slop_teleop && colcon test-result --verbose

## Antrieb + Sicherheit (bestromt Motoren, fragt nach sudo-Passwort, Fahren nur mit RustDesk-Sitzung)
## with_netcaps gibt dem EtherCAT-Zyklus Echtzeit-Priorität (Kern 7) und legt die USB-Interrupts von CPU0 auf CPU1
## (der EtherCAT-Interrupt selbst ist auf dem Orin fest auf CPU0).
## Alle 10 s steht die Zyklusstatistik im Log ("EtherCAT letzte 10 s: ... verspätet ... max. Periode").
## Radodometrie aus den ausgegebenen Schritten: /odom und TF odom → base_footprint (publish_tf: false schaltet das TF ab).
ros2 run slop_base with_netcaps ros2 launch slop_base bringup.launch.py

## Nur Antrieb
ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py

## Nur Sicherheit (twist_mux + RustDesk-Fahrsperre)
ros2 launch slop_safety safety.launch.py

## Tastatursteuerung (eigenes Terminal; Taste halten = fahren, wasd/Pfeile, q/e Kurve, Leertaste Stopp, +/- Tempo)
ros2 run slop_teleop keyboard_teleop

## Endstufen aus / an (an = auch Fehler quittieren)
ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: false"
ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: true"

## Hand-Stopp / aufheben
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: true"
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: false"

## Sensoren (nur Treiber: Lidar + Kamera, keine TFs zwischen Roboter und Sensoren)
ros2 launch slop_sensors sensors.launch.py

## Nur Lidar / nur Kamera
ros2 launch slop_sensors sensors.launch.py camera:=false
ros2 launch slop_sensors sensors.launch.py lidar:=false

## Lidar in RViz anschauen (PointCloud2 statt Livox-CustomMsg, Ansicht im Lidar-Frame)
ros2 launch slop_sensors sensors.launch.py camera:=false xfer_format:=0
rviz2 -f livox_frame

## Kamera mit farbiger Punktwolke in RViz anschauen
ros2 launch slop_sensors sensors.launch.py lidar:=false camera_pointcloud:=true
rviz2 -f camera_link

## IMUs plotten
ros2 run rqt_plot rqt_plot /livox/imu/linear_acceleration/z /camera/imu/linear_acceleration/y

## Lidar-Netzwerk verbinden / prüfen
nmcli con up livox-mid360
ping -c 3 192.168.1.162

## Motortest ohne ROS (aufbocken, diff_drive darf nicht laufen; CSV-Logs landen in tools/logs/)
cd ~/slop-bot_ws/tools
sudo ~/ethercat_venv/bin/python3 diff_motor.py info
sudo ~/ethercat_venv/bin/python3 diff_motor.py monitor
sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --rpm 10 --time 3

## Fremdpakete (Git-Submodule) nach dem Klonen holen
git submodule update --init

## Hängende Prozesse beenden
pkill -9 -x livox_ros_drive; pkill -x realsense2_came; pkill -x rviz2
