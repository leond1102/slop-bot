# Slop-Bot – Startbefehle

Überblick über Pakete, Hardware und Aufbau: [README.md](README.md)

Jedes neue Terminal zuerst (und nach jedem Bauen in offenen Terminals neu):
source ~/slop-bot_ws/install/setup.bash

# 1. Roboter steuern

## Fahren per Tastatur (2 Terminals: Antrieb, Tastatur; bestromt Motoren, fragt nach sudo-Passwort, Fahren nur mit RustDesk-Sitzung)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 run slop_teleop keyboard_teleop

## Hand-Stopp / aufheben (eigenes Terminal)
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: true"
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: false"

## Endstufen aus / an (an = auch Fehler quittieren)
ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: false"
ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: true"

# 2. RViz

## Nur das Modell, ohne Hardware (1 Terminal, Räder per Schieberegler)
ros2 launch slop_description description.launch.py rviz:=true gui:=true

## Roboter live mit Modell, Rädern und Odometrie (3 Terminals: Antrieb + Modell + EKF, RViz, Tastatur; Gyro-Bias aus calib/ geladen, sonst Roboter beim Start 5 s stillhalten)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_description rviz.launch.py

ros2 run slop_teleop keyboard_teleop

## Lidar anschauen (2 Terminals: Treiber, RViz; PointCloud2 statt Livox-CustomMsg, Ansicht im Lidar-Frame)
ros2 launch slop_sensors sensors.launch.py camera:=false xfer_format:=0

rviz2 -f livox_frame

## Kamera mit farbiger Punktwolke anschauen (2 Terminals: Treiber, RViz)
ros2 launch slop_sensors sensors.launch.py lidar:=false camera_pointcloud:=true

rviz2 -f camera_link

## Modell + Lidar-Punktwolke + Kamerabild + Kamera-Punktwolke, ohne Antrieb (1 Terminal; Roboter steht, nicht zusammen mit dem Antrieb)
ros2 launch slop_description view.launch.py

## Dasselbe ohne Kamera-Punktwolke (weniger CPU)
ros2 launch slop_description view.launch.py camera_pointcloud:=false

## Alles zusammen: fahren und Sensoren im Modell sehen (4 Terminals: Antrieb + Modell + EKF, Sensoren, RViz mit allen Anzeigen, Tastatur)
## Fixed Frame odom; Kamera-Punktwolke in RViz einschalten und Sensoren dann mit camera_pointcloud:=true starten
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_sensors sensors.launch.py xfer_format:=0

ros2 launch slop_description rviz.launch.py

ros2 run slop_teleop keyboard_teleop

## IMUs plotten (2 Terminals: Sensoren, Plot)
ros2 launch slop_sensors sensors.launch.py

ros2 run rqt_plot rqt_plot /livox/imu/linear_acceleration/z /camera/imu/linear_acceleration/y

# 3. Kalibrierung

## Alle Kalibrierungen messen (2 Terminals: Sensoren, Messung; Roboter dabei stillstellen, waagrecht, nicht anfassen; ca. 15 s)
ros2 launch slop_sensors sensors.launch.py

ros2 run slop_calibration calibrate all

## Nur Gyro-Bias (Livox- und Kamera-IMU; der EKF braucht dann beim Start keine 5 s Wartezeit mehr)
ros2 run slop_calibration calibrate gyro

## Nur Sensorneigung Roll/Pitch aus der Schwerkraft (danach bringup bzw. description neu starten, das URDF liest sie beim Start)
ros2 run slop_calibration calibrate mount

## Längeres Messfenster (Standard 10 s)
ros2 run slop_calibration calibrate all --duration 20

## Stand anzeigen: welche Kalibrierungen es gibt und wie alt die gespeicherten Werte sind
ros2 run slop_calibration calibrate list

## Gespeicherte Werte ansehen (Ordner calib/, siehe calib/README.md)
cat ~/slop-bot_ws/calib/gyro.yaml ~/slop-bot_ws/calib/mounts.yaml

## Gyro-Bias nur einmal neu messen, ohne zu speichern (Roboter stillhalten; läuft im imu_adapter)
ros2 service call /imu_adapter/recalibrate std_srvs/srv/Trigger

# A. Bauen, Tests und einzelne Befehle

## Bauen – immer über das Skript (saubere Umgebung, ohne ~/ros2_ws und venv-Python)
cd ~/slop-bot_ws && ./colcon_build.sh && source install/setup.bash

## Einzelne Pakete bauen
./colcon_build.sh --packages-select slop_base slop_safety slop_teleop slop_sensors slop_description slop_localization slop_bringup slop_calibration

## Tests (ohne Hardware)
colcon test --packages-select slop_base slop_safety slop_teleop slop_localization slop_calibration && colcon test-result --verbose

## Nur Antrieb
ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py

## Nur EKF (/odom und TF odom → base_footprint aus /wheel/odom und weiteren Quellen; braucht: sudo apt install ros-humble-robot-localization)
ros2 launch slop_localization localization.launch.py

## Antrieb + Sicherheit ohne EKF (dann kein /odom)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py ekf:=false

## Nur Sicherheit (twist_mux + RustDesk-Fahrsperre)
ros2 launch slop_safety safety.launch.py

## Nur Robotermodell (TF base_footprint → base_link → Räder, livox_frame, camera_link)
ros2 launch slop_description description.launch.py

## Sensoren (nur Treiber: Lidar + Kamera)
ros2 launch slop_sensors sensors.launch.py

## Nur Lidar / nur Kamera
ros2 launch slop_sensors sensors.launch.py camera:=false
ros2 launch slop_sensors sensors.launch.py lidar:=false

## Motortest ohne ROS (aufbocken, diff_drive darf nicht laufen; liest Grenzen und Getriebe aus diff_drive.yaml; CSV-Logs landen in tools/logs/)
cd ~/slop-bot_ws/tools
sudo ~/ethercat_venv/bin/python3 diff_motor.py info
sudo ~/ethercat_venv/bin/python3 diff_motor.py monitor
sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --rpm 10 --time 3
sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --only left --revs 10 --rpm 5
sudo ~/ethercat_venv/bin/python3 diff_motor.py jog --only right --revs 10 --rpm 5

## Lidar-Netzwerk verbinden / prüfen
nmcli con up livox-mid360
ping -c 3 192.168.1.162

## Fremdpakete (Git-Submodule) nach dem Klonen holen
git submodule update --init

## Hängende Prozesse beenden (beendet auch eigene RViz-Fenster)
pkill -9 -x livox_ros_drive; pkill -x realsense2_came; pkill -x rviz2
