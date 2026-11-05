# Slop-Bot – Startbefehle

Überblick über Pakete, Hardware und Aufbau: [README.md](README.md)

Jedes neue Terminal zuerst (und nach jedem Bauen in offenen Terminals neu):
source ~/slop-bot_ws/install/setup.bash

# 1. Roboter steuern

## Fahren per Tastatur (2 Terminals: Antrieb + Modell + EKF, Tastatur; bestromt Motoren, fragt nach sudo-Passwort, Fahren nur mit RustDesk-Sitzung)
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

## Hinderniserkennung der Kamera ansehen (3 Terminals: Antrieb + Modell + EKF, Kamera mit Erkennung, RViz; rote Würfel = Hindernis, grün = freier Boden, dazu Farb- und Tiefenbild)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_sensors camera.launch.py

ros2 launch slop_description rviz.launch.py rviz_config:=obstacles

## Dasselbe ohne Antrieb (Modell statt Bringup)
ros2 launch slop_description description.launch.py

ros2 launch slop_sensors camera.launch.py

ros2 launch slop_description rviz.launch.py rviz_config:=obstacles

## Hinderniserkennung prüfen (ca. 13 Hz; im Kamera-Terminal alle 10 s "Boden gesehen bei z = …", soll nahe 0 cm sein)
ros2 topic hz /camera/obstacles

## IMUs plotten (2 Terminals: Sensoren, Plot)
ros2 launch slop_sensors sensors.launch.py

ros2 run rqt_plot rqt_plot /livox/imu/linear_acceleration/z /camera/imu/linear_acceleration/y

# 3. Kalibrierung

## Alle Kalibrierungen messen (2 Terminals: Sensoren, Messung; Roboter dabei stillstellen, waagrecht, nicht anfassen; ca. 15 s)
ros2 launch slop_sensors sensors.launch.py

ros2 run slop_calibration calibrate all

## Nur Gyro-Bias (Livox- und Kamera-IMU; der EKF braucht dann beim Start keine 5 s Wartezeit mehr)
ros2 run slop_calibration calibrate gyro

## Nur Sensorneigung Roll/Pitch aus der Schwerkraft (danach bringup bzw. description neu starten, das URDF liest sie beim Start; nach jedem Umbau eines Sensors, z. B. Kamera vorn seit 2026-11-05)
ros2 run slop_calibration calibrate mount

## Längeres Messfenster (Standard 10 s)
ros2 run slop_calibration calibrate all --duration 20

## Stand anzeigen: welche Kalibrierungen es gibt und wie alt die gespeicherten Werte sind
ros2 run slop_calibration calibrate list

## Gespeicherte Werte ansehen (Ordner calib/, siehe calib/README.md)
cat ~/slop-bot_ws/calib/gyro.yaml ~/slop-bot_ws/calib/mounts.yaml

## Gyro-Bias nur einmal neu messen, ohne zu speichern (Roboter stillhalten; läuft im imu_adapter)
ros2 service call /imu_adapter/recalibrate std_srvs/srv/Trigger

# 4. LIO (FAST-LIO2)

## LIO starten (3 Terminals: Antrieb + Modell + EKF, LIO mit Lidar, Tastatur; Roboter beim Start 2-3 s stillstellen)
## Körper und Mast werden aus den Scans geschnitten (src/slop_lio/config/crop.yaml)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_lio lio.launch.py

ros2 run slop_teleop keyboard_teleop

## LIO in RViz ansehen (4 Terminals: Antrieb + Modell + EKF, LIO mit Wolke, RViz mit LIO-Ansicht (Fixed Frame map, Ausschnitt-Quader, LIO- und EKF-Pose), Tastatur)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_lio lio.launch.py clouds:=true

ros2 launch slop_description rviz.launch.py rviz_config:=lio

ros2 run slop_teleop keyboard_teleop

## LIO, wenn der Lidar-Treiber schon läuft (sensors.launch.py, nur im Livox-Format: ohne xfer_format:=0)
ros2 launch slop_lio lio.launch.py lidar:=false

## LIO ohne Ausschneiden von Körper und Mast (Vergleich)
ros2 launch slop_lio lio.launch.py crop:=false

## LIO ohne 2D-Scan (/livox/points und /scan laufen sonst mit)
ros2 launch slop_lio lio.launch.py scan:=false

## LIO ohne map → odom (wenn slam_toolbox die Karte führt, macht mapping.launch.py selbst)
ros2 launch slop_lio lio.launch.py map_tf:=false

## 2D-Scan prüfen (ca. 10 Hz, Frame base_footprint)
ros2 topic hz /scan
ros2 topic echo /scan --field header --once

## Laserscan und Lidar-Wolke überlagert in RViz (3 Terminals: Antrieb + Modell + EKF, LIO mit Scan, RViz; Fixed Frame base_footprint, Scan rot, Wolke nach Höhe gefärbt)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_lio lio.launch.py

ros2 launch slop_description rviz.launch.py rviz_config:=scan

## Dasselbe ohne Antrieb (Modell statt Bringup)
ros2 launch slop_description description.launch.py

ros2 launch slop_lio lio.launch.py

ros2 launch slop_description rviz.launch.py rviz_config:=scan

## LIO prüfen (eigenes Terminal): Nodes, Rate (ca. 10 Hz), Verzögerung (ca. 0,03 s), Pose in map, TF-Kette
## Im LIO-Terminal steht beim Start "map festgelegt: ... gekippt um roll …, pitch …" (Neigung des Lidars, ca. 1–2°)
ros2 node list | grep -E "ekf_filter_node|imu_adapter|fast_lio|lio_bridge|diff_drive_node"
ros2 topic hz /lio/odom
ros2 topic delay /lio/odom
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint

## LIO-Geschwindigkeit ansehen (geht in den EKF; im Stand Streuung ca. 0,01 m/s)
ros2 topic echo /lio/odom --field twist.twist

## Fahrt prüfen: 1 m geradeaus und 90° Drehung gegen das Maßband (Pose von Rädern, LIO und EKF vergleichen)
ros2 topic echo /wheel/odom --field pose.pose.position
ros2 topic echo /lio/odom --field pose.pose.position
ros2 run tf2_ros tf2_echo odom base_footprint

## Aufzeichnen für die Auswertung (Ordner bags/, nicht in Git)
ros2 bag record -o ~/slop-bot_ws/bags/lio_test /livox/lidar /livox/imu /wheel/odom /odom /lio/odom /Odometry /tf /tf_static /joint_states

# 5. Karte (slam_toolbox)

## Karte aufnehmen (3 Terminals: Antrieb + Modell + EKF, LIO + slam_toolbox + RViz, Tastatur; beim Start 2-3 s stillstehen, langsam fahren, zum Schluss zurück zum Start)
## Details und Tipps: src/slop_navigation/README.md
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_navigation mapping.launch.py

ros2 run slop_teleop keyboard_teleop

## Karte speichern (eigenes Terminal, solange mapping.launch.py läuft; Name frei wählbar)
mkdir -p ~/slop-bot_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/slop-bot_ws/maps/wohnung
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson/slop-bot_ws/maps/wohnung'}"

## Kartieren ohne RViz / wenn lio.launch.py schon läuft (dann muss es mit map_tf:=false gestartet sein)
ros2 launch slop_navigation mapping.launch.py rviz:=false
ros2 launch slop_navigation mapping.launch.py lio:=false

# A. Bauen, Tests und einzelne Befehle

## Bauen – immer über das Skript (saubere Umgebung, ohne ~/ros2_ws und venv-Python)
cd ~/slop-bot_ws && ./colcon_build.sh && source install/setup.bash

## Einzelne Pakete bauen
./colcon_build.sh --packages-select slop_base slop_safety slop_teleop slop_sensors slop_description slop_localization slop_bringup slop_calibration slop_lio slop_navigation fast_lio

## Tests (ohne Hardware, 95 Stück)
colcon test --packages-select slop_base slop_safety slop_teleop slop_localization slop_calibration slop_lio && colcon test-result --verbose

## Nur Antrieb (Busprozess ethercat_bus + diff_drive_node; Zyklusstatistik steht im Log von ethercat_bus)
ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py

## EtherCAT-Statistik des letzten Laufs nachlesen (Busprozess ethercat_bus: Zyklen, verspätet, WKC-Fehler, max. Periode)
grep -h "ethercat_bus" $(ls -td ~/.ros/log/*/ | head -20 | sed 's|$|launch.log|') 2>/dev/null | grep -E "EtherCAT letzte|Beendet|verloren|still" | tail -20

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

## Fremdpakete (Git-Submodule) nach dem Klonen holen (FAST_LIO braucht --recursive; colcon_build.sh wendet den Patch an)
git submodule update --init --recursive

## FAST-LIO-Patch neu erzeugen, nachdem im Submodule etwas geändert wurde
git -C src/third_party/FAST_LIO diff > src/third_party/patches/fast_lio.patch

## ALLE hängengebliebenen Slop-Bot-Prozesse beenden (Lidar-Treiber, Kamera, FAST-LIO, RViz, Antrieb, Reste von ros2 launch; zeigt erst die Liste und fragt nach; beendet nur Prozesse aus diesem Workspace und den Roboter-ROS-Paketen)
./stop_all.sh

## Dasselbe nur anzeigen / ohne Rückfrage
./stop_all.sh --list
./stop_all.sh --yes

## Einzeln nach Namen (beendet auch eigene RViz-Fenster; Notlösung, wenn stop_all.sh nicht reicht)
pkill -9 -x livox_ros_drive; pkill -x realsense2_came; pkill -x rviz2
