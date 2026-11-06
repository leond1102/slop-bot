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

## Weboberfläche: alles per Knopf starten, fahren, Notaus (Browser; Handy: …/handy). Einrichten: src/slop_web/README.md
ros2 run slop_web web
## dann im Browser: http://100.99.71.126:8080 (Tailnet) oder http://127.0.0.1:8080 (am Jetson), Handy: http://100.99.71.126:8080/handy

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
ros2 run nav2_map_server map_saver_cli -f ~/slop-bot_ws/maps/wohnung --free 0.196
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson/slop-bot_ws/maps/wohnung'}"

## Bestehende Karte ERWEITERN (neue Räume dazu): Navigation/Lokalisierung beenden, dann (Roboter beim Start stillstehen
## lassen: die Startpose wird am Scan gesucht, um die letzte gespeicherte Pose oder in der ganzen Karte; x:= y:= yaw:=
## gibt eine Vermutung vor)
ros2 launch slop_navigation mapping.launch.py extend:=office-3
## … in den neuen Bereich fahren (langsam, Überlappung mit dem Bekannten), dann unter NEUEM Namen speichern (wie oben)
## und Erfahrung + letzte Pose übertragen; danach mit der neuen Karte navigieren
ros2 run nav2_map_server map_saver_cli -f ~/slop-bot_ws/maps/office-4 --free 0.196
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson/slop-bot_ws/maps/office-4'}"
ros2 run slop_navigation experience_transfer ~/slop-bot_ws/maps/office-3.yaml ~/slop-bot_ws/maps/office-4.yaml
ros2 launch slop_navigation navigation.launch.py map:=$HOME/slop-bot_ws/maps/office-4.yaml
## Auf der Webseite: Karte wählen → „Karte erweitern komplett“ → fahren → Name eingeben → „Erweiterte Karte speichern“

## Kartieren ohne RViz / wenn lio.launch.py schon läuft (dann muss es mit map_tf:=false gestartet sein)
ros2 launch slop_navigation mapping.launch.py rviz:=false
ros2 launch slop_navigation mapping.launch.py lio:=false

# 6. Lokalisierung in der Karte (map_server + AMCL)

## Lokalisieren (3 Terminals: Antrieb + Modell + EKF, LIO + AMCL + RViz, Tastatur; Startpose (0,0,0) = Start beim Kartieren)
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_navigation localization.launch.py

ros2 run slop_teleop keyboard_teleop

## Andere Startpose / andere Karte (sonst in RViz "2D Pose Estimate")
ros2 launch slop_navigation localization.launch.py x:=2.0 y:=-1.0 yaw:=1.57
ros2 launch slop_navigation localization.launch.py map:=$HOME/slop-bot_ws/maps/office-4.yaml

## Roboter versetzt / Pose falsch: relocalize sucht von selbst (Scan passt 3 s lang unter 60 %); von Hand sofort suchen
ros2 service call /relocalize std_srvs/srv/Trigger

## Wie gut passt der Scan zur Karte? (Anteil der Scanpunkte auf Wänden, 0..1; gut ab 0.8)
ros2 topic echo /relocalize/score

## Ohne automatische Relokalisierung
ros2 launch slop_navigation localization.launch.py relocalize:=false

## Im Stand nachschärfen (mehrmals aufrufen; AMCL rechnet sonst erst nach 10 cm Fahrt weiter)
ros2 service call /request_nomotion_update std_srvs/srv/Empty

## Lokalisierung prüfen: Pose und Unsicherheit, TF
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint

# 7. Navigation (Nav2)

## Navigieren (2 Terminals: Antrieb + Modell + EKF, Lokalisierung + Nav2 + RViz; Ziel in RViz mit "Nav2 Goal", Tastatur in einem 3. Terminal hat Vorrang)
## Vorerst nur mit dem Lidar (Kamera nicht nötig). Details, Costmaps und Sicherheit: src/slop_navigation/README.md
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py

ros2 launch slop_navigation navigation.launch.py

## Sicherheitsstufe locker: extrem nah an alles, volles Tempo, rückwärts erlaubt, dreht sich erst am Ziel in die Zielrichtung (Standard: safety:=streng)
## Auf der Webseite: Auswahl „Sicherheit“ neben der Karte, dann Navigation starten
ros2 launch slop_navigation navigation.launch.py safety:=locker

## Ziel abbrechen (oder im Nav2-Panel in RViz "Cancel"); Hand-Stopp stoppt auch Nav2
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: true"

## Nav2 prüfen: alle Knoten aktiv, Befehle am Ausgang, Zonen
ros2 lifecycle get /controller_server
ros2 topic echo /navigation/cmd_vel
ros2 topic echo /polygon_stop --once

## Dynamisches Sicherheitsfeld: Wände bekommen nur die kleinen Zonen (rot/gelb, 2 cm Stopp / 10 cm langsam), unbekannte
## Hindernisse die größeren (dunkelrot/orange, 10 cm Stopp / 40 cm langsam); unbekannte Punkte in RViz magenta
## Im Navigations-Terminal: "Sicherheitsfeld: Karte genutzt" = normal; "alle Punkte gelten als unbekannt" = vorsichtig
## (keine Karte / Lokalisierung unsicher), dann gelten die größeren Zonen auch für Wände
ros2 topic hz /safety/unknown_points

## Sensor-Wächter (startet mit navigation.launch.py): "frei" oder "gesperrt: /scan seit 0.5 s still" usw.
## Gesperrt = Nav2 steht, Tastatur und Web fahren weiter
ros2 topic echo /safety/sensor_status

## Nicht den "Reset"-Knopf im Nav2-Panel benutzen: Abbrechen mit "Cancel", Neustart mit Ctrl-C und neu starten

## Erfahrungskarte: lernt bei jeder Fahrt mit (Lokalisierung muss passen), Zähler in maps/<karte>.experience.npz
## In RViz "Erfahrung (Beständigkeit)" einschalten: weiß = immer frei, grau = mal belegt (Stuhl), schwarz = fest
## Im Navigations-Terminal jede Minute: "Scans letzte 60 s: gezählt …, gesperrt: … | Zellen: fest …, vielleicht …, frei …"

## Navigieren mit weichen Kosten aus der Erfahrung (Stellen, wo oft ein Stuhl steht, werden gemieden, aber nie gesperrt)
ros2 launch slop_navigation navigation.launch.py experience_costs:=true

## Erfahrung sofort speichern (passiert sonst jede Minute und beim Beenden)
ros2 service call /experience/save std_srvs/srv/Trigger

## Karte mit der Erfahrung aktualisieren: erst nur zählen, dann neue Karte schreiben (Original bleibt, neue heißt office-3-neu)
ros2 run slop_navigation experience_export ~/slop-bot_ws/maps/office-3.yaml --nur-zeigen
ros2 run slop_navigation experience_export ~/slop-bot_ws/maps/office-3.yaml
ros2 launch slop_navigation navigation.launch.py map:=$HOME/slop-bot_ws/maps/office-3-neu.yaml

## Ohne Erfahrungskarte
ros2 launch slop_navigation navigation.launch.py experience:=false

# A. Bauen, Tests und einzelne Befehle

## Bauen – immer über das Skript (saubere Umgebung, ohne ~/ros2_ws und venv-Python)
cd ~/slop-bot_ws && ./colcon_build.sh && source install/setup.bash

## Einzelne Pakete bauen
./colcon_build.sh --packages-select slop_base slop_safety slop_teleop slop_sensors slop_description slop_localization slop_bringup slop_calibration slop_lio slop_navigation fast_lio

## Tests (ohne Hardware, 161 Stück)
colcon test --packages-select slop_base slop_safety slop_teleop slop_localization slop_calibration slop_lio slop_navigation slop_web && colcon test-result --verbose

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
