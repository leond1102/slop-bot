# Slop-Bot Workspace (ROS 2 Humble, Jetson Orin)

Differentialantrieb mit zwei Schrittmotoren am EtherCAT-Treiber CL57E-4A, Livox Mid-360s und RealSense D435i.

**Stand (Branch `slam-neu`):** Antrieb, Sicherheit, Tastatursteuerung, Sensortreiber, Robotermodell (URDF), EKF und
LIO-Odometrie. Die Antriebsnode sendet Rohodometrie der Räder (`/wheel/odom`), der EKF daraus `/odom` und das TF
`odom → base_footprint` (Räder, Livox-Gyro, LIO-Geschwindigkeit). Das URDF liefert `base_footprint → Sensoren`, FAST-LIO über
`lio_bridge` das TF `map → odom`. Eine gespeicherte Karte und Navigation gibt es noch nicht. (Der alte FAST-LIO-Stack liegt auf `main`.)

Alle Startbefehle: [STARTBEFEHLE.md](STARTBEFEHLE.md)
Offene Aufgaben: [TODO.md](TODO.md)

## Pakete

| Paket | Inhalt |
|---|---|
| [slop_base](src/slop_base) | Antrieb in zwei Prozessen: `ethercat_bus` (EtherCAT mit pysoem, CSP, 2-ms-Zyklus als Echtzeit-Thread, **ohne ROS**) und `diff_drive_node` (`/cmd_vel` → Drehzahlen, `/wheel/odom`, `/joint_states`), verbunden über einen lokalen Unix-Socket (`bus_link.py`). `with_netcaps` startet mit den nötigen Rechten. |
| [slop_localization](src/slop_localization) | EKF (`robot_localization`): bildet `/odom` und das TF `odom → base_footprint` aus allen Odometrie-/IMU-Quellen. Quellen (Radodometrie, Livox-Gyro über `imu_adapter`) stehen in `config/ekf.yaml`, neue Sensoren werden dort als Block ergänzt. Braucht `sudo apt install ros-humble-robot-localization` |
| [slop_bringup](src/slop_bringup) | `bringup.launch.py`: Antrieb + Sicherheit + Robotermodell + EKF in einem Start (`ekf:=false` ohne EKF) |
| [slop_calibration](src/slop_calibration) | `calibrate`-Befehl (Gyro-Bias, Sensorneigung) und Speicher für die Ergebnisse im Ordner `calib/` (siehe [calib/README.md](calib/README.md)) |
| [slop_lio](src/slop_lio) | FAST-LIO2 (Livox Mid-360s) und Brücke `lio_bridge`: `/lio/odom`, TF `map → odom`; die LIO-Geschwindigkeit geht als `odom1` in den EKF. Siehe [src/slop_lio/README.md](src/slop_lio/README.md) |
| [slop_navigation](src/slop_navigation) | Karte aufnehmen mit `slam_toolbox` (`mapping.launch.py`), lokalisieren mit `map_server` + AMCL (`localization.launch.py`) und `relocalize` (findet den Roboter nach dem Versetzen selbst wieder), navigieren mit Nav2 (`navigation.launch.py`: Costmaps aus Lidar und Kamera, Collision Monitor, nie rückwärts); Anleitung im README |
| [slop_safety](src/slop_safety) | `twist_mux`-Konfiguration und `remote_guard`: Fahrsperre, solange keine RustDesk-Sitzung besteht |
| [slop_teleop](src/slop_teleop) | `keyboard_teleop`: Tastatursteuerung mit Totmann (Taste halten = fahren) |
| [slop_web](src/slop_web) | Weboberfläche: Programme starten/stoppen, Fahren mit Totmann, Notaus – im Browser und am Handy (Tailnet) |
| [slop_description](src/slop_description) | URDF/xacro des Roboters (Räder, Schleppräder, Livox, RealSense) und `description.launch.py` mit `robot_state_publisher`. Radmaße, Spur und Gelenknamen liest es aus `diff_drive.yaml` |
| [slop_sensors](src/slop_sensors) | Launch-Dateien und Configs für Livox Mid-360s und RealSense D435i; `livox_to_cloud` (Livox → PointCloud2) und Einstellungen für den 2D-Scan `/scan` (`pointcloud_to_laserscan`); `depth_obstacles` (Hindernisse aus dem Tiefenbild der RealSense) |
| `src/third_party` (Git-Submodule) | `Livox-SDK2`, `livox_ros_driver2`, `realsense-ros` 4.58.4, `FAST_LIO` (Branch ROS2, mit `src/third_party/patches/fast_lio.patch`) |
| [tools/diff_motor.py](tools/diff_motor.py) | Motortest ohne ROS (Parameter lesen, Achsen zuordnen, Getriebe prüfen), CSV-Logs in `tools/logs/` |

## Datenfluss

```
keyboard_teleop ──teleop/cmd_vel (Prio 50)──┐
(Nav2, später)  ──navigation/cmd_vel (20)───┤
remote_guard ──safety/remote_lock (Sperre)──┼─► twist_mux ──/cmd_vel──► diff_drive_node ──EtherCAT──► CL57E-4A
Hand-Stopp   ──safety/stop (Sperre)─────────┘                              ▲ prüft zusätzlich
                                                                           └ safety/remote_lock

livox_lidar_publisher ─► /livox/lidar, /livox/imu        (Frame livox_frame)
camera (RealSense)    ─► /camera/color, /camera/depth, /camera/imu   (Frames unter camera_link)

diff_drive_node ─/wheel/odom──────────────┐
/livox/imu ─► imu_adapter ─/imu/livox─────┼─► EKF ──► /odom, TF odom → base_footprint
/livox/* ─► fast_lio ─► lio_bridge ─/lio/odom (nur Geschwindigkeit)─┘
                             └──► TF map → odom, TF static map → camera_init
robot_state_publisher (URDF) ──► TF base_footprint → base_link → Räder, livox_frame, camera_link

/livox/lidar ─► livox_to_cloud ─► /livox/points ─► pointcloud_to_laserscan ─► /scan   (für slam_toolbox / Nav2)
/camera/depth/image_rect_raw ─► depth_obstacles ─► /camera/obstacles, /camera/floor  (Hindernisse vor dem Roboter)
```

## Sicherheit

- **Fahren nur mit RustDesk-Sitzung:** `remote_guard` sendet 5×/s `safety/remote_lock`. Fehlt die Sitzung oder die Node,
  sperren `twist_mux` und `diff_drive_node` (beide unabhängig, nach 1 s ohne Nachricht).
- **Hand-Stopp:** `ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: true"` (aufheben mit `false`).
- **Nur `twist_mux` darf auf `/cmd_vel` senden.** Sendet jemand anders, steht der Roboter.
- **Totmann und Timeouts:** Teleop sendet nur bei gehaltener Taste. Ohne `/cmd_vel` für 0,5 s bremst `diff_drive_node`.
  Hängt die Node, bremst der Bus-Watchdog im Busprozess `ethercat_bus` nach 0,3 s; ist sie 2 s still, schaltet er die
  Endstufen aus. Stirbt der Busprozess, hält der Treiber die letzte Sollposition und meldet einen Alarm; die Node
  beendet sich dann ebenfalls.
- **Anlaufsperre:** Nach Freigabe oder aufgehobener Sperre zählen Fahrbefehle erst nach 1 s Ruhe oder einem Stopp-Befehl.
- **Fehler an einem Rad schaltet beide ab.** Neu freigeben: `ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: true"`.

## Hardware und Geometrie

| | |
|---|---|
| Rechner | Jetson Orin, 8 Kerne. EtherCAT-Zyklus (Prozess `ethercat_bus`) auf Kern 7 (SCHED_FIFO 80), USB-Interrupts auf CPU1 |
| Antrieb | CL57E-4A an `enP8p1s0`, Achse 3 = links (Getriebe 1:7,5), Achse 4 = rechts (1:14, gespiegelt). Schrittmotoren ohne Encoder (Open Loop) |
| Räder | Ø 7 cm, Spur 24 cm, angetrieben vorn. Schleppräder 34 cm dahinter. Höchstens 10 U/min am Rad → 0,037 m/s, 0,31 rad/s |
| `base_footprint` | Boden, Mitte zwischen den angetriebenen Rädern; x vorn, y links, z oben (TF `odom → base_footprint` vom EKF) |
| Lidar | Livox **Mid-360s** (SDK-Typ 35, eigene Config `MID360s_config.json`), IP `192.168.1.162`; Jetson `192.168.1.50/24` am USB-LAN-Adapter (NetworkManager `livox-mid360`). 4 Scanlinien, ~20 000 Punkte je Scan bei 10 Hz, davon ~18 % Nullpunkte (kein Echo) |
| Kamera | RealSense D435i, SN 262722070636, USB 3.2. Farbe + Tiefe 640×480 @ 30 Hz, IMU 200 Hz (über den RSUSB-Build, siehe unten) |

**Montage der Sensoren** (Position im [URDF](src/slop_description/urdf/slop_bot.urdf.xacro), Neigung aus
[calib/mounts.yaml](calib/mounts.yaml), `ros2 run slop_calibration calibrate mount`):

| Sensor | Position zu `base_footprint` | Neigung |
|---|---|---|
| Lidar `livox_frame` | x 0,03, y 0, z 0,315 m | roll −0,15°, pitch +1,27° (Schwerkraft), yaw nie bestimmt |
| Kamera `camera_link` | x 0,08, y 0, z 0,15 m (vorn am Roboter, seit 2026-11-05) | **nach dem Umbau neu messen** (`calibrate mount`); in `mounts.yaml` steht noch die alte Lage |

Die Kamera soll Hindernisse und Abgründe vor dem Roboter erkennen; das Lidar sieht den Boden erst ab ca. 2,6 m.

## Bauen

```bash
cd ~/slop-bot_ws && ./colcon_build.sh && source install/setup.bash
./colcon_build.sh --packages-select slop_base        # einzelne Pakete
```

**Immer über [`colcon_build.sh`](colcon_build.sh) bauen**, nicht mit `colcon build` im normalen Terminal. Das Skript
baut in einer leeren Umgebung, nur mit `/opt/ros/humble`, und umgeht so drei Fallen:

- `~/.bashrc` sourct `~/ros2_ws`. colcon würde diesen Workspace fest als Unterlage in `install/setup.bash` eintragen,
  danach lädt slop-bot_ws immer dessen Pakete mit (und meldet `not found: …/g_hri/…`).
- Im VS-Code-Terminal steht `~/ethercat_venv/bin` vorn im `PATH`. CMake nähme dessen Python ohne `catkin_pkg`
  (Fehler „No module named catkin_pkg“). Zusätzlich setzt [`colcon.meta`](colcon.meta) für jedes Paket `/usr/bin/python3`.
- `/opt/MVS/lib/aarch64` (Hikrobot-SDK, aus `~/.bashrc` und `~/.profile`) bringt eine alte libusb mit.
  `camera.launch.py` nimmt den Pfad zur Laufzeit aus `LD_LIBRARY_PATH`.

Tests (104, ohne Hardware):

```bash
source install/setup.bash && colcon test --packages-select slop_base slop_safety slop_teleop slop_localization slop_calibration slop_lio slop_navigation slop_web && colcon test-result --verbose
```

## Neu aufsetzen

1. Klonen samt Fremdpaketen: `git clone --recurse-submodules <repo> ~/slop-bot_ws`
   (oder im bestehenden Klon `git submodule update --init --recursive`; FAST_LIO braucht sein eigenes Submodule `ikd-Tree`).
2. **librealsense mit RSUSB-Backend** unter `~/librealsense-rsusb/install` (statisch, wird nur zum Bauen gebraucht).
   Grund: Der Jetson-Kernel hat kein HID-Sensor-Hub, mit der apt-librealsense (V4L) gibt es keine Kamera-IMU.
   Anleitung in `dbr_gesture/README.md` (alt_ros2_ws), danach `make -j6 all && cmake --install .` im Build-Ordner und
   `librealsense-file.a`, `librs_lz4.a`, `libsqlite3_lib.a` aus `build/Release` nach `install/lib` kopieren
   (`cmake --install` bricht vorher ab, weil es das Python-Modul ohne sudo nicht installieren kann).
3. **pysoem** in einer venv unter `~/ethercat_venv` (z. B. `python3 -m venv ~/ethercat_venv && ~/ethercat_venv/bin/pip install pysoem`).
   `with_netcaps` hängt deren `site-packages` an den `PYTHONPATH`; die Node selbst läuft mit dem System-Python.
4. ROS-Pakete aus apt, u. a. `ros-humble-twist-mux` und `ros-humble-robot-localization`.
5. Netzwerk für den Lidar: NetworkManager-Verbindung `livox-mid360`, statisch `192.168.1.50/24`, keine Standardroute.
6. `./colcon_build.sh`

Lokale Anpassungen der Fremdpakete legt `colcon_build.sh` selbst an, falls sie fehlen:

- `livox_ros_driver2/package.xml`: Kopie von `package_ROS2.xml` plus `<depend>livox_sdk2</depend>` (im Upstream-Repo
  per `.gitignore` ausgeschlossen). Das `build.sh` des Treibers **nicht** benutzen – es löscht `build/` und `install/`
  des ganzen Workspace.
- `realsense-ros/realsense2_description` und `realsense2_rgbd_plugin`: `COLCON_IGNORE` (werden nicht gebraucht).
- `FAST_LIO`: wendet `src/third_party/patches/fast_lio.patch` an (Details in [src/slop_lio/README.md](src/slop_lio/README.md)).

Weitere Build-Optionen in `colcon.meta`: Livox-SDK2 braucht `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` (System-CMake 4.x),
`livox_ros_driver2` bekommt die SDK-Pfade, `realsense2_camera` den Pfad zu librealsense-rsusb und `-lyaml-cpp`
(der statische Export vergisst diese Abhängigkeit).

## Offene Punkte und Eigenheiten

- `motor_current_ma` in [diff_drive.yaml](src/slop_base/config/diff_drive.yaml) ist noch nicht eingetragen (Typenschild).
- **Radodometrie ist Open Loop:** Sie zählt die ausgegebenen Schritte (Getriebe 1:5,18 links, 1:13,73 rechts, nachgezählt
  mit `tools/diff_motor.py jog --revs 10`) und driftet bei Schlupf oder Schrittverlust. Sie geht als `/wheel/odom` (ohne TF)
  in den EKF, zusammen mit dem Livox-Gyro und der LIO-Geschwindigkeit. Weitere Quellen (z. B. Kamera-IMU) kommen als Block in
  [ekf.yaml](src/slop_localization/config/ekf.yaml). Der EKF ist die einzige Quelle für `odom → base_footprint`.
- **LIO:** Stand, Messwerte und offene Punkte in [src/slop_lio/README.md](src/slop_lio/README.md). Unter Fahrt noch nicht vermessen.
- **Livox ohne Zeitsynchronisation (kein PTP):** Der Treiber stempelt mit der Empfangszeit des Jetson.
- Der Livox-Treiber stürzt beim Beenden manchmal ab (exit −11) und ignoriert SIGINT. Harmlos, Reste beendet
  `pkill -9 -x livox_ros_drive`.
- Die RealSense meldet „IMU Calibration is not available“: Die Kamera-IMU ist nicht werkskalibriert
  (einmal `rs-imu-calibration` laufen lassen).
- RViz zeigt das Livox-CustomMsg-Format (`xfer_format:=1`, Standard) nicht an, dafür `xfer_format:=0`.
- VS Code nutzt `.venv` (nicht versioniert) als Interpreter. Diese venv hat kein `rclpy`, deshalb kennt die
  Code-Analyse die ROS-Module nicht.

## Verzeichnisse

```
slop-bot_ws/
├── colcon_build.sh      Bauen (saubere Umgebung)
├── stop_all.sh          hängengebliebene Roboter-Prozesse beenden
├── colcon.meta          CMake-Optionen je Paket
├── STARTBEFEHLE.md      alle Startbefehle
├── TODO.md              offene Aufgaben (u. a. EtherCAT-Zyklus als eigener Prozess)
├── calib/               Kalibrierergebnisse (gyro.yaml, mounts.yaml), siehe calib/README.md
├── bags/                Aufzeichnungen (nicht in Git)
├── maps/                gespeicherte Karten (slam_toolbox / Nav2)
├── src/
│   ├── slop_base/       Antrieb (EtherCAT)
│   ├── slop_bringup/    bringup.launch.py (alles zusammen)
│   ├── slop_calibration/ Kalibrierbefehl + Speicher
│   ├── slop_description/ URDF + robot_state_publisher
│   ├── slop_localization/ EKF (robot_localization)
│   ├── slop_lio/        FAST-LIO2 + Brücke zu TF und EKF
│   ├── slop_navigation/ Karte (slam_toolbox), später Nav2
│   ├── slop_safety/     twist_mux + RustDesk-Fahrsperre
│   ├── slop_sensors/    Sensortreiber-Launch + Configs
│   ├── slop_teleop/     Tastatursteuerung
│   ├── slop_web/        Weboberfläche (Browser, Handy)
│   └── third_party/     Livox-SDK2, livox_ros_driver2, realsense-ros, FAST_LIO (Submodule), patches/
└── tools/
    ├── diff_motor.py    Motortest ohne ROS
    └── logs/            CSV-Logs der Motortests
```
