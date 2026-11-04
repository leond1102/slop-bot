# Slop-Bot Workspace (ROS 2 Humble, Jetson Orin)

Differentialantrieb mit zwei Schrittmotoren am EtherCAT-Treiber CL57E-4A, Livox Mid-360s und RealSense D435i.

**Stand (Branch `slam-neu`):** Grundgerüst ohne SLAM. Enthalten sind nur Antrieb, Sicherheit, Tastatursteuerung und
die Sensortreiber. Es gibt **keine Odometrie und keine TFs** zwischen Roboter und Sensoren – das ist die Aufgabe des
SLAM, der hier neu aufgebaut wird. (Der alte FAST-LIO-Stack liegt auf `main`.)

Alle Startbefehle: [STARTBEFEHLE.md](STARTBEFEHLE.md)

## Pakete

| Paket | Inhalt |
|---|---|
| [slop_base](src/slop_base) | `diff_drive_node`: `/cmd_vel` → Räder über EtherCAT (pysoem, CSP, 2-ms-Zyklus als Echtzeit-Thread), `/joint_states` zurück. `with_netcaps` startet mit den nötigen Rechten. `bringup.launch.py` = Antrieb + Sicherheit |
| [slop_safety](src/slop_safety) | `twist_mux`-Konfiguration und `remote_guard`: Fahrsperre, solange keine RustDesk-Sitzung besteht |
| [slop_teleop](src/slop_teleop) | `keyboard_teleop`: Tastatursteuerung mit Totmann (Taste halten = fahren) |
| [slop_sensors](src/slop_sensors) | Launch-Dateien und Configs für Livox Mid-360s und RealSense D435i, sonst nichts |
| `src/third_party` (Git-Submodule) | `Livox-SDK2`, `livox_ros_driver2`, `realsense-ros` 4.58.4 |
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
```

## Sicherheit

- **Fahren nur mit RustDesk-Sitzung:** `remote_guard` sendet 5×/s `safety/remote_lock`. Fehlt die Sitzung oder die Node,
  sperren `twist_mux` und `diff_drive_node` (beide unabhängig, nach 1 s ohne Nachricht).
- **Hand-Stopp:** `ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: true"` (aufheben mit `false`).
- **Nur `twist_mux` darf auf `/cmd_vel` senden.** Sendet jemand anders, steht der Roboter.
- **Totmann und Timeouts:** Teleop sendet nur bei gehaltener Taste. Ohne `/cmd_vel` für 0,5 s bremst `diff_drive_node`.
  Hängt die Node, bremst der Bus-Watchdog im EtherCAT-Thread nach 0,3 s. Stirbt der Prozess, hält der Treiber die
  letzte Sollposition und meldet einen Alarm.
- **Anlaufsperre:** Nach Freigabe oder aufgehobener Sperre zählen Fahrbefehle erst nach 1 s Ruhe oder einem Stopp-Befehl.
- **Fehler an einem Rad schaltet beide ab.** Neu freigeben: `ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: true"`.

## Hardware und Geometrie

| | |
|---|---|
| Rechner | Jetson Orin, 8 Kerne. EtherCAT-Zyklus auf Kern 7 (SCHED_FIFO 80), USB-Interrupts auf CPU1 |
| Antrieb | CL57E-4A an `enP8p1s0`, Achse 3 = links (Getriebe 1:7,5), Achse 4 = rechts (1:14, gespiegelt). Schrittmotoren ohne Encoder (Open Loop) |
| Räder | Ø 7 cm, Spur 24 cm, angetrieben vorn. Schleppräder 34 cm dahinter. Höchstens 10 U/min am Rad → 0,037 m/s, 0,31 rad/s |
| `base_footprint` | Boden, Mitte zwischen den angetriebenen Rädern; x vorn, y links, z oben (wird noch von keiner Node gesendet) |
| Lidar | Livox **Mid-360s** (SDK-Typ 35, eigene Config `MID360s_config.json`), IP `192.168.1.162`; Jetson `192.168.1.50/24` am USB-LAN-Adapter (NetworkManager `livox-mid360`). 4 Scanlinien, ~20 000 Punkte je Scan bei 10 Hz, davon ~18 % Nullpunkte (kein Echo) |
| Kamera | RealSense D435i, SN 262722070636, USB 3.2. Farbe + Tiefe 640×480 @ 30 Hz, IMU 200 Hz (über den RSUSB-Build, siehe unten) |

**Montage der Sensoren – nur grobe Werte, nachmessen!** (aus dem alten LIO-Stack, 2026-11-03/04)

| Sensor | Position zu `base_footprint` | Neigung |
|---|---|---|
| Lidar `livox_frame` | x/y ≈ 0 (ungemessen), z ≈ 0,30 m (Platzhalter) | roll −0,4°, pitch +1,3° (Schwerkraft), yaw nie bestimmt |
| Kamera `camera_link` | x −0,12, y +0,06, z 0,40 m (ICP, ±5–10 cm) | pitch −8,6° = **schaut nach oben** (Schwerkraft und ICP einig), yaw ≈ −4° |

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

Tests (40, ohne Hardware):

```bash
source install/setup.bash && colcon test --packages-select slop_base slop_safety slop_teleop && colcon test-result --verbose
```

## Neu aufsetzen

1. Klonen samt Fremdpaketen: `git clone --recurse-submodules <repo> ~/slop-bot_ws`
   (oder im bestehenden Klon `git submodule update --init`).
2. **librealsense mit RSUSB-Backend** unter `~/librealsense-rsusb/install` (statisch, wird nur zum Bauen gebraucht).
   Grund: Der Jetson-Kernel hat kein HID-Sensor-Hub, mit der apt-librealsense (V4L) gibt es keine Kamera-IMU.
   Anleitung in `dbr_gesture/README.md` (alt_ros2_ws), danach `make -j6 all && cmake --install .` im Build-Ordner und
   `librealsense-file.a`, `librs_lz4.a`, `libsqlite3_lib.a` aus `build/Release` nach `install/lib` kopieren
   (`cmake --install` bricht vorher ab, weil es das Python-Modul ohne sudo nicht installieren kann).
3. **pysoem** in einer venv unter `~/ethercat_venv` (z. B. `python3 -m venv ~/ethercat_venv && ~/ethercat_venv/bin/pip install pysoem`).
   `with_netcaps` hängt deren `site-packages` an den `PYTHONPATH`; die Node selbst läuft mit dem System-Python.
4. ROS-Pakete aus apt, u. a. `ros-humble-twist-mux`.
5. Netzwerk für den Lidar: NetworkManager-Verbindung `livox-mid360`, statisch `192.168.1.50/24`, keine Standardroute.
6. `./colcon_build.sh`

Lokale Anpassungen der Fremdpakete legt `colcon_build.sh` selbst an, falls sie fehlen:

- `livox_ros_driver2/package.xml`: Kopie von `package_ROS2.xml` plus `<depend>livox_sdk2</depend>` (im Upstream-Repo
  per `.gitignore` ausgeschlossen). Das `build.sh` des Treibers **nicht** benutzen – es löscht `build/` und `install/`
  des ganzen Workspace.
- `realsense-ros/realsense2_description` und `realsense2_rgbd_plugin`: `COLCON_IGNORE` (werden nicht gebraucht).

Weitere Build-Optionen in `colcon.meta`: Livox-SDK2 braucht `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` (System-CMake 4.x),
`livox_ros_driver2` bekommt die SDK-Pfade, `realsense2_camera` den Pfad zu librealsense-rsusb und `-lyaml-cpp`
(der statische Export vergisst diese Abhängigkeit).

## Offene Punkte und Eigenheiten

- **`right_gear: 20.0`** in [diff_drive.yaml](src/slop_base/config/diff_drive.yaml) ist ein ungeklärter Workaround
  (Getriebe laut Test 1:14). `motor_current_ma` ist noch nicht eingetragen (Typenschild).
- **Keine Odometrie:** Open Loop ohne Encoder war zu ungenau. `odom → base_footprint` und `base_footprint → Sensor`
  muss der neue SLAM liefern.
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
├── colcon.meta          CMake-Optionen je Paket
├── STARTBEFEHLE.md      alle Startbefehle
├── src/
│   ├── slop_base/       Antrieb (EtherCAT)
│   ├── slop_safety/     twist_mux + RustDesk-Fahrsperre
│   ├── slop_sensors/    Sensortreiber-Launch + Configs
│   ├── slop_teleop/     Tastatursteuerung
│   └── third_party/     Livox-SDK2, livox_ros_driver2, realsense-ros (Submodule)
└── tools/
    ├── diff_motor.py    Motortest ohne ROS
    └── logs/            CSV-Logs der Motortests
```
