# slop_sensors

Startet die Sensoren des Slop-Bots und die Montage-TFs `base_footprint → Sensor`.

```bash
ros2 launch slop_sensors sensors.launch.py                # Lidar + Kamera + TFs
ros2 launch slop_sensors sensors.launch.py camera:=false  # nur Lidar
ros2 launch slop_sensors sensors.launch.py xfer_format:=0 # Lidar als PointCloud2 (RViz)

ros2 launch slop_sensors view.launch.py                   # Sensoren + RViz, Fixed Frame odom (braucht Antrieb)
ros2 launch slop_sensors view.launch.py fixed_frame:=base_footprint   # ohne Antrieb
ros2 launch slop_sensors view.launch.py camera_pointcloud:=false
```

`view.launch.py` stellt den Lidar auf PointCloud2 um, weil RViz das Livox-CustomMsg-Format nicht
anzeigen kann, und schaltet die farbige Kamera-Punktwolke ein. Für FAST-LIO wieder
`sensors.launch.py` mit CustomMsg nehmen.

| Topic | Rate | Quelle |
|---|---|---|
| `/livox/lidar` | 10 Hz | Livox Mid-360s (CustomMsg, mit `xfer_format:=0` PointCloud2) |
| `/livox/imu` | 200 Hz | IMU im Lidar, Beschleunigung in **g** |
| `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` | ~27 Hz | RealSense D435i |
| `/camera/imu` | 200 Hz | IMU der D435i, in m/s² |

Montage-TFs kommen von der Node `mount_tf`: roll/pitch misst sie beim Start aus der Schwerkraft der
Sensor-IMUs (Roboter still und waagrecht halten), x/y/z/yaw kommen aus `config/mounts.yaml`
(noch Platzhalter, nachmessen). Die Blickrichtung (yaw) ist aus der Schwerkraft nicht bestimmbar.
Neu messen: `ros2 service call /mount_tf/recalibrate std_srvs/srv/Trigger`.
Abschalten: `mount_from_gravity:=false`.
In RViz (beide Ansichten, Anzeige „Schwerkraft (mount_tf)“ auf `/mount_tf/markers`): orange Pfeil = live
gemessene Schwerkraft im Sensor, grüner Pfeil = senkrecht laut base_footprint, Text = Status, roll/pitch und
Abweichung live. Liegen beide Pfeile übereinander, passt die Montage.

## Kürzungen der Sensordaten

Alle Einstellungen, die Datenmenge kosten oder sparen (Auflösung und Bildrate der RealSense, Rate des Lidars,
Ausdünnung und Reichweiten in FAST-LIO, mount_calib und lio_map), stehen zentral in
[config/data_reduction.yaml](config/data_reduction.yaml), mit Begründung. Die Launch-Dateien laden sie nach der
jeweiligen Node-Config (Werte dort gewinnen). Nach Änderungen `colcon build --packages-select slop_sensors`.

## Rechenzeit (gemessen 2026-11-03, lio.launch.py ohne RViz, 100 % = 1 Kern von 8)

| Node | vorher | jetzt | Maßnahme |
|---|---|---|---|
| mount_calib | 125 % | 30 % | ICP nach dem ersten Treffer nur alle 10 s, open3d höchstens 2 Threads |
| mount_tf | 38–47 % | 10–13 % | IMUs vorab in C++ auf 25 Hz gedrosselt (Container `sensor_helpers`, +8 %), Rohempfang |
| lio_map | 22 % | 13–15 % | nur jeder 2. Scan, Binärsuche im Raster |
| fast_lio + livox_lidar_publisher | 92 % | 46 % | beide in einem Container (`lio_container`), Punktwolken intra-process statt serialisiert |
| camera (RealSense) | 17 % | – | |

Gesamt: ~2,6 → ~1,4 Kerne. Composable Nodes helfen nur, wenn Sender und Empfänger im selben Prozess sind: daher
Livox + FAST-LIO zusammen (lio.launch.py), die RealSense bleibt eigener Prozess (ihre Empfänger sind Python-Nodes
und RViz). Der Livox-Treiber stürzt beim Beenden ab und beendet dann den ganzen `lio_container` (harmlos).
Ohne Container: `lio.launch.py composable:=false`.

## Hardware

- **Livox Mid-360s** (SDK-Typ 35, *nicht* Mid-360): braucht `config/MID360s_config.json`
  im `Mid360s`-Format. Mit einer `MID360`-Config verwirft das SDK die Antwort des Lidars ohne Fehlermeldung.
- Netz: Lidar `192.168.1.162`, Jetson `192.168.1.50/24` am USB-LAN-Adapter
  (NetworkManager-Verbindung `livox-mid360`, statisch, keine Standardroute).
- **RealSense D435i**, SN 262722070636.

## Fremdpakete im Workspace (`src/third_party`, Build-Optionen in `colcon.meta`)

- `Livox-SDK2` + `livox_ros_driver2`: Das `build.sh` des Treibers **nicht** benutzen, es löscht
  `build/` und `install/` des ganzen Workspaces. `package.xml` ist eine Kopie von `package_ROS2.xml`
  mit zusätzlicher Abhängigkeit auf `livox_sdk2`.
- `realsense-ros` 4.58.4 (`realsense2_camera`, `realsense2_camera_msgs`), überlagert die apt-Version
  in `/opt/ros/humble`. Gebaut gegen **librealsense 2.58.4 mit RSUSB-Backend** aus
  `~/librealsense-rsusb/install` (statisch). Grund: Der Jetson-Kernel hat kein HID-Sensor-Hub,
  mit der apt-librealsense (V4L) gibt es keine Kamera-IMU.
  - `yaml-cpp` wird über die Linker-Flags in `colcon.meta` dazugelinkt, weil der statische
    librealsense-Export diese Abhängigkeit vergisst.
  - `~/librealsense-rsusb` wird nur zum Bauen gebraucht (statisch gelinkt). Neu aufsetzen:
    Anleitung in `dbr_gesture/README.md` (alt_ros2_ws), danach `make -j6 all && cmake --install .`
    im Build-Ordner und die Dateien `librealsense-file.a`, `librs_lz4.a` und `libsqlite3_lib.a` aus
    `build/Release` nach `install/lib` kopieren. `cmake --install` bricht vorher ab, weil es das
    Python-Modul ohne sudo nicht installieren kann.

## Build-Fallen

- `~/ethercat_venv` steht vorn im `PATH`, deshalb würde CMake dessen Python ohne ROS-Module nehmen.
  `colcon.meta` setzt für die betroffenen Pakete `/usr/bin/python3`.
- System-CMake ist 4.x, deshalb braucht Livox-SDK2 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`.
- Der Livox-Treiber stürzt beim Beenden ab (exit -11) und ignoriert manchmal SIGINT. Das ist harmlos,
  Restprozesse beendet `pkill -9 -x livox_ros_drive`.
