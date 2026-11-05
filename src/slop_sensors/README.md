# slop_sensors

Startet die Sensortreiber des Slop-Bots. Dazu kommt `livox_to_cloud` (Livox → PointCloud2) mit der Konfiguration für den
2D-Scan (`config/scan.yaml`); beides startet `slop_lio/launch/lio.launch.py` (siehe unten). Die TFs zwischen Roboter und
Sensoren kommen aus dem URDF (`slop_description`).

```bash
ros2 launch slop_sensors sensors.launch.py                # Lidar + Kamera
ros2 launch slop_sensors sensors.launch.py camera:=false  # nur Lidar
ros2 launch slop_sensors sensors.launch.py lidar:=false   # nur Kamera
ros2 launch slop_sensors sensors.launch.py xfer_format:=0 # Lidar als PointCloud2 (RViz)
ros2 launch slop_sensors sensors.launch.py camera_pointcloud:=true   # farbige Kamera-Punktwolke
```

Anschauen ohne TF-Baum: `rviz2 -f livox_frame` (Lidar, nur mit `xfer_format:=0`) bzw. `rviz2 -f camera_link`
(Kamera). RViz kann das Livox-CustomMsg-Format (`xfer_format:=1`, Standard) nicht anzeigen.

| Topic | Rate | Quelle |
|---|---|---|
| `/livox/lidar` | 10 Hz | Livox Mid-360s (CustomMsg, mit `xfer_format:=0` PointCloud2), Frame `livox_frame` |
| `/livox/imu` | 200 Hz | IMU im Lidar, Beschleunigung in **g** |
| `/camera/color/image_raw`, `/camera/depth/image_rect_raw` | 30 Hz | RealSense D435i, 640×480 |
| `/camera/imu` | 200 Hz | IMU der D435i (Gyro + Accel vereint), in m/s² |
| `/camera/obstacles`, `/camera/floor` | ~13 Hz | `depth_obstacles`: Hindernisse und freier Boden, Frame `base_footprint` |
| `/livox/points`, `/scan` | 10 Hz | `livox_to_cloud` und `pointcloud_to_laserscan` (mit `lio.launch.py`) |

Der RealSense-Treiber sendet selbst `camera_link` → Sensor- und optische Frames der Kamera; `base_footprint → livox_frame`
und `base_footprint → camera_link` kommen aus dem URDF (`description.launch.py`, im Bringup).

## 2D-Scan für slam_toolbox und Nav2

```
/livox/lidar (CustomMsg) ─► livox_to_cloud ─► /livox/points (PointCloud2, livox_frame, Roboter ausgeschnitten)
                                                └─► pointcloud_to_laserscan ─► /scan (LaserScan, base_footprint)
```

- **`livox_to_cloud`** (C++, `src/livox_to_cloud.cpp`): Der Livox-Treiber sendet nur ein Format, FAST-LIO braucht das
  CustomMsg. Der Knoten wandelt um und verwirft dabei Nullpunkte, Punkte näher als `blind` (0,3 m) und Punkte in den
  Quadern des Roboters, dieselben wie bei FAST-LIO (URDF + `slop_lio/config/crop.yaml`). Ca. 17 000 Punkte je Scan, 2–3 % eines
  Kerns. Rechnet nur, wenn jemand `/livox/points` abonniert.
- **`pointcloud_to_laserscan`**: schneidet das Höhenband 0,15–1,80 m über dem Boden aus und legt es als 360°-Scan
  (0,5°, 720 Strahlen) in `base_footprint`.
- Start: `ros2 launch slop_lio lio.launch.py` (Standard `scan:=true`), Einstellungen in `config/scan.yaml`.
- **Achtung, Nahbereich:** Das Mid-360 schaut nur 7° nach unten und sitzt 0,315 m hoch. Den Boden trifft es erst ab ca. 2,6 m;
  niedrige Hindernisse davor fehlen im Scan. Dafür ist die RealSense vorn am Roboter gedacht (Hindernisse und Abgründe).

## Hinderniserkennung mit der RealSense (`depth_obstacles`)

```
/camera/depth/image_rect_raw ─► depth_obstacles ─► /camera/obstacles (PointCloud2, base_footprint)  Hindernisse
                                                └► /camera/floor     (PointCloud2, base_footprint)  freier Boden
```

- C++ (`src/depth_obstacles.cpp`), startet mit `camera.launch.py` / `sensors.launch.py` (aus mit `obstacles:=false`),
  Einstellungen in `config/obstacles.yaml`. Braucht das TF `base_footprint → camera_link` aus dem Modell (Bringup oder
  `description.launch.py`).
- Jeder 4. Pixel des Tiefenbilds wird in `base_footprint` umgerechnet (z = 0 ist der Boden):
  - **Hindernis:** 0,05–0,9 m über dem Boden, 0,25–2,5 m Tiefe. Je 5-cm-Zelle ein Punkt, ab 3 Punkten in der Zelle (gegen Rauschen).
  - **Freier Boden:** |z| < 3 cm, in 10-cm-Zellen. Damit räumt die Costmap später Zellen wieder frei.
  - Dazwischen (3–5 cm) wird nicht entschieden. **Abgründe werden noch nicht erkannt.**
- Gemessen am 2026-11-05: 12–13 Bilder/s, 1 ms je Bild, ca. 2,5 % eines Kerns.
- **Neigungsprüfung:** Alle 10 s steht im Log, auf welcher Höhe der Boden gesehen wird. Am 2026-11-05: −0,7 cm (bis 1,2 m) und
  −1,2 cm (weiter). Weicht er mehr als 3 cm ab, warnt die Node: dann `ros2 run slop_calibration calibrate mount`.
- Für die Nav2-Costmap später: `sensor_frame: camera_link` angeben, sonst rechnet das Freiräumen vom Boden aus statt
  von der Kamera.
- RViz: `ros2 launch slop_description rviz.launch.py rviz_config:=obstacles`.

## Hardware

- **Livox Mid-360s** (SDK-Typ 35, *nicht* Mid-360): braucht `config/MID360s_config.json`
  im `Mid360s`-Format. Mit einer `MID360`-Config verwirft das SDK die Antwort des Lidars ohne Fehlermeldung.
- Netz: Lidar `192.168.1.162`, Jetson `192.168.1.50/24` am USB-LAN-Adapter
  (NetworkManager-Verbindung `livox-mid360`, statisch, keine Standardroute).
- **RealSense D435i**, SN 262722070636.

Fremdpakete (Livox-SDK2, livox_ros_driver2, realsense-ros) und ihre Build-Besonderheiten: siehe
[README des Workspace](../../README.md).
