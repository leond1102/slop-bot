# slop_sensors

Startet nur die Sensortreiber des Slop-Bots. Keine TFs zwischen Roboter und Sensoren, keine Filter oder
Berechnungen: Grundlage für einen neuen SLAM-Aufbau.

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

TFs kommen nur vom RealSense-Treiber selbst (`camera_link` → Sensor- und optische Frames der Kamera).
Zwischen `livox_frame`, `camera_link` und dem Roboter gibt es keine Verbindung.

## Hardware

- **Livox Mid-360s** (SDK-Typ 35, *nicht* Mid-360): braucht `config/MID360s_config.json`
  im `Mid360s`-Format. Mit einer `MID360`-Config verwirft das SDK die Antwort des Lidars ohne Fehlermeldung.
- Netz: Lidar `192.168.1.162`, Jetson `192.168.1.50/24` am USB-LAN-Adapter
  (NetworkManager-Verbindung `livox-mid360`, statisch, keine Standardroute).
- **RealSense D435i**, SN 262722070636.

Fremdpakete (Livox-SDK2, livox_ros_driver2, realsense-ros) und ihre Build-Besonderheiten: siehe
[README des Workspace](../../README.md).
