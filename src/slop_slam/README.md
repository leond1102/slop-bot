# slop_slam

LiDAR-Inertial-Odometrie mit FAST-LIO2 und dem Livox Mid-360s.

```bash
ros2 launch slop_slam lio.launch.py                 # Sensoren + FAST-LIO + lio_tf + RViz
ros2 launch slop_slam lio.launch.py camera:=false rviz:=false
ros2 service call /map_save std_srvs/srv/Trigger    # Karte → ~/slop-bot_ws/maps/lio_map.pcd
```

Beim Start den Roboter 2–3 s still und waagrecht stehen lassen (IMU-Initialisierung von FAST-LIO und
Montage aus der Schwerkraft von slop_sensors/mount_tf).

## TF

```
map ─┬─ camera_init ── body              (FAST-LIO intern, statisch an map gehängt)
     └─ odom ── base_footprint ── livox_frame, camera_link ...
```

- `map` = Roboterlage beim Start.
- **Antrieb läuft** (slop_base sendet `odom → base_footprint`): `lio_tf` sendet `map → odom` und korrigiert so
  die Drift der Radodometrie.
- **Antrieb läuft nicht**: `lio_tf` sendet `map → odom` = Identität und `odom → base_footprint` selbst.
- Den Fall erkennt `lio_tf` automatisch (fremdes `odom → base_footprint` auf /tf in den letzten 0,5 s).
- `/lio/odom` (nav_msgs/Odometry): Pose von base_footprint in map, ~10 Hz.

## Topics von FAST-LIO

`/Odometry` (IMU-Pose in camera_init), `/cloud_registered` (aktueller Scan in der Welt), `/Laser_map`
(Karte, jede Sekunde ein Scan dazu, wächst unbegrenzt), `/path`.

## Fremdpaket FAST_LIO (src/third_party/FAST_LIO, Branch ROS2)

Lokal geändert (`git diff` in src/third_party/FAST_LIO):
- `pcl_ros` → `tf2_ros`: pcl_ros ist nicht installiert und wird im Code nicht benutzt.
- C++14 → C++17: ROS 2 Humble braucht C++17. Vorher kam das indirekt über pcl_ros.

Config: [config/fast_lio_mid360s.yaml](config/fast_lio_mid360s.yaml). Extrinsik IMU↔Lidar wie Mid-360
(nicht geschätzt, `extrinsic_est_en: false`).

## Bekannte Falle

`~/.bashrc` stellt `/opt/MVS/lib/aarch64` (Hikrobot-MVS-SDK) an den Anfang von `LD_LIBRARY_PATH`. Dessen alte
libusb hat kein `libusb_set_option`, damit stürzt FAST-LIO (über PCL) beim Start ab. `lio.launch.py` und
`slop_sensors/camera.launch.py` starten die Nodes deshalb ohne /opt/MVS im Pfad.
