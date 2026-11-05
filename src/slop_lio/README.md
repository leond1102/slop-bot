# slop_lio

LiDAR-Inertial-Odometrie mit FAST-LIO2 (Livox Mid-360s) und eine Brücke zu TF und EKF.

```bash
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py   # Antrieb, Modell, EKF
ros2 launch slop_lio lio.launch.py                                           # Lidar + FAST-LIO + Brücke
ros2 launch slop_lio lio.launch.py lidar:=false clouds:=true                 # Lidar läuft schon; Wolke für RViz
ros2 launch slop_lio lio.launch.py scan:=false                               # ohne /livox/points und /scan
```

`lio.launch.py` startet außerdem den 2D-Scan für slam_toolbox und Nav2 (`livox_to_cloud` → `/livox/points` →
`pointcloud_to_laserscan` → `/scan`, mit denselben ausgeschnittenen Quadern wie FAST-LIO). Details in
[slop_sensors/README.md](../slop_sensors/README.md).

Beim Start den Roboter 2–3 s stillstehen lassen (IMU-Initialisierung von FAST-LIO). Läuft der Lidar-Treiber schon
(`sensors.launch.py`), `lidar:=false` setzen, und der Treiber muss dann im Livox-Format laufen (`xfer_format:=1`, Standard).

## Datenfluss

```
Livox ─ /livox/lidar (CustomMsg) ─┐
      └ /livox/imu ───────────────┴─► fast_lio ── /Odometry (IMU-Pose in camera_init)
                                                      │
                                              lio_bridge (slop_lio)
                                       ┌──────────────┼───────────────────┐
                                  /lio/odom       TF map → odom     TF static map → camera_init
                                       │
                                  EKF (odom1: nur Geschwindigkeit vx, vyaw)
```

- **EKF besitzt `odom → base_footprint`**: stetig, aus Rädern, Livox-Gyro und der LIO-Geschwindigkeit.
- **`map → odom`** kommt aus der LIO-Pose: `map_T_odom = map_T_base · odom_T_base⁻¹` (nur x, y, Gier). Drift und Sprünge von LIO
  landen dort und nicht in `odom`.
- **Warum nur die Geschwindigkeit in den EKF?** Die Pose in `map` bräuchte das TF `map → odom`, und das wird aus dem EKF-Ergebnis
  gebildet (Zirkelbezug). Die Geschwindigkeit im Roboterframe braucht kein TF. FAST-LIO liefert selbst keine; die Brücke
  rechnet sie aus der aktuellen Pose und der von vor `twist_window` Scans (Standard 3 = 0,3 s).
- **`camera_init`** (Welt von FAST-LIO) ist der IMU-Frame beim Start, also **nicht** nach der Schwerkraft ausgerichtet, sondern um
  die Neigung des Lidars gekippt (am Roboter ca. 1,3–1,9° Pitch).
- **`map`** = Lage von `base_footprint` beim ersten LIO-Wert, mit voller Drehung. Auf ebenem Boden ist `map` damit waagrecht.
  Die Brücke meldet beim Start, wie schief der Roboter in `camera_init` steht. `map → camera_init` ist danach fest (statisches TF).
- Das Modell (robot_state_publisher, im Bringup) liefert `base_footprint → livox_frame`; die Sensorneigung kommt aus
  `calib/mounts.yaml` (`ros2 run slop_calibration calibrate mount`).
- Die Extrinsik Lidar↔IMU (`extrinsic_T`, `extrinsic_R`) steht nur in `config/fast_lio_mid360s.yaml`; `lio.launch.py` reicht
  beide an die Brücke.

## FAST-LIO (src/third_party/FAST_LIO)

Submodule (Branch ROS2, Commit cc3c9e8) mit lokalen Änderungen in `src/third_party/patches/fast_lio.patch`, die
`colcon_build.sh` automatisch anwendet:

- `pcl_ros` → `tf2_ros` (pcl_ros ist nicht installiert), C++14 → C++17 (ROS 2 Humble).
- Parameter `preprocess.exclude_boxes`: Quader im Lidar-Frame, deren Punkte vor der Registrierung verworfen werden.
- **QoS der Abos:** Lidar „best effort“ statt „reliable“, IMU mit Tiefe 1000 statt 10. Mit „reliable“ kam die Odometrie
  1,1–1,9 s zu spät, weil DDS bei den ~400 KB großen Scans verlorene Teile nachliefert und die folgenden Scans so lange
  zurückhält. Die IMU-Tiefe 10 entsprach nur 50 ms bei 200 Hz. Danach: Verzögerung 22 ms, Zittern der Pose im Stand halbiert.

Nach einer Änderung im Submodule den Patch neu erzeugen:
`git -C src/third_party/FAST_LIO diff > src/third_party/patches/fast_lio.patch`

Nicht übernommen vom alten Stand auf `main`: Komponenten-Container (Lidar-Treiber und FAST-LIO intra-process) und 16-Byte-Punkte;
beides sind spätere Optimierungen.

## Körper und Mast aus den Scans schneiden

`lio.launch.py` schneidet zwei Quader aus jedem Scan, bevor FAST-LIO ihn sieht (Einstellungen: `config/crop.yaml`,
aus mit `crop:=false`). Beide im Lidar-Frame (Ursprung = Mitte des Lidars):
- **Roboterkörper** mit 10 % Zuschlag auf die halbe Breite links und rechts (+ 3 cm oben). Maße und Lage kommen aus dem URDF
  (`chassis`, `chassis_joint`, `livox_joint`), stehen also nur dort.
- **Quader hinter dem Lidar** ab dessen Mitte: 0,60 m nach hinten, 0,15 m nach links und rechts, 0,45 m nach oben (Mast und
  Fahne; Werte in `config/crop.yaml`). Bis 2026-11-05 nur 0,40 m tief: Die Fahne lag dann im Nav2-Footprint.

`preprocess.blind` (0,5 m) verwirft ohnehin alles näher als 0,5 m. Beide Quader wirken deshalb nur auf Punkte, die weiter weg
liegen. Laut Messung im Stand (2026-11-05) liegen alle Körperpunkte innerhalb von 0,5 m. Hinter dem Quader, bei x = −0,4 bis −0,75 m
und bis 0,4 m über dem Lidar, kommen aber noch ca. 60–200 Punkte je Scan bei FAST-LIO an. Ob das Mast und Fahne sind oder
Gegenstände hinter dem Roboter, ist offen. Wandern sie in RViz (`rviz_config:=lio`) mit dem Roboter mit, die Tiefe in
`rear_box` auf ca. 0,8 m erhöhen.

In RViz erscheinen die Quader als durchsichtige Würfel (`/lio/crop_boxes`, orange = Körper, rot = hinter dem Lidar,
`frame_locked`, fahren also mit dem Lidar mit) in der Ansicht `rviz_config:=lio` (Fixed Frame `map`, dazu LIO-Wolke, LIO-Pose grün und EKF-Pose gelb im Vergleich).

Eine Neigung des Lidars (wenige Grad) bleibt unberücksichtigt.

## Messwerte im Stand (2026-11-05, Roboter still, Motoren aus)

| | Wert |
|---|---|
| Verzögerung `/Odometry` bzw. `/lio/odom` gegen die Uhr | 22–26 ms (vorher 1100–1900 ms) |
| Positionssprung der LIO-Pose je Scan | Median 4,6 mm, max. 12 mm (vorher 8,2 / 36 mm) |
| LIO-Geschwindigkeit (`twist_window` 3) | Streuung 0,013 m/s, max. 0,037 m/s; Drehrate 0,006 rad/s |
| EKF-Geschwindigkeit `/odom` | Streuung 0,013 m/s |
| Lidar-Scan beim Empfänger | 104 ms nach Scanbeginn, also ~4 ms nach dem letzten Punkt |

## Bekannte Grenzen / offen

- Livox ohne PTP: Zeitstempel sind Empfangszeit am Jetson. Zu Beginn sind IMU-Werte einige Sekunden alt (Warnung im
  `imu_adapter`), danach ca. 1 ms. Besser wäre die Gerätezeit der Livox (Treiber-Patch).
- `extrinsic_T` / `extrinsic_R` sind die Werte des Mid-360, für das Mid-360s nicht verifiziert.
- Die EKF-Gier driftet im Stand um ca. 0,04°/s (Rest-Bias des Gyros). `map` bleibt davon unberührt, die Drift landet in
  `map → odom`. Hilft: `ros2 run slop_calibration calibrate gyro` im warmen Zustand.
- Unter Fahrt noch nicht gemessen.
