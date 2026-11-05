# slop_lio

LiDAR-Inertial-Odometrie mit FAST-LIO2 (Livox Mid-360s) und eine Brücke zu TF und EKF.

```bash
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py   # Antrieb, Modell, EKF
ros2 launch slop_lio lio.launch.py                                           # Lidar + FAST-LIO + Brücke
ros2 launch slop_lio lio.launch.py lidar:=false clouds:=true                 # Lidar läuft schon; Wolke für RViz
```

Beim Start den Roboter 2–3 s stillstehen lassen (IMU-Initialisierung von FAST-LIO).

## Datenfluss

```
Livox ─ /livox/lidar (CustomMsg) ─┐
      └ /livox/imu ───────────────┴─► fast_lio ── /Odometry (IMU-Pose in camera_init)
                                                      │
                                              lio_bridge (slop_lio)
                                       ┌──────────────┼───────────────────┐
                                  /lio/odom       TF map → odom     TF map → camera_init
                                       │
                                  EKF (odom1: nur Geschwindigkeit vx, vyaw)
```

- **EKF besitzt `odom → base_footprint`**: stetig, aus Rädern, Livox-Gyro und der LIO-Geschwindigkeit.
- **`map → odom`** kommt aus der LIO-Pose: `map_T_odom = map_T_base · odom_T_base⁻¹` (nur x, y, Gier). Drift und Sprünge von LIO
  landen dort und nicht in `odom`.
- **Warum nur die Geschwindigkeit in den EKF?** Die Pose in `map` bräuchte das TF `map → odom`, und das wird aus dem EKF-Ergebnis
  gebildet (Zirkelbezug). Die Geschwindigkeit im Roboterframe braucht kein TF. FAST-LIO liefert selbst keine; die Brücke
  rechnet sie aus aufeinanderfolgenden Posen (10 Hz).
- **`map`** = Lage von `base_footprint` beim ersten LIO-Wert (Position, Gier). Roll/Pitch bleiben schwerkraftausgerichtet.
- Das Modell (robot_state_publisher, im Bringup) liefert `base_footprint → livox_frame`; die Sensorneigung kommt aus
  `calib/mounts.yaml` (`ros2 run slop_calibration calibrate mount`).
- Die Extrinsik Lidar↔IMU (`extrinsic_T`) steht nur in `config/fast_lio_mid360s.yaml`; `lio.launch.py` reicht sie an die Brücke.

## FAST-LIO (src/third_party/FAST_LIO)

Submodule (Branch ROS2, Commit cc3c9e8) mit lokalen Änderungen in `src/third_party/patches/fast_lio.patch`, die
`colcon_build.sh` automatisch anwendet: `pcl_ros` → `tf2_ros` (pcl_ros ist nicht installiert), C++14 → C++17 (ROS 2 Humble) und
der Parameter `preprocess.exclude_boxes` (Quader im Lidar-Frame, deren Punkte vor der Registrierung verworfen werden).
Nicht übernommen vom alten Stand auf `main`: Komponenten-Container (Lidar-Treiber und FAST-LIO intra-process) und 16-Byte-Punkte;
beides sind spätere Optimierungen.

## Körper und Mast aus den Scans schneiden

`lio.launch.py` schneidet zwei Quader aus jedem Scan, bevor FAST-LIO ihn sieht (Einstellungen: `config/crop.yaml`,
aus mit `crop:=false`). Beide im Lidar-Frame (Ursprung = Mitte des Lidars):
- **Roboterkörper** mit 10 % Zuschlag auf die halbe Breite links und rechts (+ 3 cm oben). Maße und Lage kommen aus dem URDF
  (`chassis`, `chassis_joint`, `livox_joint`), stehen also nur dort.
- **Quader hinter dem Lidar** ab dessen Mitte: 0,40 m nach hinten, 0,15 m nach links und rechts, 1 m nach oben (Mast und Fahne;
  Werte in `config/crop.yaml`). Die Fahne reicht bis 0,57 m nach hinten, für sie ganz bräuchte die Tiefe mindestens 0,6 m.

In RViz erscheinen die Quader als durchsichtige Würfel (`/lio/crop_boxes`, orange = Körper, rot = hinter dem Lidar) in der
Ansicht `rviz_config:=lio` (Fixed Frame `map`, dazu LIO-Wolke, LIO-Pose grün und EKF-Pose gelb im Vergleich).

Eine Neigung des Lidars (wenige Grad) bleibt unberücksichtigt. Geprüft mit einer synthetischen Szene in einer abgetrennten
ROS-Domäne: ohne Ausschneiden lagen 354 Punkte in den Quadern, mit Ausschneiden 0, alle anderen blieben erhalten.

## Bekannte Grenzen / offen

- Livox ohne PTP: Zeitstempel sind Empfangszeit; zu Beginn weicht der IMU-Stempel einige Sekunden von der Jetson-Uhr ab
  (Warnung im `imu_adapter`), danach nicht mehr. Besser wäre die Gerätezeit der Livox (Treiber-Patch).
- `extrinsic_T` ist der Wert des Mid-360, für das Mid-360s nicht verifiziert.
- Last im Stand (Orin NX, alle Werte in % eines Kerns): FAST-LIO 20–40, Livox-Treiber 14–58 (schwankt stark), EKF 6–10,
  `imu_adapter` ca. 31 (Python-Empfang der 200-Hz-IMU, mit C++ deutlich kleiner), `lio_bridge` 11–13. Unter Fahrt noch nicht gemessen.
