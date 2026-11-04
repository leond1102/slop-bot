# slop_slam

LiDAR-Inertial-Odometrie mit FAST-LIO2 und dem Livox Mid-360s.

```bash
ros2 launch slop_slam lio.launch.py                 # Sensoren + FAST-LIO + lio_tf + RViz
ros2 launch slop_slam lio.launch.py camera:=false rviz:=false
ros2 service call /lio_map/save std_srvs/srv/Trigger   # Karte → ~/slop-bot_ws/maps/lio_map.pcd
```

Beim Start den Roboter 2–3 s still stehen lassen (IMU-Initialisierung von FAST-LIO). Die Sensor-Neigung wird beim
Start nicht gemessen; neu messen nur bei Bedarf (still und waagrecht):
`ros2 service call /mount_tf/recalibrate std_srvs/srv/Trigger` (wird in `~/slop-bot_ws/calib/mounts_lio.yaml` gespeichert).

## TF

```
map ─┬─ camera_init ── body              (FAST-LIO intern, statisch an map gehängt)
     └─ odom ── base_footprint ── livox_frame, camera_link ...
```

- `map` = Roboterlage beim Start.
- **Keine Radodometrie** (zu ungenau): Die Roboterpose kommt allein aus LIO. `lio_tf` sendet `map → odom` statisch
  als Identität und `odom → base_footprint` selbst.
- **Verzögerung:** Eine LIO-Pose ist beim Eintreffen ~0,1–0,2 s alt. `lio_tf` sendet sie zu ihrer Scanzeit und
  rechnet dazwischen mit 30 Hz bis „jetzt“ voraus (Geschwindigkeit der letzten 0,5 s, konstant im Roboterframe),
  höchstens 0,5 s weit. Hängt LIO länger, bleibt die TF stehen. Parameter `predict_rate`, `max_predict`,
  `velocity_window`; `predict_rate: 0` = ohne Vorausrechnung.
- `/lio/odom` (nav_msgs/Odometry): Pose von base_footprint in map (~10 Hz, Scanzeit), mit Geschwindigkeit.

## Montage laufend nachführen (mount_calib)

Nur mit `ros2 launch slop_slam lio.launch.py calibrate:=true` (kostet ~30 % eines Kerns, v. a. der ICP; ohne lädt
mount_tf die zuletzt gespeicherte Montage). FAST-LIO sendet `/cloud_registered_body` nur in diesem Modus:

- **RealSense, Drehung:** alle 2 s wird das Tiefenbild (bis 3 m) per ICP an die vollen Lidar-Scans der letzten
  3 s (`/cloud_registered_body`) angepasst. Übernommen wird nur die **Drehung** (roll/pitch/yaw): Die Position ist
  aus ICP nur auf ~5–10 cm bestimmt (systematischer Tiefenfehler der D435i, ein Blickwinkel), die Drehung auf
  ~1–2°. Die Position kommt aus `mounts.yaml` (nachmessen!); weicht ICP > 15 cm davon ab, steht eine Warnung
  im Log. Ganze Lage nachführen: Parameter `camera_translation: true`. Übernommen nur, wenn der Roboter langsam
  dreht, ≥ 30 % der Kamerapunkte einen Lidar-Partner < 5 cm haben, rmse ≤ 3,5 cm, die Szene Struktur in alle
  Richtungen hat und die Korrektur nicht springt. Bilanz alle 30 s im Log.
- **Lidar, yaw:** fährt der Roboter laut Fahrbefehl (`/cmd_vel`) und LIO geradeaus, zeigt die LIO-Bewegung im
  IMU-Frame die Blickrichtung des Lidars (Differentialantrieb fährt nur entlang x). Braucht Fahren, ab 15 cm.
  roll/pitch bleiben aus der Schwerkraft, x/y/z aus mounts.yaml.
- Ergebnis geht an `mount_tf` (Anzeige in RViz: Quelle „LIO“) und nach `~/slop-bot_ws/calib/mounts_lio.yaml`.
  mount_tf lädt daraus beim nächsten Start die Drehung (die Position immer aus mounts.yaml). Dieselbe Datei
  schreibt auch `/mount_tf/recalibrate` (Schwerkraft).
- `lio_tf` friert `map` nach 10 s ein, damit spätere Korrekturen die Karte nicht mitdrehen.

Gemessen am 2026-11-03 (Stillstand): Kamera pitch −8,6° (Schwerkraft und ICP einig), yaw ~0°. Position laut ICP
grob x −0,1…−0,2, y 0…+0,12, z 0,35…0,46 m je nach Aufnahme → in mounts.yaml x −0,12 / y 0,06 / z 0,40 als Startwert.

## Topics von FAST-LIO

`/Odometry` (IMU-Pose in camera_init), `/cloud_registered` (aktueller Scan in der Welt, alle Punkte), `/path`.
`/Laser_map` ist aus (wuchs unbegrenzt, RViz hing). Stattdessen `slop_slam/lio_map`: Voxelkarte (5 cm) auf `/lio/map`
alle 2 s, speichern mit `ros2 service call /lio_map/save std_srvs/srv/Trigger`, leeren mit `/lio_map/clear`.

## Fremdpaket FAST_LIO (src/third_party/FAST_LIO, Branch ROS2)

Lokal geändert (`git diff` in src/third_party/FAST_LIO):
- Zusätzlich als Komponente gebaut: Bibliothek `fastlio_component`, Plugin `LaserMappingNode` (gleiche Quelle,
  `main()` per `#ifndef FASTLIO_COMPONENT` ausgeblendet). `lio.launch.py` lädt sie mit dem Livox-Treiber in den
  Container `lio_container` (`component_container_mt`, intra-process). Die Executable `fastlio_mapping` gibt es
  weiter (`composable:=false`).
- `pcl_ros` → `tf2_ros`: pcl_ros ist nicht installiert und wird im Code nicht benutzt.
- C++14 → C++17: ROS 2 Humble braucht C++17. Vorher kam das indirekt über pcl_ros.
- Alle Punktwolken gehen schlank raus: nur x, y, z, intensity = 16 Byte/Punkt (`to_slim_msg`), statt 48 Byte
  (internes `PointXYZINormal` über `pcl::toROSMsg`). `/cloud_registered` 256 → 86 kB je Scan.
- IMU-Abo mit Tiefe 1000 statt 10: Die Callbacks teilen sich die Callback-Gruppe mit der Rechenschleife,
  dauert ein Scan länger als 50 ms, gingen sonst IMU-Messungen verloren.

`lio_container` läuft mit `thread_num: 2` (Standard wäre ein Thread je Kern, kostete ~20 % CPU fürs Aufwecken).

Config: [config/fast_lio_mid360s.yaml](config/fast_lio_mid360s.yaml). Extrinsik IMU↔Lidar wie Mid-360
(nicht geschätzt, `extrinsic_est_en: false`).

## Bekannte Falle

`~/.bashrc` stellt `/opt/MVS/lib/aarch64` (Hikrobot-MVS-SDK) an den Anfang von `LD_LIBRARY_PATH`. Dessen alte
libusb hat kein `libusb_set_option`, damit stürzt FAST-LIO (über PCL) beim Start ab. `lio.launch.py` und
`slop_sensors/camera.launch.py` starten die Nodes deshalb ohne /opt/MVS im Pfad.
