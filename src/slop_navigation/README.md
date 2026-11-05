# slop_navigation

Karte aufnehmen mit `slam_toolbox`, in der gespeicherten Karte lokalisieren mit `map_server` + AMCL (später Nav2).

## Karte aufnehmen

```bash
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py   # Antrieb, Modell, EKF
ros2 launch slop_navigation mapping.launch.py                                # LIO + /scan + slam_toolbox + RViz
ros2 run slop_teleop keyboard_teleop                                         # fahren
```

```
/scan (Lidar, 360°, Höhe 0,15–1,8 m) ─┐
TF odom → base_footprint (EKF) ───────┴─► slam_toolbox ─► /map, TF map → odom
```

- `mapping.launch.py` startet `lio.launch.py` mit `map_tf:=false`: Beim Kartieren besitzt `slam_toolbox` das TF
  `map → odom`. Die LIO-Brücke liefert dann nur noch die Geschwindigkeit an den EKF; dadurch ist die Odometrie, auf der
  `slam_toolbox` aufsetzt, schon sehr gut.
- Einstellungen: `config/slam_mapping.yaml` (Grundlage `mapper_params_online_async.yaml`; neuer Scan in die Karte alle
  0,2 m oder 0,3 rad, Karte alle 2 s aktualisiert, 5 cm je Zelle).
- RViz (`slop_description/rviz/mapping.rviz`, Fixed Frame `map`, Draufsicht): Karte, Roboter, Scan (rot), Posegraph und
  das Bedienfeld von slam_toolbox.

So wird die Karte gut:
1. Beim Start 2–3 s stillstehen (LIO-Initialisierung).
2. Langsam fahren (Tempo im Teleop mit `-` auf 30–50 %) und langsam drehen. Der rote Scan muss auf den Wänden der Karte
   liegen bleiben; verschmiert er, langsamer fahren.
3. Jeden Raum einmal ganz abfahren, Türen langsam durchfahren.
4. Zum Schluss an den Startpunkt zurück: Dann schließt `slam_toolbox` die Schleife und zieht die Karte gerade
   (im Posegraph erscheint eine Kante zum Anfang).
5. Speichern (beides, solange `mapping.launch.py` noch läuft), Namen frei wählen:

```bash
mkdir -p ~/slop-bot_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/slop-bot_ws/maps/wohnung
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson/slop-bot_ws/maps/wohnung'}"
```

- `wohnung.pgm` + `wohnung.yaml`: Bild der Karte für Nav2 (map_server, AMCL).
- `wohnung.posegraph` + `wohnung.data`: Posegraph für slam_toolbox (Lokalisieren oder Weiterkartieren).

Getestet am 2026-11-05 im Stand ohne Motoren: `/scan` mit 721 Strahlen wird angenommen, `/map` erscheint, `map → odom`
sendet nur `slam_toolbox` (`lio_bridge`: `publish_map_odom: false`). Erste Karte: `maps/office` (21 × 24 m, ca. 65 m² befahren).

## Lokalisieren in der gespeicherten Karte

```bash
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py   # Antrieb, Modell, EKF
ros2 launch slop_navigation localization.launch.py                           # LIO + /scan + map_server + AMCL + RViz
ros2 run slop_teleop keyboard_teleop                                         # fahren
```

```
maps/office.yaml ─► map_server ─► /map ─┐
/scan ──────────────────────────────────┼─► AMCL ─► TF map → odom, /amcl_pose, /particle_cloud
TF odom → base_footprint (EKF) ─────────┘
```

- Standardkarte `~/slop-bot_ws/maps/office.yaml`, andere mit `map:=$HOME/slop-bot_ws/maps/<name>.yaml` (voller Pfad, `~`
  geht nicht).
- **Startpose:** (0, 0, 0) = Stelle und Richtung, an der das Kartieren begann. Steht der Roboter woanders:
  `x:=… y:=… yaw:=…` mitgeben oder in RViz mit **2D Pose Estimate** den Pfeil an die Stelle und in Blickrichtung ziehen.
  AMCL rechnet erst nach 0,1 m oder 0,15 rad Bewegung weiter; im Stand nachschärfen mit
  `ros2 service call /request_nomotion_update std_srvs/srv/Empty` (mehrmals).
- AMCL sendet `map → odom`, `lio.launch.py` läuft mit `map_tf:=false`. Einstellungen: `config/localization.yaml`
  (Odometrie-Rauschen 0,1 statt 0,2, weil die EKF-Odometrie mit LIO gut ist; 60 von 721 Strahlen; 500–2000 Partikel).
- RViz (`slop_description/rviz/localization.rviz`): Karte, Roboter, Scan (rot, soll auf den Wänden liegen), Partikel
  (blau, eng beisammen = sicher), Pose mit Unsicherheitsellipse; Werkzeug „2D Pose Estimate“.

Getestet am 2026-11-05 im Stand ohne Motoren, Roboter am Startpunkt der Karte: 90 % der Scanpunkte auf einer Wand der
Karte (±10 cm). Startpose absichtlich 30 cm und 10° daneben gesetzt: nach 25 Nachrechnungen im Stand bei
(−0,07 m, 0,02 m, 2°), 96 % auf der Wand, Unsicherheit 7 cm / 3°. Last: AMCL 4,5 %, map_server 2 % eines Kerns.
