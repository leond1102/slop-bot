# slop_navigation

Karte aufnehmen mit `slam_toolbox` (später: Lokalisierung in der gespeicherten Karte und Nav2).

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

## Getestet (2026-11-05, im Stand, ohne Motoren)

`/scan` mit 721 Strahlen wird angenommen, `/map` erscheint, `map → odom` sendet nur `slam_toolbox`
(`lio_bridge`: `publish_map_odom: false`).
