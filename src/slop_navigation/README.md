# slop_navigation

Karte aufnehmen mit `slam_toolbox`, in der gespeicherten Karte lokalisieren mit `map_server` + AMCL + `relocalize`,
navigieren mit Nav2 (Costmaps aus Lidar und Kamera, NavFn, Regulated Pure Pursuit, Collision Monitor).

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
ros2 run nav2_map_server map_saver_cli -f ~/slop-bot_ws/maps/wohnung --free 0.196
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/jetson/slop-bot_ws/maps/wohnung'}"
```

- `wohnung.pgm` + `wohnung.yaml`: Bild der Karte für Nav2 (map_server, AMCL). `--free 0.196` ist wichtig: Ohne schreibt
  map_saver_cli `free_thresh: 0.25`, und beim Laden zählen die unbekannten Zellen (Grauwert 205 = 0,196) als frei.
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
- **Startpose:** AMCL startet bei (0, 0, 0) = Stelle und Richtung, an der das Kartieren begann (oder `x:=… y:=… yaw:=…`).
  Passt das nicht, findet `relocalize` den Roboter selbst (siehe unten). Von Hand: in RViz **2D Pose Estimate**.
  AMCL rechnet erst nach 0,1 m oder 0,15 rad Bewegung weiter; im Stand nachschärfen mit
  `ros2 service call /request_nomotion_update std_srvs/srv/Empty` (mehrmals).
- AMCL sendet `map → odom`, `lio.launch.py` läuft mit `map_tf:=false`. Einstellungen: `config/localization.yaml`
  (Odometrie-Rauschen 0,1 statt 0,2, weil die EKF-Odometrie mit LIO gut ist; 60 von 721 Strahlen; 500–2000 Partikel).
- RViz (`slop_description/rviz/localization.rviz`): Karte, Roboter, Scan (rot, soll auf den Wänden liegen), Partikel
  (blau, eng beisammen = sicher), Pose mit Unsicherheitsellipse; Werkzeug „2D Pose Estimate“.

## Automatische Relokalisierung (`relocalize`)

AMCL verfolgt die Pose nur. Wird der Roboter versetzt (getragen, geschoben; die Räder zählen nur ausgegebene Schritte) oder
steht er beim Start nicht am Kartenstart, liegen alle Partikel falsch und AMCL findet von selbst nicht zurück. Nav2 hat
dafür keine Standard-Node; `relocalize` (startet mit `localization.launch.py`, aus mit `relocalize:=false`) macht das:

- **Erkennen:** jede Sekunde der Anteil der Scanpunkte, die in der Karte auf einer Wand liegen (±10 cm), auf
  `/relocalize/score`. Richtige Pose: 90–99 %, versetzt: 15–50 %.
- **Beim Start**, sobald AMCL läuft: passt die Startpose (≥ 80 %), nichts tun; sonst die zuletzt gespeicherte Pose
  (`maps/<karte>.last_pose.yaml`, nicht in Git) fein nachsuchen; passt die auch nicht, die ganze Karte durchsuchen.
- **Im Betrieb:** unter 60 % für 3 s → die ganze Karte durchsuchen; danach 10 s Pause.
- **Suche** (`relocalize_core.py`): grob jede 4. freie Zelle (20 cm) in 5°-Schritten, dann fein um die 12 besten Orte
  (5 cm, 1°). Gesetzt wird nur, wenn der beste Ort ≥ 80 % erreicht und ≥ 8 Prozentpunkte vor dem besten *anderen* Ort
  (> 0,5 m oder > 20° entfernt) liegt; sonst Warnung im Log und die Pose bleibt (z. B. in gleich aussehenden Fluren).
  Liegt das Ergebnis an der aktuellen Stelle (Umgebung nur verändert: Menschen, Möbel), bleibt die Pose ebenfalls.
- **Letzte Pose:** alle 2 s gespeichert, wenn sie passt, und beim Beenden.
- Von Hand suchen: `ros2 service call /relocalize std_srvs/srv/Trigger`.
- Zusätzlich ist in AMCL die eingebaute Wiederfindung an (`recovery_alpha_slow/fast` 0,001 / 0,1): Sie streut beim Fahren
  zufällige Partikel, wenn der Scan schlechter passt.

Getestet am 2026-11-05 mit dem echten Lidar, ohne Motoren, Roboter von Hand an eine andere Stelle gestellt:

| Fall | Ablauf | Ergebnis |
|---|---|---|
| Start an fremder Stelle | Startpose passt zu 25 % → Suche 5,3 s | (0,18 m, 0,56 m, −57°), 100 % gegen 88 %; AMCL danach 99 % |
| AMCL absichtlich auf (−6 m, 0, 0°) gesetzt | 16 % → nach 3 s Suche 5,5 s | wieder (0,13 m, 0,61 m, −58°), AMCL 99 % |
| Neustart | gespeicherte Pose passt (100 %) | sofort gesetzt, ohne Suche; AMCL 96 % |

Die Suche dauert auf dem Jetson 2–6 s (mit laufendem LIO eher 5 s) und blockiert solange nur die Node selbst.

## Erster Test der Lokalisierung

Getestet am 2026-11-05 im Stand ohne Motoren, Roboter am Startpunkt der Karte: 90 % der Scanpunkte auf einer Wand der
Karte (±10 cm). Startpose absichtlich 30 cm und 10° daneben gesetzt: nach 25 Nachrechnungen im Stand bei
(−0,07 m, 0,02 m, 2°), 96 % auf der Wand, Unsicherheit 7 cm / 3°. Last: AMCL 4,5 %, map_server 2 % eines Kerns.

## Navigation (Nav2)

```bash
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py   # Antrieb, Modell, EKF
ros2 launch slop_sensors camera.launch.py                                    # Kamera + Hinderniserkennung
ros2 launch slop_navigation navigation.launch.py                             # Lokalisierung + Nav2 + RViz
```

Ziel in RViz mit **Nav2 Goal** setzen (Pfeil an die Stelle und in Zielrichtung ziehen). Die Tastatur hat immer Vorrang
(twist_mux: Tastatur 50, Nav2 20), Hand-Stopp und RustDesk-Sperre (100) sperren auch Nav2.
Einstellungen: `config/nav2.yaml`, Verhaltensbäume: `behavior_trees/`, RViz: `slop_description/rviz/navigation.rviz`.

### Footprint

Körper 50 × 32 cm, symmetrisch, beginnt 10 cm vor der Antriebsachse (= Drehpunkt = `base_footprint`):
`[[0.10, 0.16], [0.10, -0.16], [-0.40, -0.16], [-0.40, 0.16]]`, dazu 2 cm Zuschlag. Innenkreis 0,16 m, die hinteren Ecken
liegen 0,43 m vom Drehpunkt: Beim Drehen auf der Stelle schwenkt das Heck weit aus. Dieselben Maße stehen im URDF
(`slop_bot.urdf.xacro`: `body_front`, `body_length`, `body_width`) und bestimmen den Quader, der den Roboter aus
Lidar-Wolke und `/scan` schneidet.

### Wie die Costmaps funktionieren

Eine Costmap ist ein Raster (5 cm) mit Kosten 0–254 je Zelle: 254 = Hindernis, 253 = der Roboter-Mittelpunkt hier hieße
Berührung (Innenkreis), darunter abnehmende Kosten nach außen, 0 = frei, 255 = unbekannt. Sie entsteht aus Schichten:

| Schicht | Inhalt | Wo |
|---|---|---|
| `static_layer` | die gespeicherte Karte (`/map`) | nur global |
| `obstacle_layer` | Sensoren: **markieren** setzt Zellen mit Messpunkten auf Hindernis, **freiräumen** (Raytracing) setzt alle Zellen zwischen Sensor und Messpunkt auf frei | global und lokal |
| `inflation_layer` | Sicherheitsabstand: um jedes Hindernis fallen die Kosten bis 0,45 m ab (`cost_scaling_factor` 4) | global und lokal |

- **Global** (`map`, ganze Karte, 1 Hz): Grundlage für den **Planer** (NavFn/A*), der den Weg mit den geringsten Kosten
  sucht. Er plant nicht durch unbekannte Bereiche (`allow_unknown: false`).
- **Lokal** (`odom`, 4 × 4 m mitfahrend, 5 Hz, nur Sensoren): Grundlage für den **Controller**. Er prüft jeden Befehl mit dem
  Footprint gegen diese Karte. `odom` springt nicht, wenn AMCL korrigiert.
- Sensoren: `/scan` (Lidar, markiert und räumt frei, `inf` = bis zur Reichweite frei), `/camera/obstacles` (markiert),
  `/camera/floor` (räumt frei, Raytracing von `camera_link` aus). Ein Hindernis, das weggeht, verschwindet erst, wenn ein
  Sensor durch seine Zelle hindurchsieht. Vor dem Roboter erledigt das die Kamera mit dem freien Boden; niedrige Dinge
  sieht das Lidar erst ab ca. 2,6 m.
- Worauf es ankommt: Footprint richtig; der Roboter darf sich selbst nicht als Hindernis sehen (Ausschnitt-Quader);
  `inflation_radius` größer als der Außenkreis, sonst fährt der Planer zu dicht an Wände; Freiräumen muss möglich sein, sonst
  bleiben „Geister“ (z. B. vorbeigelaufene Menschen) liegen.

### Erfahrungskarte (`experience`)

Lernt bei jeder Fahrt, was in der Karte **fest** (Wand, Tisch), **vielleicht belegt** (Stuhl, Mülleimer) oder **frei**
ist, und gibt „vielleicht“ als weiche Kosten an den Planer. Startet mit `localization.launch.py` (also auch mit
`navigation.launch.py`); ohne: `experience:=false`.

- **Zählen:** je Zelle *gesehen* (Strahl ging hindurch oder traf) und *belegt* (Strahl traf). Beständigkeit
  p = belegt / gesehen. Ab 5 Beobachtungen: p ≥ 0,80 fest, 0,15–0,80 vielleicht, darunter frei.
- **Vergessen:** Halbwertszeit 7 Tage (`half_life_days`; Baustelle eher 1–2). p bleibt dabei gleich, aber neue
  Beobachtungen wiegen mehr: Ein weggeräumter Schrank wird nach einigen Fahrten zu „vielleicht“, dann „frei“.
- **Lernsperre** (sonst schriebe eine falsche Pose Wände an falsche Stellen): nur bei relocalize-Score ≥ 75 %,
  AMCL-Unsicherheit ≤ 0,2 m / 6°, Drehrate ≤ 0,3 rad/s (Scan sonst verzerrt). Langzeit-Zählung nur nach 0,2 m / 20°
  Bewegung oder alle 60 s im Stand, damit ein Mensch neben dem stehenden Roboter nicht hundertfach zählt.
- **Weiche Kosten** (`experience_costs:=true`): Nav2-Keepout-Filter in der globalen Costmap (`experience_filter`). Maske
  1–60 → Kosten bis 150 von 254: teurer, **nie verboten**; der Planer nimmt lieber ~1 m Umweg als eine Stuhl-Stelle.
  Zellen, die in dieser Sitzung zuletzt **frei gesehen** wurden, haben keine Kosten („erst mal schauen“); steht dort
  wirklich etwas, trägt es die Sensorschicht ein.
- **Overwrite** in der globalen Sensorschicht (`combination_method: 0`): Was die Sensoren sehen, ersetzt die Karte auch
  als frei – ein Stuhl, der beim Kartieren im Weg stand, verschwindet für den Planer, sobald das Lidar durchsieht.
- **Karte aktualisieren** (`experience_export`, schreibt neue Dateien, Original bleibt): fest → Wand, frei → frei,
  vielleicht → frei (außer an Wandkanten: in der Karte belegt und eine feste Zelle ≤ 2 Zellen daneben). Nur Zellen mit
  ≥ 10 Beobachtungen. Erfahrung und letzte Pose werden für die neue Karte mitkopiert.
- Dateien: `maps/<karte>.experience.npz` (jede Minute und beim Beenden). Passt das Raster nicht mehr (neue Karte unter
  altem Namen), wird die alte Datei als `.alt` beiseitegelegt und neu begonnen.
- Anzeige: RViz „Erfahrung (Beständigkeit)“ (`/experience/persistence`, Standard aus).
- Grenzen: nur das Höhenband des Lidars (0,15–1,8 m); Kamera noch nicht. Taugt erst nach einigen Fahrten.
- Getestet (2026-11-05, Domäne 78, Aufnahme `~/nav_test`, ohne Antrieb): lernt aus echten Scans, Maske kommt im
  Keepout-Filter des `planner_server` an; Maskenwert 40 auf freiem Gang → Costmap 40 (Kosten 102), nicht gesperrt.
  16 Unit-Tests (`test/test_experience.py`, `test/test_relocalize.py`).

### Sicherheit

```
controller_server / behavior_server ─cmd_vel_nav─► velocity_smoother ─cmd_vel_smoothed─► collision_monitor
    ─/navigation/cmd_vel─► twist_mux (Prio 20) ─/cmd_vel─► diff_drive_node ─► ethercat_bus (Watchdog 0,3 s)
```

- **Nie rückwärts:** RPP `allow_reversing: false`, `velocity_smoother` `min_velocity` x = 0 (ein Rückwärtsbefehl wird 0),
  Verhaltensbäume ohne `BackUp`. Hinten gibt es keine Kamera und unter 0,15 m kein Lidar.
- **Kein Drehen als Recovery** (`Spin` aus den Bäumen entfernt): Das Heck schwenkt bis 0,43 m aus. Recovery = Costmaps
  leeren und 5 s warten.
- **Tempo:** höchstens 0,18 m/s (Ziel 0,15) und 0,6 rad/s; Beschleunigung 0,2 m/s².
- **Collision Monitor** (letzte Bremse, nur aktuelle Sensordaten, unabhängig von Karte und Costmaps; Zonen in RViz rot/gelb):
  - Stopp: Streifen 15 cm vor der Front (x 0,10–0,25 m, ±0,20 m)
  - Langsam auf 40 %: bis 60 cm vor der Front (x 0,10–0,60 m, ±0,25 m)
  - Vorausschau mit dem Footprint: bremst so, dass bis zur Berührung ≥ 1,5 s bleiben. **Unter Humble nur für
    Geradeausfahrt, nicht für Drehungen** (getestet). Gegen Anstoßen beim Drehen schützt der Controller: RPP prüft jeden
    Befehl, auch Drehen auf der Stelle, mit dem Footprint gegen die lokale Costmap (mit Gedächtnis).
- **Pose neu gesetzt** (`relocalize` im Betrieb): laufende Nav2-Ziele werden abgebrochen, ein neues Ziel ist nötig.
- Darunter wie immer: twist_mux-Sperren, Totmann der Tastatur, Fahrsperre ohne RustDesk, Watchdog im Busprozess.

### Getestet (2026-11-05, Domäne 77, ohne Antrieb)

- Alle Nav2-Knoten aktiv; globale und lokale Costmap, Footprint und Zonen werden veröffentlicht. Lokale Costmap: keine
  Hinderniszelle im eigenen Footprint.
- Planer: Wege zu Zielen 2–4 m entfernt in 2–18 ms.
- Sicherheitskette mit Befehlen: vorwärts 0,05–0,18 m/s geht durch, rückwärts wird 0, 0,30 m/s wird auf 0,18 begrenzt,
  0,6 rad/s geht durch.
- Collision Monitor mit künstlichen Hindernissen: 10 cm vor der Front → Stopp, 35 cm → 40 %, außerhalb → unverändert.
- Dabei gefunden und behoben: Die Fahne (bis 0,57 m hinter dem Lidar) lag im Footprint und ließ die Vorausschau immer
  wieder auf 0 springen → hinterer Ausschnitt-Quader 0,60 m tief, 0,45 m hoch (`slop_lio/config/crop.yaml`).
- **tf2-Deadlock (2026-11-05):** Mit ros-humble-tf2 0.25.23 blieb im `controller_server`/`planner_server` zufällig die
  TF stehen (Footprint mit immer gleichem Zeitstempel, „Transform data too old map → odom“, Ziele sofort „erreicht“
  oder „Resulting plan has 0 poses“, Hängen bei Ctrl-C/Reset). Ursache: Sperren-Kreuzung in tf2 zwischen TF-Empfang und
  `waitForTransform` der Costmap-Sensoren. **Behoben ab ros-humble-tf2 0.25.24** (`dpkg -l | grep ros-humble-tf2`).
- Approach-Polygon des Collision Monitors: unter Humble werden `points` ignoriert, es nimmt immer
  `local_costmap/published_footprint`.

### Offen

- **Karte neu aufnehmen:** `maps/office` wurde mit dem alten, zu kurzen Ausschnitt-Quader aufgenommen. Wo der Roboter
  länger stand (Startpunkt), stehen Teile seiner eigenen Rückseite als „Wand“ in der Karte (48 Zellen im Footprint am
  Startpunkt). Beim Speichern `--free 0.196` angeben (siehe oben), sonst gilt Unbekanntes als frei.
- Noch nicht gefahren: Ziele anfahren, Controller abstimmen (Tempo, Lookahead, Toleranzen).
