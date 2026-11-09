# slop_navigation

Karte aufnehmen mit `slam_toolbox`, in der gespeicherten Karte lokalisieren mit `map_server` + AMCL + `relocalize`,
navigieren mit Nav2 (Costmaps vorerst nur aus dem Lidar, NavFn, Regulated Pure Pursuit, Collision Monitor, zwei
Sicherheitsstufen streng/locker).

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
sendet nur `slam_toolbox` (`lio_bridge`: `publish_map_odom: false`). Erste Karte war `maps/office` (21 × 24 m, ca. 65 m²
befahren; am 2026-11-06 gelöscht, Standard ist jetzt `office-3`).

## Karte erweitern (neue Bereiche dazu)

Statt neu zu kartieren, setzt slam_toolbox eine gespeicherte Karte fort (`mapping.launch.py extend:=office-3`, Details in
`slop_navigation/map_extend.py`). AMCL bleibt für den Alltag; erweitert wird bewusst per Knopf/Befehl.

1. Navigation/Lokalisierung beenden.
2. `mapping.launch.py extend:=office-3`, Roboter beim Start stillstehen lassen: `extend_start` bestimmt die Startpose am
   aktuellen Scan wie `relocalize` – Feinsuche um die letzte gespeicherte Pose (egal wie alt) bzw. um `x:= y:= yaw:=`,
   sonst in der ganzen Karte (feste Struktur als Referenz). Erst dann startet slam_toolbox mit `office-3.posegraph/.data`
   genau dort. Keine eindeutige Pose oder fehlender Posegraph → alles beendet sich mit Meldung.
   (Bis 2026-11-06 vormittags verlangte die Launch eine höchstens 10 min alte gespeicherte Pose – über die Webseite
   scheiterte das, weil man dort nicht „kurz lokalisieren“ kann.)
3. In den neuen Bereich fahren (mit Überlappung zum Bekannten, langsam), dann unter **neuem Namen** speichern
   (`map_saver_cli` + `serialize_map`). Die Webseite lehnt vorhandene Namen ab (map_saver überschreibt ohne Rückfrage).
4. `experience_transfer office-3.yaml office-4.yaml`: Erfahrung und letzte Pose auf die größere Karte umrechnen
   (Koordinaten bleiben gleich, nur Größe und Ursprung wachsen; Rundung höchstens eine halbe Zelle).
5. Mit `map:=…/office-4.yaml` navigieren. Die erweiterte Karte hat wieder einen Posegraph, lässt sich also selbst
   weiter erweitern.

Getestet (2026-11-06, Domäne 83, Aufnahme `~/nav_test`, ohne Antrieb): slam_toolbox lädt den echten `office-3`-Posegraph;
Start genau an der AMCL-Pose, 8 cm / 3° daneben und 25 cm / 8° daneben enden alle bei derselben Pose (−0,34 / −1,70 /
−138,0°, also ~7 cm neben AMCL – der bekannte Versatz Karte/AMCL). Speichern unter neuem Namen (`.yaml/.pgm/.posegraph/
.data`) und Übertragen funktionieren. Startsuche (`extend_start`, Domäne 85): alte, aber richtige gespeicherte Pose →
in 0,2 s bestätigt (97 %); falsche gespeicherte Pose (Roboter woanders) → 36 %, ganze Karte durchsucht, gefunden in 1,6 s
(98 % gegen 76 %); ohne /scan → nach 40 s Abbruch mit Meldung, slam_toolbox startet nicht. Über die Webseite: „läuft“
erst, wenn slam_toolbox /map sendet (11 s); Abbruch erscheint als Fehlermeldung mit Grund. 2026-11-06 am Roboter in neue
Bereiche gefahren und getestet.

## Lokalisieren in der gespeicherten Karte

```bash
ros2 run slop_base with_netcaps ros2 launch slop_bringup bringup.launch.py   # Antrieb, Modell, EKF
ros2 launch slop_navigation localization.launch.py                           # LIO + /scan + map_server + AMCL + RViz
ros2 run slop_teleop keyboard_teleop                                         # fahren
```

```
maps/office-3.yaml ─► map_server ─► /map ─┐
/scan ────────────────────────────────────┼─► AMCL ─► TF map → odom, /amcl_pose, /particle_cloud
TF odom → base_footprint (EKF) ───────────┘
```

- Standardkarte `~/slop-bot_ws/maps/office-3.yaml`, andere mit `map:=$HOME/slop-bot_ws/maps/<name>.yaml` (voller Pfad, `~`
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

- **Referenz ist nur feste Struktur** (seit 2026-11-06): Mit der Erfahrungskarte (`/experience/known`) zählen Wände,
  Schränke, Tische – nicht Stühle oder Mülleimer, auch wenn sie beim Kartieren dastanden. Regel wie Sicherheitsfeld und
  Export (`experience_core.structure`):
  - **fest** (Treffer): Erfahrung „fest“, oder Kartenwand, die weder beweglich noch verschwunden ist
  - **beweglich** (neutral): Erfahrung „vielleicht“. Ein Scanpunkt dort zählt weder als Treffer noch als Fehler.
  - Eine Kartenwand gilt erst als **verschwunden**, wenn die Erfahrung dort „frei“ sagt und im Umkreis von 20 cm nichts
    belegt war – Karte und Scans liegen am Slop-Bot oft 5–20 cm gegeneinander versetzt.
  - **Welche Pose gewinnt** (Suche, Eindeutigkeit), entscheiden nur Treffer auf fester Struktur: Ein Sofa kann einen
    sonst symmetrischen Raum nicht „eindeutig“ machen und die Suche nicht anziehen. **Ob die Pose gut genug ist**
    (80 % / 60 %), zählt ohne die neutralen Punkte (höchstens die Hälfte darf herausfallen): Stühle an ihren üblichen
    Plätzen lösen keine falsche Suche aus.
  - Ohne Erfahrung (oder solange sie zu wenig gesehen hat) wie bisher: alle Kartenwände. Im Log:
    „Referenz: … feste Zellen (davon … neu aus der Erfahrung); aus der Karte ausgenommen: … beweglich, … verschwunden“.
  - Aufnahme `~/nav_test` (Büro): Referenz 3042 Zellen (Karte 2737, davon 44 beweglich, 113 verschwunden, +462 neu);
    Güte an der AMCL-Pose 0,91 → 0,98; Suche findet die Pose (98 % gegen 76 %, vorher 92 % gegen 72 %).
- **Erkennen:** jede Sekunde die Güte – Anteil der Scanpunkte auf fester Struktur (±10 cm), bewegliche Stellen neutral –
  auf `/relocalize/score`. Richtige Pose: 90–99 %, versetzt: 15–50 %.
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
ros2 launch slop_navigation navigation.launch.py                             # Lokalisierung + Nav2 + RViz (streng)
ros2 launch slop_navigation navigation.launch.py safety:=locker              # oder: locker (siehe Sicherheitsstufen)
```

**Vorerst nur mit dem Lidar:** Die Kamera ist aus Costmaps, Collision Monitoren und Sensor-Wächter herausgenommen (die
Einträge stehen auskommentiert in `config/nav2.yaml` und `slop_safety/config/sensor_guard.yaml`). Nav2 sieht damit nichts
unter 0,15 m Höhe.

Ziel in RViz mit **Nav2 Goal** setzen (Pfeil an die Stelle und in Zielrichtung ziehen). Die Tastatur hat immer Vorrang
(twist_mux: Tastatur 50, Nav2 20), Hand-Stopp und RustDesk-Sperre (100) sperren auch Nav2.
Einstellungen: `config/nav2.yaml`, Verhaltensbäume: `behavior_trees/`, RViz: `slop_description/rviz/navigation.rviz`.

### Footprint

Kommt aus dem URDF (`slop_description`): `navigation.launch.py` rechnet beim Start den Umriss aller Teile von oben
(`slop_navigation/footprint.py`, Greifer offen, Schlitz zwischen den Backen gefüllt) und setzt damit `footprint` der
Costmaps, die `points` aller Zonen der Collision Monitore (Zuschläge in `config/zones.yaml`) und den Körper für
`unknown_obstacles`. Ändert sich der Roboter im URDF, passt sich Nav2 beim nächsten Start an; das Log zeigt
„Footprint aus dem URDF: …“, die erzeugte Datei liegt in `~/.ros/slop_navigation/footprint_<stufe>.yaml`.

Stand (T-Form): Körper 50 × 32 cm, beginnt 10 cm vor der Antriebsachse (= Drehpunkt = `base_footprint`), davor der
Greifer bis 31 cm vor der Achse, ±10 cm (Backen ±9 cm). Footprint-Zuschlag 2 cm. Innenkreis 0,14 m, die hinteren Ecken
liegen 0,43 m vom Drehpunkt: Beim Drehen auf der Stelle schwenkt das Heck weit aus. Die Werte in `nav2.yaml` sind nur der
Stand dazu (Test `test_footprint.py` prüft, dass sie zum URDF passen). Der Quader, der den Roboter aus Lidar-Wolke und
`/scan` schneidet, nimmt nur den Körper (`chassis`); der Greifer liegt unter dem Höhenband des Scans (ab 0,15 m).

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
- Sensoren: vorerst nur `/scan` (Lidar, markiert und räumt frei, `inf` = bis zur Reichweite frei). Mit Kamera kämen
  `/camera/obstacles` (markiert) und `/camera/floor` (räumt frei, Raytracing von `camera_link` aus) dazu. Ein Hindernis,
  das weggeht, verschwindet erst, wenn ein Sensor durch seine Zelle hindurchsieht. Niedrige Dinge sieht das Lidar erst ab
  ca. 2,6 m, unter 0,15 m Höhe gar nicht.
- Worauf es ankommt: Footprint richtig; der Roboter darf sich selbst nicht als Hindernis sehen (Ausschnitt-Quader);
  `inflation_radius` größer als der Außenkreis, sonst fährt der Planer zu dicht an Wände; Freiräumen muss möglich sein, sonst
  bleiben „Geister“ (z. B. vorbeigelaufene Menschen) liegen.

### Erfahrungskarte (`experience`)

Lernt bei jeder Fahrt, was in der Karte **fest** (Wand, Tisch), **vielleicht belegt** (Stuhl, Mülleimer) oder **frei**
ist, und gibt „vielleicht“ als weiche Kosten an den Planer. Startet mit `localization.launch.py` (also auch mit
`navigation.launch.py`); ohne: `experience:=false`.

- **Zählen:** je Zelle *gesehen* (Strahl ging hindurch oder traf) und *belegt* (Strahl traf). Beständigkeit
  p = belegt / gesehen. Ab 5 Beobachtungen: p ≥ 0,80 fest, 0,15–0,80 vielleicht, darunter frei.
- **Treffer-Schutz** (`hit_protect`, 2 Zellen = 10 cm): Zellen um einen Treffer räumt derselbe Scan nicht frei. Ohne ihn
  räumten flach streifende Strahlen und kleine Posenfehler die Wandzellen frei – Wände sahen aus wie Stühle (gemessen:
  nur 123 statt 305 Wandzellen „fest“; von 2737 Kartenwänden galten 1111 als „vielleicht“). Dateien, die vor
  2026-11-06 ohne Schutz gelernt wurden (Format 1), legt `experience` beim Start als `….v1.alt` beiseite und beginnt neu.
- **Vergessen:** Halbwertszeit 14 Tage (`half_life_days`; Baustelle eher 1–2). p bleibt dabei gleich, aber neue
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
- **Karte aktualisieren** (`experience_export`, schreibt neue Dateien, Original bleibt): dieselbe Regel wie die
  Relokalisierung – Kartenwände fallen nur weg, wenn sie beweglich oder verschwunden sind und nicht direkt an fester
  Struktur liegen; neue feste Struktur wird Wand, erkundete Fläche frei. Nur Zellen mit ≥ 10 Beobachtungen. Fiele mehr
  als ein Fünftel der Kartenwände weg, bricht der Export ab (`--trotzdem`). Erfahrung und letzte Pose werden mitkopiert.
- Dateien: `maps/<karte>.experience.npz` (jede Minute und beim Beenden). Passt das Raster nicht mehr (neue Karte unter
  altem Namen), wird die alte Datei als `.alt` beiseitegelegt und neu begonnen.
- Anzeige: RViz „Erfahrung (Beständigkeit)“ (`/experience/persistence`, Standard aus).
- Grenzen: nur das Höhenband des Lidars (0,15–1,8 m); Kamera noch nicht. Taugt erst nach einigen Fahrten.
- Getestet (2026-11-05, Domäne 78, Aufnahme `~/nav_test`, ohne Antrieb): lernt aus echten Scans, Maske kommt im
  Keepout-Filter des `planner_server` an; Maskenwert 40 auf freiem Gang → Costmap 40 (Kosten 102), nicht gesperrt.
  Stand 2026-11-06: 26 Unit-Tests (`test/test_experience.py`, `test/test_relocalize.py`, `test/test_safety.py`).

### Sicherheitsstufen (`safety:=streng` | `safety:=locker`)

| | **streng** (Standard) | **locker** |
|---|---|---|
| Wofür | Alltag, Menschen in der Nähe | enge Stellen, Rangieren, Roboter unter Aufsicht |
| Abstand | Wände eng (A: 2 cm Stopp, 10 cm langsam), Unbekanntes mit Abstand (B: 10 cm Stopp, 40 cm langsam) | alles gleich, bekannt oder nicht: 1 cm Stopp, keine Langsam-Zone |
| Tempo nahe an Hindernissen | gedrosselt (Langsam-Zonen, RPP nach Costmap-Kosten) | voll; frontal bremst nur die Vorausschau (ab ca. 11 cm, physikalisch nötig) |
| Rückwärts | nie | erlaubt: Liegt der Weg hinter dem Roboter, fährt er rückwärts |
| Drehen auf der Stelle | ja (RPP dreht zum Weg und am Ziel) | unterwegs nein (Bögen statt Wenden), **am Ziel ja: dreht in die Zielrichtung** |
| Verhaltensbaum | `navigate_to_pose.xml` | `navigate_to_pose_locker.xml`: erst hinfahren, dann drehen |
| Planer | bleibt eher in der Gangmitte (`cost_scaling_factor` 4) | nimmt den kurzen Weg dicht an der Wand (20) |
| Knoten | B `collision_monitor_dynamic` + `unknown_obstacles` + A | nur A |
| Wächter überwacht | `/scan`, `/safety/unknown_points`, `/odom` | `/scan`, `/odom` |

- `locker` lädt `config/nav2_locker.yaml` nach `nav2.yaml` (überschreibt nur, was dort steht), die Launch startet B und
  `unknown_obstacles` nicht und hängt A direkt an `cmd_vel_smoothed`. Gewechselt wird durch Neustart von
  `navigation.launch.py`.
- **Zwei Schritte in `locker`:** RPP (Humble) kann nicht zugleich rückwärts fahren und sich auf der Stelle drehen – mit
  `use_rotate_to_heading: true` schaltet es `allow_reversing` still ab. Deshalb zwei RPP-Controller im
  `controller_server` und eigene Verhaltensbäume (`behavior_trees/*_locker.xml`):
  1. `FollowPath` (rückwärts erlaubt, kein Drehen auf der Stelle) fährt hin, bis die Position stimmt
     (`position_goal_checker`, 0,15 m, Richtung egal).
  2. Danach, außerhalb der Pipeline mit Neuplanung, dreht `RotateToGoal` (wie `FollowPath` in streng) auf der Stelle in
     die Zielrichtung (`general_goal_checker`, ±0,25 rad). Kommt währenddessen ein neues Ziel, bricht der Schritt ab und
     es geht mit dem neuen Ziel von vorn los. Beim Drehen schwenkt das Heck wie in streng bis 0,43 m aus.
- **Grenze von `locker`:** Von 0,18 m/s braucht der Roboter ca. 10 cm bis zum Stand. Seitlich darf er mit vollem Tempo
  1 cm an etwas vorbei; frontal (vorwärts wie rückwärts) bremst die Vorausschau (0,6 s bis zur Berührung) ab ca. 11 cm.
  Hinten wie vorn sieht er nichts unter 0,15 m Höhe.
- Darunter gelten in beiden Stufen dieselben Sperren: twist_mux, Totmann, Fahrsperre ohne RustDesk, Watchdog im Busprozess.

### Sicherheit (Stufe streng, wenn nicht anders gesagt)

```
streng: controller_server / behavior_server ─cmd_vel_nav─► velocity_smoother ─cmd_vel_smoothed─►
        collision_monitor_dynamic (B) ─cmd_vel_dynamic─► collision_monitor (A) ─/navigation/cmd_vel─►
        twist_mux (Prio 20) ─/cmd_vel─► diff_drive_node ─► ethercat_bus (Watchdog 0,3 s)
locker: … velocity_smoother ─cmd_vel_smoothed─► collision_monitor (A, 1-cm-Stopp + Vorausschau) ─/navigation/cmd_vel─► …
```

- **Nie rückwärts:** RPP `allow_reversing: false`, `velocity_smoother` `min_velocity` x = 0 (ein Rückwärtsbefehl wird 0),
  Verhaltensbäume ohne `BackUp` (in beiden Stufen). Hinten gibt es keine Kamera und unter 0,15 m kein Lidar.
- **Kein Drehen als Recovery** (`Spin` aus den Bäumen entfernt): Das Heck schwenkt bis 0,43 m aus. Recovery = Costmaps
  leeren und 5 s warten.
- **Tempo:** höchstens 0,18 m/s (Ziel 0,15) und 0,6 rad/s; Beschleunigung 0,2 m/s².
- **Dynamisches Sicherheitsfeld: zwei Collision Monitore hintereinander** (jeder kann nur bremsen). Bekannte Struktur
  (Wände) passiert der Roboter eng, Unbekanntes (Mensch, Stuhl, Kiste) mit Abstand:

  | | sieht | Stopp | Langsam (40 %) | Vorausschau |
  |---|---|---|---|---|
  | **B** `collision_monitor_dynamic` | nur unbekannte Lidar-Punkte (`/safety/unknown_points`) | 10 cm vor der Front, ±0,22 m | 40 cm vor, 15 cm hinter und neben dem Körper | ≥ 1 s |
  | **A** `collision_monitor` | alle Lidar-Punkte, unabhängig von Karte und Lokalisierung | 2 cm um den Körper | 10 cm um den Körper | ≥ 0,6 s |
  | **A** in `locker` | alle Lidar-Punkte | 1 cm um den Körper | – | ≥ 0,6 s |

  - `unknown_obstacles` trennt die Scanpunkte: bekannt = in der Erfahrungskarte fest, oder in der Karte belegt und die
    Erfahrung widerspricht nicht; ± 10 cm Toleranz. **Im Zweifel unbekannt:** ohne Karte, ohne TF, bei unsicherer
    Lokalisierung (relocalize < 60 %, AMCL > 0,3 m) oder außerhalb der Karte gehen alle Punkte an B – dann gelten die
    größeren Zonen auch für Wände. Ein Mensch, der direkt an einer bekannten Wand steht, gilt als Wand: dafür bleibt A.
  - Zonen in RViz: A rot/gelb, B dunkelrot/orange, unbekannte Punkte magenta.
  - Bremsweg bei 0,18 m/s ≈ 10 cm, bei 40 % ≈ 2,4 cm: Die kleinen Zonen von A sind frontal nur mit der Vorausschau
    sicher (bei 2 cm Abstand höchstens 0,03 m/s). B: Langsam-Zone 40 cm bremst von 0,18 auf 0,07 m/s (≈ 10 cm),
    danach reichen 2,4 cm Bremsweg für die 10-cm-Stoppzone. Flache Dinge (< 0,15 m) sähe nur die Kamera (vorerst aus).
  - Vorausschau: **unter Humble nur für Geradeausfahrt, nicht für Drehungen** (getestet), und sie nimmt immer den
    Footprint der lokalen Costmap (`points` werden ignoriert). Gegen Anstoßen beim Drehen schützt der Controller: RPP
    prüft jeden Befehl, auch Drehen auf der Stelle, mit dem Footprint gegen die lokale Costmap.
  - **Humble ignoriert eine Quelle, die länger als 1 s nichts liefert oder für die das TF fehlt (kein Stopp!).** Fehlt
    `/odom` (EKF), trifft das alle Quellen. Dagegen hilft der Sensor-Wächter (nächster Punkt).
- **Sensor-Wächter** (`sensor_guard` aus `slop_safety`, startet mit `navigation.launch.py`, beide Stufen): sperrt Nav2
  über twist_mux (`safety/sensor_lock`, Priorität 30), sobald eine Quelle der Collision Monitore fehlt oder veraltet ist.
  Tastatur und Web fahren weiter.

  | Quelle | Grenze | gemessen (2026-11-06, Stand) | Stufe |
  |---|---|---|---|
  | `/scan` | 0,4 s | höchstens 0,15 s | beide |
  | `/safety/unknown_points` | 0,4 s | höchstens 0,15 s | streng |
  | `/odom` | 0,3 s | – (50 Hz) | beide |
  | `/camera/obstacles` | 0,4 s | höchstens 0,24 s | vorerst aus |

  - Geprüft wird, ob Nachrichten ankommen und wie alt ihr Zeitstempel ist (so rechnet auch der Collision Monitor), auch
    Zeitstempel mehr als 0,2 s in der Zukunft. Freigabe erst nach 1 s ohne Problem.
  - Reaktion: Quelle 0,4 s still → Sperre → `diff_drive_node` bremst nach 0,5 s ohne `/cmd_vel`. Gesamt ~1 s nach den
    letzten Daten, genau dann ignorierte der Collision Monitor die Quelle. Bis dahin rechnet er die letzten Daten auf
    die aktuelle Pose um.
  - Fail-Safe: Startet gesperrt. Läuft der Wächter nicht oder hängt er, sperrt twist_mux nach 0,5 s von selbst.
  - Nav2 behält sein Ziel. Nach der Freigabe fährt es weiter. Dauert die Sperre länger, meldet der Fortschrittsprüfer
    nach 15 s einen Fehler. Der Verhaltensbaum versucht es dann bis zu 6 Mal neu und bricht das Ziel danach ab.
  - Stand ansehen: `ros2 topic echo /safety/sensor_status` („frei“ oder „gesperrt: /scan seit 0.5 s still“). Im Log alle
    60 s das höchste Alter je Quelle. Einstellungen: `slop_safety/config/sensor_guard.yaml`.
- **Pose neu gesetzt** (`relocalize` im Betrieb): laufende Nav2-Ziele werden abgebrochen, ein neues Ziel ist nötig.
- Darunter wie immer: twist_mux-Sperren, Totmann der Tastatur, Fahrsperre ohne RustDesk, Watchdog im Busprozess.

### Getestet (2026-11-05, Domäne 77, ohne Antrieb)

- Alle Nav2-Knoten aktiv; globale und lokale Costmap, Footprint und Zonen werden veröffentlicht. Lokale Costmap: keine
  Hinderniszelle im eigenen Footprint.
- Planer: Wege zu Zielen 2–4 m entfernt in 2–18 ms.
- Sicherheitskette mit Befehlen: vorwärts 0,05–0,18 m/s geht durch, rückwärts wird 0, 0,30 m/s wird auf 0,18 begrenzt,
  0,6 rad/s geht durch.
- Collision Monitor mit künstlichen Hindernissen: 10 cm vor der Front → Stopp, 35 cm → 40 %, außerhalb → unverändert.
- Dynamisches Sicherheitsfeld (Domäne 78/79, ohne Antrieb): mit `~/nav_test` gelten 95 % der Lidar-Punkte im Umkreis
  von 3 m als bekannte Struktur, keine Zone greift. Künstliche Hindernisse bei 0,15 m/s: Wand 8 cm vorn → A 40 %, 1 cm →
  A Stopp, 5 cm seitlich → A 40 %, B unbeteiligt; Mensch 70 cm vorn → B 40 %, 20 cm → B Stopp, 25 cm seitlich → B 40 %
  (mit den damaligen größeren B-Zonen 25 cm / 1 m; danach auf 10 cm / 40 cm verkleinert, siehe unten).
  Gefunden und behoben: `unknown_obstacles` stürzte beim ersten Wechsel „alles unbekannt“ → „Karte genutzt“ ab
  (rclpy: info und warn an derselben Stelle); B ignorierte die Quelle danach still.
- Dabei gefunden und behoben: Die Fahne (bis 0,57 m hinter dem Lidar) lag im Footprint und ließ die Vorausschau immer
  wieder auf 0 springen → hinterer Ausschnitt-Quader 0,60 m tief, 0,45 m hoch (`slop_lio/config/crop.yaml`).
- **tf2-Deadlock (2026-11-05):** Mit ros-humble-tf2 0.25.23 blieb im `controller_server`/`planner_server` zufällig die
  TF stehen (Footprint mit immer gleichem Zeitstempel, „Transform data too old map → odom“, Ziele sofort „erreicht“
  oder „Resulting plan has 0 poses“, Hängen bei Ctrl-C/Reset). Ursache: Sperren-Kreuzung in tf2 zwischen TF-Empfang und
  `waitForTransform` der Costmap-Sensoren. **Behoben ab ros-humble-tf2 0.25.24** (`dpkg -l | grep ros-humble-tf2`).
- Approach-Polygon des Collision Monitors: unter Humble werden `points` ignoriert, es nimmt immer
  `local_costmap/published_footprint`.
- 2026-11-06 am Roboter gefahren: Ziele anfahren, Erfahrungskarte im Betrieb.
- **Sicherheitsstufen (2026-11-06, Domäne 84, ohne Antrieb):** 13 Tests (`test/test_safety_modes.py`: nur Lidar, Kette
  lückenlos, streng nie rückwärts, locker rückwärts ohne Drehen unterwegs, Stoppzone 1 cm, Bäume nennen nur vorhandene
  Controller/Goal-Checker, in locker Drehen nach der Pipeline). Launch für beide Stufen
  aufgelöst (locker ohne B und `unknown_obstacles`, Wächter mit 2 Quellen; falscher Wert → Abbruch mit Meldung). Mit
  echten Knoten (`controller_server`, `planner_server`, `velocity_smoother`, `collision_monitor`, nur konfiguriert)
  kommen alle Werte an, auch in den Costmaps; RPP meldet in `locker` kein „Disabling reversing“.
  Simulation (leere Karte, Attrappen-Roboter integriert `/navigation/cmd_vel`, ganzer Nav2-Stapel): locker, Ziel 1 m hinter
  dem Roboter mit 90° → 0,86 m rückwärts, dann auf der Stelle auf 79° gedreht, erreicht (11,6 s); Ziel schräg vorn mit
  180° → Bogen vorwärts, dann auf 172° gedreht, erreicht. streng, gleiches Ziel → erst gedreht, vorwärts hin, am Ziel
  gedreht, nie rückwärts (18,1 s). **Am Roboter nicht gefahren.**
- **Sensor-Wächter (2026-11-06, Domäne 84, ohne Antrieb):** 10 Unit-Tests (`slop_safety/test/test_sensor_guard.py`).
  Mit echtem twist_mux und Attrappen-Quellen: Nav2 kommt 1,1 s nach dem Start durch. `/scan` aus → Nav2 nach 0,4 s
  gesperrt, Tastatur fährt weiter, Freigabe 1,2 s nach Rückkehr. Kamera-Zeitstempel 0,5 s in der Zukunft oder `/odom`
  0,5 s alt → sofort gesperrt. Wächter mit `kill -9` beendet → twist_mux sperrt nach 0,4 s. Strg+C sperrt sofort.
  Mit echtem Lidar und echter Kamera (40 s, Stand): keine Fehlsperre, Alter siehe Tabelle oben.

### Offen

Siehe [TODO.md](../../TODO.md).
