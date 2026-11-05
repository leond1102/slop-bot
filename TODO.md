# Slop-Bot – TODO

## EtherCAT-Zyklus als eigener Prozess – umgesetzt (2026-11-05), Abnahme am Roboter offen

**Stand:** Variante A ist gebaut: Prozess `ethercat_bus` ([bus_process.py](src/slop_base/slop_base/bus_process.py), nutzt
`Cl57eBus` unverändert, ohne rclpy) und `diff_drive_node` als Client über
[bus_link.py](src/slop_base/slop_base/bus_link.py) (Unix-Datagramm-Socket, JSON, Status 100 Hz, Sollwerte 50 Hz).
`diff_drive.launch.py` startet beide, der Startbefehl bleibt gleich. Getestet ohne Hardware: 8 Tests für bus_link und
ein Durchlauf echte Node ↔ nachgebauter Bus (Drehzahlen, Odometrie 50 Hz, Ende des Busprozesses). **Noch nicht mit dem
echten Treiber gelaufen.** Offen: Abnahme unten; Rechte nur noch für den Busprozess (heute bekommt der ganze Launch sie
über `with_netcaps`).

**Problem (vorher):** Der 2-ms-EtherCAT-Zyklus lief als Python-Thread (`_cyclic` in
[cl57e_bus.py](src/slop_base/slop_base/cl57e_bus.py)) im selben Prozess wie die ROS-Node `diff_drive_node`. Beide teilen sich
das Python-Lock (GIL). Blockiert der ROS-Teil, bleiben auch die EtherCAT-Frames aus, trotz SCHED_FIFO 80 auf Kern 7.

**Beobachtet:** Startet `lio.launch.py`, während der Bringup schon läuft, hängt im Antrieb ein `publish` 326–501 ms (DDS
verbindet die neuen Nodes). Der Treiber verliert die Verbindung, der Antrieb beendet sich:

```
ROS-Thread blockiert: update() 326 ms (publish 326 ms)
Bus-Watchdog: Sollwerte kamen nicht rechtzeitig, Räder gebremst
Beendet. … WKC-Fehler 134, max. Periode 326.0 ms
```

Gesehen am 2026-11-04 um 18:04 (501 ms, „EtherCAT-Kommunikation verloren“) und am 2026-11-05 um 10:21 (326 ms).
Logs: `~/.ros/log/python3_100422_1793845315426.log`.
Umgehung bis dahin: erst `lio.launch.py`, dann den Bringup starten.

**Ziel:** Der Zyklus läuft in einem eigenen Prozess (Python mit pysoem oder C++ mit SOEM) mit Echtzeitpriorität, ohne rclpy und
ohne GIL-Teilung mit ROS. `diff_drive_node` spricht nur noch über eine schnelle lokale Schnittstelle mit ihm
(z. B. Shared Memory oder Unix-Socket): Solldrehzahlen hin, Status, Positionen und Fehler zurück.

**Anforderungen:**
- Sicherheit bleibt im Zyklusprozess: Watchdog (kommen keine Sollwerte, bremsen), Abschalten beider Achsen bei Fehler an einer.
- Stirbt `diff_drive_node`, bremst der Zyklusprozess. Stirbt der Zyklusprozess, meldet der Treiber wie bisher einen Alarm.
- Rechte (CAP_NET_RAW, CAP_NET_ADMIN, CAP_SYS_NICE) braucht nur noch der Zyklusprozess (`with_netcaps`).
- Statistik wie bisher im Log (Zyklen, verspätet, Überläufe, WKC-Fehler, max. Periode).
- `tools/diff_motor.py` und die Tests in `slop_base` weiter nutzbar.

**Abnahme:** Bringup läuft, dann `lio.launch.py` (und RViz) starten: 0 WKC-Fehler, max. Periode unter 3 ms, kein
„Bus-Watchdog“.

## Weitere offene Punkte (Stand 2026-11-05)

- **Motoren ohne Haltemoment**, ein Motor wechselt beim Festhalten die Richtung. Software zeigt „Betrieb freigegeben“ (0x1637),
  kein Alarm. Prüfen: Motorstecker und Phasen (Kanal 3/4), Spulenwiderstand, Spannung am Treibereingang, Höchstspannung
  des CL57E-4A gegen 51 V Batterie, `diff_motor.py compare` (Soll: MaxCurrent 1300, OpenIMaxPer 100, IdleIPer 100).
- **Punkte hinter dem Crop-Quader:** 60–200 Punkte je Scan bei x −0,4 bis −0,75 m hinter dem Lidar erreichen FAST-LIO.
  In RViz prüfen, ob sie mitfahren (Mast/Fahne), dann `rear_box`-Tiefe in
  [crop.yaml](src/slop_lio/config/crop.yaml) auf ca. 0,8 m.
- **LIO unter Fahrt vermessen:** 1 m geradeaus, 90° Drehung, Rechteck zurück zum Start (Befehle in STARTBEFEHLE.md, Abschnitt 4).

## Niedrige Priorität

### Dual EKF: `map → odom` gefiltert statt sprunghaft

**Anlass:** In der LIO-Ansicht in RViz zittert der Roboter um einige Millimeter. `lio_bridge` setzt `map → odom` mit jeder
LIO-Pose (10 Hz) neu, und das Rauschen der LIO-Pose (im Stand Median 4,6 mm, Spitzen 12 mm je Scan) geht direkt in das TF.
Bisher nur ein Schönheitsfehler; angehen, wenn es z. B. mit Nav2 stört.

**Umsetzung (Standard von robot_localization, „dual EKF“):** zweite Instanz `ekf_node` mit `world_frame: map`. Eingänge:
LIO-Pose in `map` (`/lio/odom`, Pose x, y, Gier) und dieselben Geschwindigkeiten wie der odom-EKF (Räder, Gyro,
LIO-Geschwindigkeit). Diese Instanz sendet dann `map → odom`; in `lio_bridge` `publish_map_odom: false` setzen (nur eine
Quelle). Kein Zirkelbezug: Die LIO-Pose in `map` hängt nicht vom EKF ab (nur von der festen Startlage).

**Nachteile:** Korrekturen kommen verzögert an (bei einem echten LIO-Sprung stimmt `map` kurz nicht), RViz sieht ruhiger
aus als die Daten sind (für Messungen `/lio/odom` nehmen), eine falsche LIO-Pose wird nicht verworfen, nur langsamer
übernommen, mehr Konfiguration und etwas CPU (ein EKF braucht 6–10 % eines Kerns).

**Abnahme:** Roboter steht in RViz (Fixed Frame `map`) ruhig, Fahrtest gegen das Maßband nicht schlechter als vorher.
