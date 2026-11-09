# slop_web – Weboberfläche

Programme starten und stoppen, fahren, Notaus – im Browser auf dem Jetson, am Laptop und am Handy (über Tailscale).

```bash
ros2 run slop_web web
```

| Adresse | Ansicht |
|---|---|
| `http://100.99.71.126:8080` | alles: Starten, Einzeln, Aktionen, Fahren (Tastatur/Maus), Logs |
| `http://100.99.71.126:8080/handy` | nur Fahrtasten, Tempo und Notaus (schmale Bildschirme bekommen sie automatisch; `/?desktop` erzwingt die volle Seite) |
| `http://127.0.0.1:8080` | auf dem Jetson selbst |

`100.99.71.126` ist die Tailscale-Adresse des Jetson (`tailscale ip -4`). Der Server lauscht **nur** auf localhost und
dieser Adresse, nicht im übrigen Netz. Jedes Gerät in deinem Tailnet kann die Seite öffnen und damit fahren.

## Einrichten (einmalig)

**1. Antrieb ohne sudo-Passwort** (sonst kann der Knopf „Antrieb“ nicht starten, alles andere geht trotzdem):

```bash
sudo install -o root -g root -m 0755 ~/slop-bot_ws/src/slop_base/scripts/slop-netcaps /usr/local/sbin/slop-netcaps
echo 'jetson ALL=(root) NOPASSWD:SETENV: /usr/local/sbin/slop-netcaps' | sudo tee /etc/sudoers.d/slop-netcaps
sudo chmod 0440 /etc/sudoers.d/slop-netcaps && sudo visudo -c
```

`slop-netcaps` macht als root nur zwei Dinge: Interrupts auf Kerne legen (nur Zahlenpaare) und den Befehl sofort wieder
als `jetson` mit CAP_NET_RAW, CAP_NET_ADMIN und CAP_SYS_NICE starten. Die Regel erlaubt also kein beliebiges root.
`with_netcaps` nutzt den Helfer automatisch, wenn er eingerichtet ist; im Terminal funktioniert danach alles wie bisher,
nur ohne Passwortfrage. Nach Änderungen an `slop-netcaps` den `install`-Befehl wiederholen.

**2. Autostart** (optional, sonst `ros2 run slop_web web` in einem Terminal):

```bash
mkdir -p ~/.config/systemd/user
cp ~/slop-bot_ws/install/slop_web/share/slop_web/systemd/slop-web.service ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now slop-web
sudo loginctl enable-linger jetson     # optional: auch ohne Anmeldung nach dem Booten
journalctl --user -u slop-web -f       # Ausgabe ansehen
```

## Bedienung

- **Abfolgen** („Navigation komplett“ usw.) starten die Schritte nacheinander und warten jeweils, bis Daten kommen
  (Antrieb: `/wheel/odom`, Kamera: `/camera/obstacles`, Lidar: `/scan`). Was schon läuft, wird übersprungen.
  „Navigation komplett“ startet vorerst ohne Kamera (Nav2 nutzt nur das Lidar).
- **Sicherheitsstufe** (Auswahl „Sicherheit“ neben der Karte): `streng` (Standard nach jedem Laden der Seite) oder
  `locker` (extrem nah an alles, volles Tempo, rückwärts erlaubt, dreht sich erst am Ziel in die Zielrichtung; Details in
  `slop_navigation/README.md`). Gilt beim Start der Navigation, mit `locker` fragt die Seite vorher nach. Wechseln =
  Navigation stoppen und neu starten. Die laufende Stufe steht oben („Nav2 locker“, gelb) und neben der Navigation unter
  „Einzeln“ – auch, wenn die Navigation im Terminal gestartet wurde (aus `safety:=` in dessen Befehlszeile).
- **Dockingstation** („Andocken komplett“ startet Antrieb, Kamera, Navigation und den Dock-Knoten; einzeln unter „Einzeln“):
  Unter „Aktionen“ **Dock speichern** (Roboter vorher 0,6–1,5 m gerade vor den Marker stellen), **Andocken** (mit
  Rückfrage, der Roboter fährt selbstständig) und **Andocken abbrechen**. Oben steht „Dock: <Stand>“ (bereit, anfahren,
  feinfahren, fertig, fehler …), das Ergebnis kommt zusätzlich als Meldung. Details: `slop_dock/README.md`.
- **Greifer** (Karte „Greifer“, die `gripper_node` startet mit dem Antrieb): **Homen** (nur nach dem Einschalten des
  Treibers), **Greifen** mit Mitte Y und Breite in mm, **Verschieben** auf eine neue Mitte Y, **Öffnen**, **Stopp**.
  Y in Roboterkoordinaten (0 = Mitte, + = links). Fahrten mit 4 mm/s dauern bis 45 s, das Ergebnis kommt als Meldung;
  oben steht „Greifer: <Stand>“. Ohne laufende `gripper_node` sind die Knöpfe gesperrt.
- **Kamera-Debug (Dockingstation):** Karte „Kamera-Debug“ → „Debug-Bild anzeigen“: Kamerabild mit Overlay (erkannte Marker
  mit ID, verworfene Kandidaten, Pose, Hinweis auf ein falsches Wörterbuch). Läuft nur, solange es offen ist; Dock-Knoten und
  Kamera müssen laufen (die Karte sagt es, wenn nicht). Der Server liefert es als Videostrom unter `/api/dock/stream`.
- **Sensor-Wächter:** Oben „Sensor-Wächter ✓“ oder rot „Nav2 gesperrt: <Quelle> …“ (auch bei Navigation aus dem Terminal).
  Gesperrt = Nav2 steht, Fahren von der Seite geht weiter.
- Aktionen mit Dienstaufruf (Dock, Relokalisierung, Erfahrung speichern …) zeigen die Antwort des Dienstes; meldet er
  `success=False`, steht ein ✗ mit dem Grund da.
- **Karte erweitern:** Karte wählen → „Karte erweitern komplett“ (Antrieb + slam_toolbox; die Startpose wird beim
  Start am Scan gesucht, Roboter dabei stillstehen lassen; vorher Navigation stoppen) → neue Bereiche abfahren → Name eingeben → „Erweiterte Karte
  speichern“ (speichert unter dem neuen Namen, überträgt Erfahrung und letzte Pose). Vorhandene Kartennamen werden
  abgelehnt, das Original bleibt immer unverändert.
- **Einzeln:** Start/Stopp je Programm, „Log“ zeigt die Ausgabe (wie im Terminal). Stopp wirkt wie Strg+C, nach 15 s
  SIGTERM, nach 25 s SIGKILL. Lokalisierung, Navigation und Kartieren schließen sich gegenseitig aus (je ein Lidar-Treiber).
- Beendet sich ein Programm von selbst (nicht über „Stopp“), zeigt die Seite eine Fehlermeldung mit der wichtigsten
  Logzeile (z. B. „Karte erweitern: nach 40 s kein /scan – läuft der Lidar?“).
- **Im Terminal gestartet** erkennt die Seite („im Terminal“) und fasst es nicht an – dort mit Strg+C beenden.
- RViz öffnet sich auf dem Bildschirm des Jetson (RustDesk), wie beim Start im Terminal.
- **Fahren:** Taste halten = fahren, loslassen = Stopp. Desktop: `W`/`↑` vor, `S`/`↓` zurück, `A`/`←` `D`/`→` drehen,
  `Q`/`E` Kurve, `+`/`−` Tempo, **Leertaste = Notaus**. Handy: Tasten gedrückt halten.
- Logs: `~/.ros/log/slop_web/<programm>-<zeit>.log`, Aktionen in `aktionen.log`.
- Der Webserver kann neu gestartet werden, ohne dass etwas stoppt: Gestartete Programme laufen weiter und werden beim
  nächsten Start wieder übernommen (`~/.ros/slop_web/state.json`).

## Sicherheit

- **Totmann:** Der Browser meldet alle 0,1 s „Taste gehalten“. Bleibt das 0,3 s aus (losgelassen, Funkloch, Handy
  gesperrt, Tab gewechselt, Verbindung weg), kommen 0,3 s Nullbefehle, danach nichts mehr – twist_mux läuft ab, Nav2
  darf wieder. Gesendet wird auf `teleop/cmd_vel` (Priorität 50, wie `keyboard_teleop`).
- **Notaus** setzt die Hand-Stopp-Sperre von twist_mux (`/safety/stop` = true, Priorität 100), wiederholt sie alle 0,5 s
  und bricht laufende Nav2-Ziele ab, damit der Roboter nach dem Aufheben nicht einfach weiterfährt. Aufheben geht auf
  der Seite oder im Terminal (`/safety/stop` false). **Software-Stopp über das Netz, kein Not-Halt nach Norm.**
- **RustDesk-Fahrsperre bleibt:** Ohne RustDesk-Sitzung fährt der Roboter auch von der Seite aus nicht (Anzeige
  „Fahrsperre: keine RustDesk-Sitzung“).
- Der Browser kann nur auslösen, was in `config/web.yaml` steht. Karten müssen in `maps/` existieren, Kartennamen nur
  aus Buchstaben, Ziffern, `_` und `-`, die Sicherheitsstufe nur `streng` oder `locker`. Websockets von fremden Seiten werden abgewiesen (Origin-Prüfung).

## DDS-Einstellungen

Webserver und gestartete Programme müssen dieselben DDS-Einstellungen haben, sonst sehen sie sich nicht (dann zeigt die
Seite „Antrieb aus“, obwohl er läuft, und Fahren geht nicht). Deshalb legt `config/web.yaml` (`dds:`) sie fest – wie in
`~/.bashrc`: `ROS_LOCALHOST_ONLY=1`, keine Profildatei. Das gilt unabhängig davon, aus welchem Terminal der Webserver
gestartet wird; Abweichungen des Terminals stehen beim Start als Warnung im Log („DDS-Einstellung aus web.yaml statt aus
dem Terminal …“). Die wirksamen Einstellungen zeigt die Seite unten im Feld „Fahren“.
Läuft ein Programm länger als 20 s, ohne dass seine Daten bei der Seite ankommen, steht „läuft · keine Daten“ daneben.

## Anpassen

`config/web.yaml`: Port, Adressen, Tempo, was startbar ist (Befehle wie in STARTBEFEHLE.md als Liste), Abfolgen,
Aktionen. Danach neu bauen und den Webserver neu starten.

## Getestet (2026-11-05, ohne Antrieb, Domäne 80)

- 15 Unit-Tests (`test/test_web.py`): Totmann, Eingabeprüfung (Pfad-Tricks, Shell-Zeichen), Starten/Stoppen wie Strg+C
  mit Kindprozess, Eskalation bei SIGINT-ignorierendem Prozess, Übernahme nach Neustart, fremde Prozesse unberührt.
- Ende-zu-Ende mit Testkonfiguration (22 Prüfungen): Fahren 0,09 m/s bei 50 %, nach dem Loslassen 6 Nullbefehle,
  danach still; Notaus kommt nach 11 ms auf `/safety/stop` an, auch während eine Abfolge wartet; bei Notaus kein Fahren;
  Konflikte, ungültige Karte/Name abgelehnt; fremde Origin → 403.
- Darstellung Desktop und Handy (Chromium, headless).
- 2026-11-05 erster echter Start: Antrieb lief (Root-Helfer ok, EtherCAT ohne Aussetzer), aber die Seite sah ihn nicht –
  der Webserver war aus einem Terminal mit alten UDP-Testeinstellungen gestartet worden. Seitdem legt `web.yaml` die
  DDS-Einstellungen fest; nachgestellt (Webserver mit den alten Einstellungen, Antrieb-Attrappe): Seite sieht den
  Antrieb, ein Knoten aus einem normalen Terminal empfängt die Fahrbefehle.
- 2026-11-06: Fahren am Roboter über die Seite getestet.
- 2026-11-06 Dock und Sensor-Wächter (Testserver Port 8092, Domäne 84): Dock-Knoten über die Seite gestartet, `robot.dock`
  kommt an; „Dock speichern“ ohne Marker → ✗ „Marker ID 1 gerade nicht zu sehen“, „Andocken“ ohne Dock-Datei → ✗ mit Grund,
  „Abbrechen“ ✓, Stopp. Die Seite selbst nicht im Browser angesehen (nur `gjs`-Syntaxprüfung).
- 2026-11-06 Sicherheitsstufe (Testserver Port 8091, Domäne 84, Attrappe statt Nav2): Start mit `locker` → läuft,
  Anzeige „locker“; ungültige Stufe abgelehnt; Abfolge ohne Angabe → `streng`; im Terminal gestartete Navigation mit
  `safety:=locker` erkannt. JavaScript per `gjs` auf Syntax geprüft, die Seite selbst nicht im Browser angesehen.
