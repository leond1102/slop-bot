# slop_dock – Dockingstation

ArUco-Marker (ID 1, Wörterbuch ARUCO_ORIGINAL, schwarzes Quadrat 10 cm) mit der RealSense erkennen, in der Karte vermerken und
den Roboter mittig davor anhalten lassen. Der Marker hängt an der Wand auf Kamerahöhe (ca. 15 cm), der Roboter fährt
**vorwärts** hin (die Kamera sitzt vorn).

## Einrichten

1. Marker ausdrucken: `python3 tools/make_aruco_marker.py` → `marker_ARUCO_ORIGINAL_id1.png` (15 × 15 cm mit Rand). Auf **100 %**
   drucken, matt, das schwarze Quadrat nachmessen (10 cm; sonst `marker_size` in `config/dock.yaml` anpassen). Flach und
   senkrecht auf Kamerahöhe aufhängen, mittig vor der Stelle, wo der Roboter stehen soll.
2. Starten (Bringup läuft schon): `ros2 launch slop_sensors camera.launch.py` (Farbbild), `ros2 launch slop_navigation
   navigation.launch.py` (Lokalisierung und Nav2), `ros2 launch slop_dock dock.launch.py [map:=…]`.
   Nach dem Bauen den Antrieb neu starten, damit twist_mux den neuen Eingang `dock/cmd_vel` kennt.
3. **Dock vermerken:** Roboter von Hand (Tastatur) 0,6–1,5 m gerade vor den Marker stellen, Lokalisierung muss passen:
   `ros2 service call /dock/save std_srvs/srv/Trigger`. Der Knoten rechnet die Marker-Pose über die TF-Kette in die Karte
   um und schreibt `maps/<karte>.dock.yaml` (x, y, Richtung der Markernormalen). Kontrolle: `ros2 topic echo /dock/status`,
   das Bild mit eingezeichnetem Marker: `ros2 run rqt_image_view rqt_image_view /dock/image`.
4. **Andocken:** `ros2 service call /dock/start std_srvs/srv/Trigger`, Abbrechen mit `/dock/cancel` (oder Notaus).

## Debug-Ansicht: „der Knoten erkennt keinen Marker“

Das Kamerabild mit Overlay ansehen: auf der **Weboberfläche** unter „Kamera-Debug (Dockingstation)“ → „Debug-Bild
anzeigen“, oder `ros2 run rqt_image_view rqt_image_view /dock/image`. Dock-Knoten und Kamera müssen laufen. Zu sehen:

- **grüner Rahmen** = gesuchter Marker (mit ID, roter Punkt = Ecke 0), **orange** = anderer Marker (falsche ID?),
  **grau dünn** = Kandidaten, die quadratisch aussahen, aber keinen gültigen Code ergaben (zu klein, unscharf, Reflexe);
- Text oben: Ziel-ID und Wörterbuch, Bildformat, welche IDs im Bild sind, bei Erfolg Abstand, Querversatz, Winkel;
- **gelber Hinweis**, wenn nichts gefunden wird: Der Knoten probiert dann alle gängigen Wörterbücher durch
  („Kein Treffer im Wörterbuch ARUCO_ORIGINAL, aber in: 4X4_50 ID 3 → dictionary in config/dock.yaml anpassen“). Ist ein Marker
  zu sehen, aber nicht ID 1, steht das ebenfalls da. Außerdem: Pose verworfen (hängt nicht senkrecht), kein TF.

Das Farbbild wird nur empfangen, solange jemand zuschaut, ein Dienst (`/dock/save`, `/dock/start`) kurz prüft oder die
Suche/das Feinfahren läuft. **Im Leerlauf kostet der Knoten nur etwa 5 % eines Kerns.** (Vorher empfing er dauernd alle
Farbbilder: ca. 50 % eines Kerns, das belastete LIO und Nav2.) Dauerbetrieb nur zum Testen: Parameter `idle_detection: true`.

## Ablauf

```
/dock/start ─► anfahren   Nav2 fährt zu einem Punkt 1,1 m vor dem Marker, Blick zur Wand (entfällt, wenn der Marker schon
                          im Bild ist)
             ─► suche     bis 4 s auf den Marker warten
             ─► feinfahren  Kamera: auf die Markernormale ausrichten, mit höchstens 0,05 m/s zum Haltepunkt
             ─► pruefen   stehen bleiben, 1 s Messung mitteln
             ─► fertig | fehler (mit Grund)
```

- **Haltepunkt:** Marker 0,48 m von der Antriebsachse (`stop_x`) = 40 cm von der Kamera. Körperfront 38 cm vor der Wand.
- **Lockere Grenzen für „fertig“:** quer ±3 cm, Winkel ±6°, längs ±3 cm (`tol_*` in `config/dock.yaml`). Messung und
  Regelung streuen je 1–2 cm.
- **Regelung** (`dock_core.Servo`): zielt auf einen Punkt der Markernormalen, geneigt mit dem Querversatz (bis 40°), dreht
  auf der Stelle, wenn sie mehr als 40° falsch steht. Die Peilung zum Marker bleibt innerhalb ±14°, damit er nicht aus dem
  Bild läuft (die Kamera sieht bei 640×480 nur ±27°).
- **Marker senkrecht:** Ein Quadrat hat aus einem Bild zwei mögliche Lagen. Gewählt wird die mit kleinerem Bildfehler und
  waagrechter Normale; Marker, die nicht senkrecht hängen (`max_tilt`), werden verworfen. OpenCVs schnelles Verfahren
  (IPPE) liefert bei einem Marker genau auf Kamerahöhe `NaN` oder falsche Werte; dann rechnet ein iteratives Verfahren.

## Sicherheit

- Das Feinfahren geht über `dock/cmd_vel` → twist_mux (Priorität 40: über Nav2, unter der Tastatur) und **umgeht die
  Collision Monitore**. Deshalb: höchstens 0,05 m/s, Abbruch bei Marker verloren (1 s), bei einem Hindernis direkt vor
  der Front im Lidar (Box 10–20 cm vor der Front, ±19 cm; ohne `/scan` kein Fahren), nach 90 s, bei Notaus.
  Notaus und RustDesk-Sperre (100) sperren den Eingang wie alles andere. Die Sperre des Sensor-Wächters (30) trifft ihn nicht.
- Bei jedem Fehler stoppt der Knoten sofort und sendet Nullbefehle; Nav2-Ziele werden abgebrochen.
- Rückwärts fährt der Knoten nie. Ist er am Haltepunkt zu ungenau, meldet er einen Fehler; neu starten fährt wieder zum
  Anfahrpunkt.
- Kontakte oder ein Ladeanschluss sind nicht berücksichtigt: `stop_x` und die Lidar-Box (`guard_x_max`) anpassen, sobald
  die Mechanik feststeht.

## Topics und Dienste

| | |
|---|---|
| `/dock/save`, `/dock/start`, `/dock/cancel` | Trigger-Dienste |
| `dock/status` | Text zum Stand (transient local) |
| `dock/marker` | erkannter Marker in `base_footprint` (Pose zeigt aus der Wand heraus) |
| `dock/image` | Debug-Bild mit Overlay (nur, wenn jemand abonniert; schaltet das Farbbild ein) |
| `dock/cmd_vel` | Feinfahren → twist_mux |

## Getestet (2026-11-06, Domäne 84, ohne Antrieb, ohne echte Kamera)

- 47 Unit-Tests (`test/test_dock.py`): Erkennung an gerenderten Markerbildern (Abstand 0,45–1,1 m, bis 40° schräg, auf
  Kamerahöhe, mit Bildrauschen, andere ID/Wörterbuch/leeres Bild), Geometrie, Regelung im Kreis (25 Startlagen aus der
  Nav2-Toleranz ±15 cm / ±15°, Marker bleibt im Bild), Dock-Datei.
- Mit Attrappen-Roboter, der die Kamerabilder aus seiner Pose rendert, echtem Nav2 und dem echten Dock-Knoten: Dock
  speichern (Marker 3,002 / 0,000, Wahrheit 3,0 / 0,0, Normale 178° statt 180°); Andocken aus 4 m Entfernung, schräg:
  Nav2 bis zum Anfahrpunkt, dann Kamera → quer 2,4 cm, Winkel 2°; vier kritische Startlagen mit Querversatz und
  Schräglage: alle fertig (≤ 2,6 cm, ≤ 3,7°); Abbrechen stoppt sofort.
- Debug-Ansicht (Test-Webserver Port 8092): Videostrom (MJPEG) liefert ~3 Bilder/s, Overlay mit richtigem und mit falschem
  Wörterbuch (Hinweis „gefunden in 5X5_50/5X5_100/5X5_250“) angesehen; Farbbild geht beim Zuschauen an und danach aus;
  `/dock/save` und `/dock/start` schalten es kurz selbst ein; ohne Marker meldet `/dock/save` nach 3 s „nicht erkannt“.
- **Nicht getestet:** echte Kamera (Beleuchtung, Schärfe, Bildformat, echte Intrinsik), echter Marker, Fahren am Roboter.
  Erstes Ziel am Roboter: zuerst nur `dock/image` ansehen und `/dock/save`, dann `/dock/start` mit der Hand am Notaus.
