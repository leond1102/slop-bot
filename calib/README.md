# calib/ – Kalibrierergebnisse

Messwerte dieses einen Roboters. Sie liegen hier und nicht in `install/`, damit Bauen sie nicht löscht, und werden
mit Git versioniert (kleine Textdateien). Gemessen wird mit `ros2 run slop_calibration calibrate all`
(Befehle: [STARTBEFEHLE.md](../STARTBEFEHLE.md)). Anderer Ordner: Umgebungsvariable `SLOP_CALIB_DIR`.

| Datei | Inhalt | Wer liest sie |
|---|---|---|
| `gyro.yaml` | Gyro-Bias je IMU (`livox`, `camera`) in rad/s | `imu_adapter` (slop_localization): gilt beim Start sofort, keine Wartezeit mehr |
| `mounts.yaml` | Roll/Pitch der Sensoren in rad (aus der Schwerkraft, Roboter waagrecht) | `description.launch.py` (slop_description): als rpy der Sensorgelenke im URDF |

Beide Dateien gibt es erst nach der ersten Kalibrierung. Ohne sie gilt: Gyro-Bias wird bei jedem Start 5 s gemessen,
die Sensoren gelten als waagrecht eingebaut.

Der Bias driftet mit der Temperatur: gelegentlich neu messen (nach dem Aufwärmen, nach einem Umbau, bei Drehdrift).
Die Neigung ändert sich nur, wenn ein Sensor umgebaut oder verstellt wird. Gier (yaw) und die Positionen x/y/z lassen sich
aus der Schwerkraft nicht messen und stehen in `slop_bot.urdf.xacro`.

## Weitere Kalibrierungen (noch nicht umgesetzt)

Neue Kalibrierung: in `slop_calibration/calibrate.py` einen Eintrag in `CALIBRATIONS` und `RUNNERS` ergänzen, das Ergebnis
mit `store.save_entry(<datei>, <sensor>, {...})` speichern und dort lesen, wo es gebraucht wird.

- Radodometrie: Faktor für Raddurchmesser und Spurweite (aus gefahrener Strecke und Drehung gegen Messung).
- Beschleunigungs-Bias der IMUs (mehrere Lagen).
- Kamera-IMU (`rs-imu-calibration`) und Lidar–Kamera-Extrinsik (Position, Gier).
