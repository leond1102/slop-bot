# Slop-Bot – TODO

## Sicherheit

- [ ] Abgründe (Treppen, Kanten) mit der Kamera erkennen.

## Navigation

- [ ] Kamera wieder in Nav2 aufnehmen (Costmaps, Collision Monitore, Sensor-Wächter).
- [ ] Sicherheitsstufe im Betrieb umschalten statt per Neustart.

## Dockingstation

- [ ] Am Roboter testen: Marker ansehen (`/dock/image`), `/dock/save`, `/dock/start`.
- [ ] Haltepunkt und Lidar-Wächter an Kontakte/Ladeanschluss anpassen, sobald die Mechanik steht.
- [ ] Nach dem Andocken: Ladestand/Ladung erkennen, Abdocken (rückwärts oder Wenden).

## Sensoren und Kalibrierung

- [ ] Livox-Zeitstempel: Gerätezeit oder PTP statt Empfangszeit am Jetson.
- [ ] FAST-LIO-Extrinsik für das Mid-360s prüfen (stehen noch Mid-360-Werte drin).
- [ ] Kamera-IMU kalibrieren (`rs-imu-calibration`).
- [ ] Gier des Lidars und Lage Lidar–Kamera bestimmen.
- [ ] Radodometrie kalibrieren (Raddurchmesser, Spurweite).
- [ ] Prüfen, ob hinter dem hinteren Ausschnitt-Quader noch Mast oder Fahne im Scan liegen.

## Antrieb

- [ ] `motor_current_ma` vom Typenschild in `diff_drive.yaml` eintragen.

## Erfahrungskarte

- [ ] Kamera als zweite Quelle (sieht auch flache Dinge).

## Später

- [ ] FAST-LIO als Komponente mit dem Lidar-Treiber (intra-process) und 16-Byte-Punkte.
