# Slop-Bot – Startbefehle

Jedes neue Terminal zuerst:
source ~/slop-bot_ws/install/setup.bash

## Antrieb + Sicherheit (bestromt Motoren, fragt nach sudo-Passwort, Fahren nur mit RustDesk-Sitzung)
ros2 run slop_base with_netcaps ros2 launch slop_base bringup.launch.py

## Nur Antrieb
ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py

## Nur Sicherheit (twist_mux + RustDesk-Fahrsperre)
ros2 launch slop_safety safety.launch.py

## Tastatursteuerung (eigenes Terminal; Taste halten = fahren, wasd/Pfeile, q/e Kurve, Leertaste Stopp, +/- Tempo)
ros2 run slop_teleop keyboard_teleop

## Endstufen aus / an
ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: false"
ros2 service call /diff_drive/enable std_srvs/srv/SetBool "data: true"

## Hand-Stopp / aufheben
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: true"
ros2 topic pub --once /safety/stop std_msgs/msg/Bool "data: false"

## Sensoren (Lidar + Kamera + TFs)
ros2 launch slop_sensors sensors.launch.py

## Nur Lidar
ros2 launch slop_sensors sensors.launch.py camera:=false

## Nur Kamera
ros2 launch slop_sensors sensors.launch.py lidar:=false

## Sensor-Neigung neu aus der Schwerkraft messen (Roboter still und waagrecht)
ros2 service call /mount_tf/recalibrate std_srvs/srv/Trigger

## Sensoren mit Neigung nur aus mounts.yaml
ros2 launch slop_sensors sensors.launch.py mount_from_gravity:=false

## Sensoren + RViz (Welt fest, braucht laufenden Antrieb)
ros2 launch slop_sensors view.launch.py

## Sensoren + RViz ohne Antrieb (Ansicht fest am Roboter)
ros2 launch slop_sensors view.launch.py fixed_frame:=base_footprint

## LIO / SLAM: Sensoren + FAST-LIO + RViz (beim Start 2-3 s still und waagrecht stehen lassen)
ros2 launch slop_slam lio.launch.py

## LIO ohne RViz / ohne Kamera
ros2 launch slop_slam lio.launch.py rviz:=false
ros2 launch slop_slam lio.launch.py camera:=false

## LIO-Karte speichern (nach ~/slop-bot_ws/maps/lio_map.pcd)
ros2 service call /map_save std_srvs/srv/Trigger

## IMUs plotten
ros2 run rqt_plot rqt_plot /livox/imu/linear_acceleration/z /camera/imu/linear_acceleration/y

## Lidar-Netzwerk verbinden / prüfen
nmcli con up livox-mid360
ping -c 3 192.168.1.162

## Motortest ohne ROS (aufbocken, diff_drive darf nicht laufen)
cd ~/slop-bot_ws/tests
sudo ~/ethercat_venv/bin/python3 diff_motor_test.py monitor
sudo ~/ethercat_venv/bin/python3 diff_motor_test.py jog --rpm 10 --time 3

## Hängende Prozesse beenden
pkill -9 -x livox_ros_drive; pkill -x realsense2_came; pkill -x static_transfor; pkill -x rviz2

## Bauen
cd ~/slop-bot_ws && colcon build && source install/setup.bash
