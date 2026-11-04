#!/usr/bin/env python3
"""Kalibrierungen messen und in calib/ speichern.

  ros2 run slop_calibration calibrate list              # Kalibrierungen und Stand der gespeicherten Werte
  ros2 run slop_calibration calibrate all               # alle Kalibrierungen nacheinander
  ros2 run slop_calibration calibrate gyro              # nur Gyro-Bias (Livox- und Kamera-IMU)
  ros2 run slop_calibration calibrate mount             # nur Sensorneigung (Roll/Pitch) aus der Schwerkraft
  ros2 run slop_calibration calibrate all --duration 20 # längeres Messfenster (Standard 10 s)

Voraussetzung: Sensoren laufen (ros2 launch slop_sensors sensors.launch.py), der Roboter steht still und waagrecht
auf ebenem Boden und wird während der Messung nicht berührt. Sensoren, von denen keine IMU-Daten kommen, werden
übersprungen; die anderen werden trotzdem gespeichert. Ergebnisse: calib/gyro.yaml, calib/mounts.yaml
(Ordner siehe slop_calibration/store.py). Weitere Kalibrierungen: Eintrag in CALIBRATIONS.

Gyro-Bias und Neigung werden aus demselben Messfenster gerechnet (eine Messung für beide).
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

from slop_calibration import store
from slop_calibration.still import G, StillStats, rotate, tilt_from_gravity

# Quellen: Topic, Einheit der Beschleunigung (Livox: g), Frame der Neigung im URDF
SENSORS = {
    'livox': {'topic': '/livox/imu', 'accel_scale': G, 'mount_frame': 'livox_frame'},
    'camera': {'topic': '/camera/imu', 'accel_scale': 1.0, 'mount_frame': 'camera_link'},
}

# Name → Beschreibung
CALIBRATIONS = {
    'gyro': 'Gyro-Bias der IMUs (rad/s) → calib/gyro.yaml, vom imu_adapter beim Start geladen',
    'mount': 'Roll/Pitch der Sensoren aus der Schwerkraft (rad) → calib/mounts.yaml, vom URDF geladen',
}

MAX_GYRO_STD = 0.02       # rad/s, darüber bewegt sich der Roboter
MAX_ACCEL_STD = 0.3       # m/s²
GRAVITY_TOLERANCE = 0.1   # |a| darf höchstens 10 % von g abweichen (sonst falsche Einheit oder Beschleunigung)


class Collector(Node):
    def __init__(self):
        super().__init__('calibrate')
        self.stats = {name: StillStats() for name in SENSORS}
        self.frames = {}
        self.collecting = False
        for name, cfg in SENSORS.items():
            self.create_subscription(Imu, cfg['topic'], lambda m, n=name: self.on_imu(n, m), qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_imu(self, name, msg):
        self.frames[name] = msg.header.frame_id
        if not self.collecting:
            return
        s = SENSORS[name]['accel_scale']
        a = msg.linear_acceleration
        w = msg.angular_velocity
        self.stats[name].add(self.get_clock().now().nanoseconds * 1e-9, (w.x, w.y, w.z),
                             (a.x * s, a.y * s, a.z * s))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def measure(node, duration, wait):
    """Messfenster für alle Sensoren, die Daten liefern. Gibt {name: StillStats} der brauchbaren Sensoren zurück."""
    print(f'Warte bis zu {wait:g} s auf IMU-Daten ...')
    end = time.monotonic() + wait
    while rclpy.ok() and time.monotonic() < end and not node.frames:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.frames:
        print('FEHLER: Keine IMU-Daten. Laufen die Sensoren? (ros2 launch slop_sensors sensors.launch.py)')
        return {}
    node.spin_for(0.5)          # auch die langsamer startende zweite IMU abwarten
    print(f'Messe {duration:g} s – Roboter NICHT bewegen: ' + ', '.join(sorted(node.frames)))
    node.collecting = True
    node.spin_for(duration)
    node.collecting = False
    good = {}
    for name in sorted(node.frames):
        st = node.stats[name]
        if st.n < 10:
            print(f'  {name}: nur {st.n} Messwerte, übersprungen')
            continue
        gm, gs, am, asd = st.result()
        g = math.sqrt(sum(v * v for v in am))
        print(f'  {name}: {st.n} Werte ({st.n / max(st.duration, 1e-6):.0f} Hz), Gyro-Streuung {max(gs):.4f} rad/s, '
              f'Beschleunigungsstreuung {max(asd):.3f} m/s², |a| = {g:.2f} m/s²')
        if max(gs) > MAX_GYRO_STD or max(asd) > MAX_ACCEL_STD:
            print(f'  {name}: ZU UNRUHIG (Roboter bewegt oder Boden vibriert) → übersprungen')
            continue
        good[name] = st
    return good


def calibrate_gyro(node, good):
    for name, st in good.items():
        gm, gs, _, _ = st.result()
        path = store.save_entry('gyro', name, {'frame': node.frames[name], 'bias': [round(v, 6) for v in gm],
                                              'samples': st.n})
        print(f'  gyro {name}: Bias [rad/s] x {gm[0]:+.5f}, y {gm[1]:+.5f}, z {gm[2]:+.5f} → {path}')
    return bool(good)


def calibrate_mount(node, good):
    done = False
    for name, st in good.items():
        _, _, am, _ = st.result()
        g = math.sqrt(sum(v * v for v in am))
        if abs(g - G) > GRAVITY_TOLERANCE * G:
            print(f'  mount {name}: |a| = {g:.2f} m/s² weicht zu stark von g ab (Einheit?) → übersprungen')
            continue
        frame = node.frames[name]
        target = SENSORS[name]['mount_frame']
        a = am
        if frame != target:       # Schwerkraft vom IMU-Frame in den Frame des URDF drehen (Kamera: TF vom Treiber)
            try:
                tf = node.tf_buffer.lookup_transform(target, frame, rclpy.time.Time())
            except Exception as e:
                print(f'  mount {name}: kein TF {frame} → {target} ({e}) → übersprungen')
                continue
            q = tf.transform.rotation
            a = rotate((q.x, q.y, q.z, q.w), am)
        roll, pitch = tilt_from_gravity(a)
        path = store.save_entry('mounts', name, {'frame': target, 'roll': round(roll, 5), 'pitch': round(pitch, 5)})
        print(f'  mount {name}: Roll {math.degrees(roll):+.2f}°, Pitch {math.degrees(pitch):+.2f}° ({target}) → {path}')
        done = True
    return done


RUNNERS = {'gyro': calibrate_gyro, 'mount': calibrate_mount}


def list_calibrations():
    print(f'Ordner: {store.calib_dir()}\n')
    for name, text in CALIBRATIONS.items():
        print(f'{name:7s} {text}')
    print()
    for fname, sensors in (('gyro', SENSORS), ('mounts', SENSORS)):
        for sensor in sensors:
            e = store.load_entry(fname, sensor)
            age = store.age_days(e) if e else None
            state = 'nicht kalibriert' if e is None else f'{e.get("stamp", "?")}' + (f' ({age:.1f} Tage alt)' if age is not None else '')
            print(f'  {fname:7s} {sensor:7s} {state}')


def main(argv=None):
    p = argparse.ArgumentParser(description='Kalibrierungen messen und in calib/ speichern')
    p.add_argument('what', nargs='+', help='list, all oder: ' + ', '.join(CALIBRATIONS))
    p.add_argument('--duration', type=float, default=10.0, help='Messfenster in s (Standard 10)')
    p.add_argument('--wait', type=float, default=15.0, help='Wartezeit auf IMU-Daten in s (Standard 15)')
    args = p.parse_args(argv if argv is not None else [a for a in sys.argv[1:] if not a.startswith('--ros-args')])

    if args.what == ['list']:
        list_calibrations()
        return 0
    names = list(CALIBRATIONS) if 'all' in args.what else args.what
    unknown = [n for n in names if n not in CALIBRATIONS]
    if unknown:
        p.error(f'unbekannt: {", ".join(unknown)} (möglich: all, list, {", ".join(CALIBRATIONS)})')

    rclpy.init()
    node = Collector()
    try:
        good = measure(node, args.duration, args.wait)
        if not good:
            return 1
        node.spin_for(1.0)       # TF der Kamera abwarten
        ok = {n: RUNNERS[n](node, good) for n in names}
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print('\nFertig. Gespeichert: ' + (', '.join(n for n, v in ok.items() if v) or 'nichts'))
    if not all(ok.values()):
        print('Nicht kalibriert: ' + ', '.join(n for n, v in ok.items() if not v))
    return 0 if all(ok.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
