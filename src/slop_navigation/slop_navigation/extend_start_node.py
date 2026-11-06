#!/usr/bin/env python3
"""Startpose zum Karte-Erweitern am aktuellen Scan bestimmen (läuft in mapping.launch.py extend:=…, VOR slam_toolbox).

  /scan   sensor_msgs/LaserScan   Sub

Wartet auf Scans (Roboter steht), sucht die Pose wie relocalize (map_extend.find_start_pose: um die Vermutung, sonst in
der ganzen Karte), schreibt sie nach out_file (YAML x, y, yaw) und beendet sich mit 0. Findet sie nichts Eindeutiges,
endet sie mit 1 – dann startet slam_toolbox nicht (sonst würde die neue Karte falsch an die alte angehängt).
"""
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from slop_navigation.map_extend import ExtendError, find_start_pose, initial_guess, load_map_model
from slop_navigation.relocalize_core import scan_points


class ExtendStart(Node):
    def __init__(self):
        super().__init__('extend_start')
        for name, default in [('map_base', ''), ('out_file', ''), ('x', ''), ('y', ''), ('yaw', ''),
                              ('min_score', 0.80), ('timeout', 40.0), ('settle', 2.0)]:
            self.declare_parameter(name, default)
        self.scan = None
        self.first = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def _p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg):
        self.scan = msg
        if self.first is None:
            self.first = time.monotonic()

    def run(self):
        log = self.get_logger()
        base = self._p('map_base')
        try:
            model = load_map_model(base)
            guess, origin = initial_guess(base, self._p('x'), self._p('y'), self._p('yaw'))
        except (OSError, ValueError, KeyError, ExtendError) as e:
            log.error(f'Karte erweitern: {e}')
            return 1
        st = model.stats
        log.info(f'Startpose suchen in {base} ({st["fest"]} feste Zellen als Referenz); Vermutung: '
                 + (f'x {guess.x:.2f}, y {guess.y:.2f}, Gier {guess.yaw:.2f} ({origin})' if guess else origin))
        t0 = time.monotonic()
        while rclpy.ok() and (self.first is None or time.monotonic() - self.first < self._p('settle')):
            if time.monotonic() - t0 > self._p('timeout'):
                log.error(f'Karte erweitern: nach {self._p("timeout"):g} s kein /scan – läuft der Lidar?')
                return 1
            rclpy.spin_once(self, timeout_sec=0.1)
        s = self.scan
        r, a = scan_points(s.ranges, s.angle_min, s.angle_increment, max(0.2, s.range_min), min(15.0, s.range_max))
        t1 = time.monotonic()
        try:
            pose, why = find_start_pose(model, r, a, guess, self._p('min_score'))
        except ExtendError as e:
            log.error(f'Karte erweitern: {e}')
            return 1
        with open(self._p('out_file'), 'w', encoding='utf-8') as f:
            yaml.safe_dump({'x': float(pose.x), 'y': float(pose.y), 'yaw': float(pose.yaw)}, f)
        log.info(f'Startpose: x {pose.x:.2f} m, y {pose.y:.2f} m, Gier {pose.yaw:.2f} rad – {why} '
                 f'({time.monotonic() - t1:.1f} s). slam_toolbox startet dort.')
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = ExtendStart()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
