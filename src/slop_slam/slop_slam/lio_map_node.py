#!/usr/bin/env python3
"""Leichte LIO-Karte für RViz und zum Speichern: Voxelraster aus den registrierten Scans von FAST-LIO.

FAST-LIOs eigene /Laser_map hängt jede Sekunde einen Scan an und schickt die ganze, unbegrenzt wachsende Wolke
jede Sekunde neu (RViz wird nach einigen Minuten zäh). Diese Node behält je Voxel (`voxel` m) einen Punkt und
schickt die Karte nur alle `period` s.

Topics:
  /cloud_registered   sensor_msgs/PointCloud2  Sub  Scans in camera_init (FAST-LIO)
  /lio/map            sensor_msgs/PointCloud2  Pub  Karte in camera_init (Voxelraster, transient local)
Services:
  ~/save              std_srvs/Trigger         Karte als PCD nach map_file schreiben
  ~/clear             std_srvs/Trigger         Karte leeren
"""
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    xyz = np.empty((n, 3), np.float32)
    for i, k in enumerate('xyz'):
        xyz[:, i] = raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
    return xyz[np.isfinite(xyz).all(axis=1)]


def write_pcd(path, xyz):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    header = ('# .PCD v0.7 - slop_slam/lio_map\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.ascontiguousarray(xyz, dtype=np.float32).tobytes())


class LioMapNode(Node):

    def __init__(self):
        super().__init__('lio_map')
        for name, default in [
            ('voxel', 0.05),           # m
            ('period', 2.0),           # s zwischen zwei Veröffentlichungen
            ('scan_every', 2),         # nur jeden n-ten Scan einrechnen (spart CPU, Karte füllt sich trotzdem)
            ('frame', 'camera_init'),
            ('map_file', os.path.expanduser('~/slop-bot_ws/maps/lio_map.pcd')),
        ]:
            self.declare_parameter(name, default)
        self.voxel = self.get_parameter('voxel').value
        self.keys = np.zeros((0,), np.int64)       # sortierte Voxel-Schlüssel
        self.points = np.zeros((0, 3), np.float32)
        self.dirty = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, '/lio/map', qos)
        self.scan_every = max(1, int(self.get_parameter('scan_every').value))
        self.scan_count = 0
        # roh empfangen, nur jeden scan_every-ten Scan entpacken
        self.create_subscription(PointCloud2, '/cloud_registered', self.scan_raw, 10, raw=True)
        self.create_service(Trigger, '~/save', self.save_cb)
        self.create_service(Trigger, '~/clear', self.clear_cb)
        self.create_timer(self.get_parameter('period').value, self.publish)

    def scan_raw(self, data):
        self.scan_count += 1
        if self.scan_count % self.scan_every:
            return
        self.scan_cb(deserialize_message(data, PointCloud2))

    def scan_cb(self, msg):
        xyz = cloud_xyz(msg)
        if len(xyz) == 0:
            return
        v = np.floor(xyz / self.voxel).astype(np.int64)
        # 21 Bit je Achse (±1 M Voxel = ±52 km bei 5 cm)
        k = ((v[:, 0] + (1 << 20)) << 42) | ((v[:, 1] + (1 << 20)) << 21) | (v[:, 2] + (1 << 20))
        k, idx = np.unique(k, return_index=True)
        # self.keys ist sortiert: Binärsuche statt np.isin
        pos = np.searchsorted(self.keys, k)
        new = (pos >= len(self.keys)) | (self.keys[np.minimum(pos, len(self.keys) - 1)] != k) if len(self.keys) else np.ones(len(k), bool)
        if not new.any():
            return
        self.keys = np.concatenate([self.keys, k[new]])
        self.points = np.concatenate([self.points, xyz[idx[new]]])
        order = np.argsort(self.keys)
        self.keys, self.points = self.keys[order], self.points[order]
        self.dirty = True

    def publish(self):
        if not self.dirty:
            return
        self.dirty = False
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame').value
        msg.height, msg.width = 1, len(self.points)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(self.points), True
        msg.data = self.points.tobytes()
        self.pub.publish(msg)

    def save_cb(self, request, response):
        path = self.get_parameter('map_file').value
        write_pcd(path, self.points)
        response.success = True
        response.message = f'{len(self.points)} Punkte nach {path}'
        self.get_logger().info(response.message)
        return response

    def clear_cb(self, request, response):
        self.keys = np.zeros((0,), np.int64)
        self.points = np.zeros((0, 3), np.float32)
        self.dirty = True
        response.success, response.message = True, 'Karte geleert'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LioMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
