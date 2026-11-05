#!/usr/bin/env python3
"""Livox-IMU für den EKF aufbereiten.

  /livox/imu   sensor_msgs/Imu  Sub  Rohdaten des Treibers (Beschleunigung in g, ohne Kovarianzen, Orientierung ungültig)
  /imu/livox   sensor_msgs/Imu  Pub  Beschleunigung in m/s², Gyro ohne Bias, Kovarianzen gesetzt, Orientierung als
                                     ungültig markiert (orientation_covariance[0] = -1); je `decimation` Eingangswerte
                                     gemittelt (200 Hz → 50 Hz): weniger Rauschen und Last, der EKF läuft mit 50 Hz
Services:
  imu_adapter/recalibrate  std_srvs/Trigger  Gyro-Bias neu bestimmen (Roboter dabei stillhalten)

Gyro-Bias: Liegt calib/gyro.yaml mit einem Eintrag für den Sensor vor (ros2 run slop_calibration calibrate gyro),
gilt dieser sofort, ohne Wartezeit. Sonst wird die ersten bias_calibration_s Sekunden nach dem Start der Mittelwert des
Gyros gebildet und danach abgezogen; der Roboter muss dabei stehen. Bewegt er sich (Streuung über max_calibration_std),
wird das Fenster verworfen und neu begonnen. Solange kein Bias vorliegt, wird nichts veröffentlicht; der EKF fährt dann
nur auf den Rädern.
Der Frame bleibt livox_frame; der EKF rechnet über das TF aus dem URDF in base_footprint um.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from slop_calibration import store
from slop_localization.imu import GyroBias, accel_g_to_ms2, diag_covariance


class ImuAdapter(Node):
    def __init__(self):
        super().__init__('imu_adapter')
        for name, default in [
            ('input_topic', '/livox/imu'),
            ('output_topic', '/imu/livox'),
            ('accel_scale', 9.80665),            # m/s² je Einheit des Eingangs (Livox: g)
            ('gyro_stddev', 0.01),               # rad/s, Rauschen + Restbias; kleiner = der EKF vertraut dem Gyro mehr
            ('accel_stddev', 0.2),               # m/s²
            ('bias_calibration_s', 5.0),         # s Fensterlänge, Roboter muss stehen
            ('max_calibration_std', 0.02),       # rad/s, darüber gilt das Fenster als "Roboter bewegt sich"
            ('sensor_name', 'livox'),            # Schlüssel in calib/gyro.yaml
            ('use_stored_bias', True),           # false: immer neu messen
            ('decimation', 4),                   # je n Eingangswerte ein gemittelter Ausgangswert (1 = jeden weiterreichen)
            ('max_stamp_offset', 0.5),           # s Abweichung Zeitstempel ↔ Jetson-Uhr, darüber Warnung
        ]:
            self.declare_parameter(name, default)
        self.scale = self._p('accel_scale')
        self.gyro_cov = diag_covariance(self._p('gyro_stddev'))
        self.accel_cov = diag_covariance(self._p('accel_stddev'))
        self.bias = GyroBias(self._p('bias_calibration_s'), self._p('max_calibration_std'))
        self.last_rejected = 0
        self.group = []                    # gesammelte Eingangswerte (Stempel, Gyro, Beschleunigung, Header)
        stored = store.load_entry('gyro', self._p('sensor_name')) if self._p('use_stored_bias') else None
        if stored and len(stored.get('bias', [])) == 3:
            self.bias.set(stored['bias'])
            age = store.age_days(stored)
            self.get_logger().info('Gyro-Bias aus calib/gyro.yaml ({}): x {:+.5f}, y {:+.5f}, z {:+.5f} rad/s{}'.format(
                stored.get('stamp', '?'), *self.bias.bias, f', {age:.1f} Tage alt' if age is not None else ''))
        self.pub = self.create_publisher(Imu, self._p('output_topic'), 10)
        self.create_subscription(Imu, self._p('input_topic'), self.callback, qos_profile_sensor_data)
        self.create_service(Trigger, 'imu_adapter/recalibrate', self.recalibrate)
        if self.bias.ready:
            self.get_logger().info(f'{self._p("input_topic")} → {self._p("output_topic")}')
        else:
            self.get_logger().info(f'{self._p("input_topic")} → {self._p("output_topic")}; keine gespeicherte Kalibrierung '
                                   f'(ros2 run slop_calibration calibrate gyro) – Gyro-Bias wird '
                                   f'{self._p("bias_calibration_s"):g} s lang gemessen, Roboter stillhalten')

    def _p(self, name):
        return self.get_parameter(name).value

    def recalibrate(self, request, response):
        self.bias.restart()
        self.get_logger().info('Gyro-Bias wird neu gemessen (nicht gespeichert; dauerhaft: '
                               'ros2 run slop_calibration calibrate gyro) – Roboter stillhalten')
        response.success, response.message = True, 'Kalibrierfenster gestartet'
        return response

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        if self.bias.add(t, gyro):
            self.get_logger().info('Gyro-Bias [rad/s]: x {:+.5f}, y {:+.5f}, z {:+.5f}'.format(*self.bias.bias))
        if self.bias.rejected != self.last_rejected:
            self.last_rejected = self.bias.rejected
            self.get_logger().warn('Roboter bewegt sich oder die IMU rauscht stark – Kalibrierfenster verworfen, '
                                   'neuer Versuch', throttle_duration_sec=5.0)
        if not self.bias.ready:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if abs(now - t) > self._p('max_stamp_offset'):
            self.get_logger().warn(f'IMU-Zeitstempel weicht {now - t:+.2f} s von der Jetson-Uhr ab – der EKF ordnet die '
                                   'Messung falsch ein (Livox ohne Zeitsynchronisation)', throttle_duration_sec=5.0)

        accel = accel_g_to_ms2((msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
                               self.scale)
        self.group.append((msg.header, gyro, accel))
        if len(self.group) < self._p('decimation'):
            return
        group, self.group = self.group, []
        n = len(group)
        mean = lambda k, i: sum(g[k][i] for g in group) / n      # noqa: E731
        out = Imu()
        out.header = group[n // 2][0]                            # Stempel der mittleren Messung
        gx, gy, gz = self.bias.correct((mean(1, 0), mean(1, 1), mean(1, 2)))
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gx, gy, gz
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (
            mean(2, 0), mean(2, 1), mean(2, 2))
        out.orientation_covariance[0] = -1.0          # keine Orientierung
        out.angular_velocity_covariance = self.gyro_cov
        out.linear_acceleration_covariance = self.accel_cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:        # zweites SIGINT vom Launch während des Aufräumens
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
