#!/usr/bin/env python3
"""Differentialantrieb der Slop-Bot-Plattform: zwei Schrittmotoren (Open Loop) am CL57E-4A über EtherCAT.

Topics:
  /cmd_vel        geometry_msgs/Twist     Sub  linear.x [m/s], angular.z [rad/s]; Rest wird ignoriert
  /wheel/odom     nav_msgs/Odometry       Pub  Radodometrie aus den ausgegebenen Schritten (odom_topic)
  /joint_states   sensor_msgs/JointState  Pub  Radwinkel [rad] und -geschwindigkeit [rad/s] (ausgegebene Schritte)
  TF              odom → base_footprint   nur mit publish_tf (Standard aus, das macht der EKF aus slop_localization)
Services:
  diff_drive/enable  std_srvs/SetBool     true = Fault-Reset + Freigabe, false = Endstufen aus

Sicherheit:
  - Watchdog: ohne neuen /cmd_vel für cmd_vel_timeout s wird auf 0 gebremst.
  - Anlaufsperre: nach dem Freigeben werden Fahrbefehle erst angenommen, wenn arm_delay s lang keiner kam
    oder ein Stopp-Befehl (alles 0) kam. Ein Sender, der schon läuft, fährt den Roboter nicht sofort los.
  - Raddrehzahl auf max_wheel_rpm begrenzt. Ist ein Befehl zu schnell, werden beide Räder gleich stark
    skaliert, damit der Roboter dieselbe Kurve fährt, nur langsamer.
  - Fehler an einem Rad schaltet beide ab (Cl57eBus).
  - Fahrsperre lock_topic (slop_safety/remote_guard, z. B. keine RustDesk-Sitzung): gesperrt bei data: true
    oder wenn länger als lock_timeout nichts kommt. Zusätzlich zu twist_mux, damit die Sperre auch greift,
    wenn twist_mux fehlt oder umgangen wird. Nach dem Aufheben gilt wieder die Anlaufsperre.
  - Auf /cmd_vel darf nur twist_mux senden (allowed_cmd_vel_nodes). Sendet noch jemand anders, steht der Roboter.
  - Absturz: Im CSP-Modus geht jeden Zyklus eine absolute Sollposition an den Treiber. Kommen keine Frames
    mehr (Jetson/Prozess abgestürzt), bleibt die Sollposition stehen und der Motor hält an. Der Treiber
    meldet danach einen Alarm (SyncCnt). Hängt nur die ROS-Node, bremst der Bus-Watchdog nach 0,3 s.

Radodometrie: Open Loop, die Odometrie zählt die ausgegebenen Schritte (Sollposition des Treibers). Verlorene
Schritte oder durchrutschende Räder merkt sie nicht, sie ist also nur so genau wie Getriebe, Raddurchmesser und
Spurweite in der Config. Sie ist ein Eingang für den EKF (slop_localization), der /odom und das TF odom → base_footprint
aus allen Sensoren bildet; deshalb Topic /wheel/odom und publish_tf: false. Ohne EKF kann publish_tf: true gesetzt werden
(nur eine Quelle für diese Transformation).

Braucht CAP_NET_RAW + CAP_NET_ADMIN für den Raw-Socket und CAP_SYS_NICE für die Echtzeit-Priorität:
  ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py

Echtzeit (rt_priority, rt_cpu, main_nice): Der EtherCAT-Zyklus läuft als SCHED_FIFO-Thread auf einem eigenen
Kern, damit andere Prozesse (SLAM, RViz usw.) ihn nicht verdrängen. Er teilt sich aber das GIL mit dem ROS-Teil dieser Node.
Deshalb kurzes GIL-Umschaltintervall und der ROS-Thread mit höherer Priorität (nice). Alle stats_period s steht
die Zyklusstatistik im Log.
"""
import gc
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from slop_base.cl57e_bus import (PARAM_DIV, PARAM_MAX_CURRENT, PARAM_RUN_MODE, PARAM_SENSOR_ERR,
                                 Cl57eBus, Cl57eError)
from slop_base.kinematics import Odometry2D, limit_wheel_speeds, wheel_speeds
from slop_base.senders import SenderCheck

TWO_PI = 2.0 * math.pi


class Wheel:
    """Umrechnung Rad ↔ Motoreinheiten für eine Seite."""

    def __init__(self, name, axis, gear, direction, joint):
        self.name, self.axis, self.gear, self.dir, self.joint = name, axis, gear, direction, joint
        self.upr = 10000          # Einheiten pro Motorumdrehung, wird aus dem Treiber gelesen
        self.last_units = None
        self.angle = 0.0          # Radwinkel seit Start [rad]

    def rad_s_to_units(self, w):
        return self.dir * w / TWO_PI * self.gear * self.upr

    def units_to_rad(self, du):
        return self.dir * du / self.upr / self.gear * TWO_PI


class DiffDriveNode(Node):
    """Dünne ROS-Hülle um Cl57eBus: /cmd_vel → Räder, Gelenkzustände zurück."""

    def __init__(self):
        super().__init__('diff_drive_node')
        for name, default in [
            ('ifname', 'enP8p1s0'),        # Ethernet-Port des EtherCAT-Busses
            ('cycle_ms', 2.0),             # EtherCAT-Zyklus
            ('dc', False),                 # Distributed Clocks (Sync0)
            ('left_axis', 3), ('left_gear', 5.18), ('left_dir', 1),
            ('right_axis', 4), ('right_gear', 13.73), ('right_dir', -1),
            ('units_per_rev', 0),          # 0 = Treiberparameter DivSet
            ('wheel_diameter', 0.07),      # m
            ('track_width', 0.24),         # Abstand der Radmitten [m]
            ('max_wheel_rpm', 10.0),       # Obergrenze am Rad
            ('wheel_acc', 6.0),            # Radbeschleunigung [rad/s²]
            ('loop', 'open'),              # open = ohne Encoder, closed = mit Encoder
            ('max_ferr_rev', 1.0),         # Schleppfehler [Motorumdrehungen], darüber Endstufen aus
            ('motor_current_ma', 0),       # Nennstrom der Motoren, 0 = nicht prüfen
            ('enable_on_start', True),
            ('cmd_vel_timeout', 0.5),      # s
            ('arm_delay', 1.0),            # s
            ('lock_topic', '/safety/remote_lock'),   # '' = keine Fahrsperre
            ('lock_timeout', 1.0),         # s ohne Nachricht auf lock_topic → gesperrt
            ('allowed_cmd_vel_nodes', ['twist_mux']),  # leere Liste = nicht prüfen
            ('unknown_sender_grace', 3.0),  # s: Publisher ohne Namen (Startphase) erst danach als Fehler melden
            ('rate', 50.0),                # Hz: Sollwerte, Odometrie, Gelenkzustände
            ('odom_topic', '/wheel/odom'),  # Rohodometrie der Räder, Eingang des EKF
            ('odom_frame', 'odom'),
            ('base_frame', 'base_footprint'),
            ('publish_tf', False),         # true nur ohne EKF: sonst zwei Quellen für odom → base_frame
            ('left_joint', 'left_wheel_joint'),
            ('right_joint', 'right_wheel_joint'),
            ('rt_priority', 80),           # SCHED_FIFO des Zyklus-Threads, 0 = aus
            ('rt_cpu', 7),                 # Kern des Zyklus-Threads, -1 = beliebig
            ('main_nice', -10),            # nice des ROS-Threads (hält das GIL kürzer), 0 = unverändert
            ('gil_switch_ms', 0.5),        # GIL-Umschaltintervall (Python-Standard 5 ms)
            ('stats_period', 10.0),        # s zwischen zwei Zyklusstatistiken im Log, 0 = aus
            ('slow_update_ms', 50.0),      # ms: dauert update() oder liegt zwischen zwei Aufrufen mehr, steht eine Warnung im Log
        ]:
            self.declare_parameter(name, default)
        self._setup_process()

        self.radius = self._p('wheel_diameter') / 2.0
        self.track = self._p('track_width')
        self.w_max = self._p('max_wheel_rpm') / 60.0 * TWO_PI
        self.wheels = [Wheel('links', self._p('left_axis'), self._p('left_gear'), self._p('left_dir'),
                             self._p('left_joint')),
                       Wheel('rechts', self._p('right_axis'), self._p('right_gear'), self._p('right_dir'),
                             self._p('right_joint'))]
        if self.radius <= 0 or self.track <= 0 or self.w_max <= 0 or self._p('wheel_acc') <= 0:
            raise Cl57eError('Parameter ungültig: wheel_diameter, track_width, max_wheel_rpm, wheel_acc > 0')
        for wh in self.wheels:
            if wh.gear <= 0 or wh.dir not in (1, -1):
                raise Cl57eError(f'Rad {wh.name}: gear > 0 und dir ±1 nötig')
        if self._p('loop') not in ('open', 'closed'):
            raise Cl57eError("loop muss 'open' oder 'closed' sein")

        self.bus = Cl57eBus(self._p('ifname'), [wh.axis for wh in self.wheels], self._p('cycle_ms') / 1000.0,
                            self._p('dc'), cmd_timeout_s=min(0.3, self._p('cmd_vel_timeout')),
                            rt_priority=self._p('rt_priority'), rt_cpu=self._p('rt_cpu'))
        try:
            name = self.bus.open()
            self._check_drive_params()
            self.bus.configure()
            self.bus.start()
        except Exception:
            self.bus.close()
            raise
        for wh in self.wheels:
            self.bus.set_limits(wh.axis, amax=self._p('wheel_acc') / TWO_PI * wh.gear * wh.upr,
                                ferr_limit=self._p('max_ferr_rev') * wh.upr)

        v_max = self.w_max * self.radius
        self.get_logger().info(
            f'{name} an {self._p("ifname")} in OP (CSP, {self._p("loop")} loop). '
            + ', '.join(f'{wh.name}: Achse {wh.axis}, 1:{wh.gear:g}, {wh.upr} Einh./U' for wh in self.wheels)
            + f'. Rad Ø {self._p("wheel_diameter") * 100:g} cm, Spur {self.track * 100:g} cm, '
            f'max {self._p("max_wheel_rpm"):g} U/min am Rad = {v_max:.3f} m/s, {2 * v_max / self.track:.2f} rad/s')
        self.get_logger().info(f'EtherCAT-Zyklus {self._p("cycle_ms"):g} ms: {self.bus.rt_status}; ROS-Thread: '
                               f'{self.proc_status}')

        self.odom = Odometry2D(self.track)
        self.last_update_t = None
        self.timing = {}
        self.last_t = None
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = None
        self.armed = False
        self.arm_since = None
        self.last_error = ''
        self.was_watchdog = False
        self.lock_msg_time = None
        self.lock_data = True
        self.locked = True
        self.lock_reason = ''
        self.allowed_senders = set(self._p('allowed_cmd_vel_nodes'))
        self.sender_check = SenderCheck(self.allowed_senders, self._p('unknown_sender_grace'))
        self.foreign_senders = []
        self.senders_blocked = False
        self.senders_waiting = False
        self.last_sender_check = None

        self.odom_pub = self.create_publisher(Odometry, self._p('odom_topic'), 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self._p('publish_tf') else None
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        if self._p('lock_topic'):
            self.create_subscription(Bool, self._p('lock_topic'), self.lock_callback, 10)
        self.create_service(SetBool, 'diff_drive/enable', self.enable_callback)
        self.create_timer(1.0 / self._p('rate'), self.update)
        if self._p('stats_period') > 0:
            self.create_timer(self._p('stats_period'), self.log_cycle_stats)

        if self._p('enable_on_start'):
            self._enable()
        # Alles bis hier Angelegte aus der Garbage Collection nehmen: kürzere GC-Pausen (halten das GIL).
        # Ohne gc.collect() davor: Das hielt bei laufendem Zyklus das GIL ~30 ms (eine Spitze von 32,7 ms)
        gc.freeze()

    def _p(self, name):
        return self.get_parameter(name).value

    def _setup_process(self):
        """ROS-Thread vom Echtzeit-Kern fernhalten, Priorität anheben, GIL schneller abgeben.
        Läuft vor dem Start des Zyklus-Threads; Fehler sind nicht fatal, nur im Log."""
        msgs = []
        sys.setswitchinterval(max(1e-4, self._p('gil_switch_ms') / 1000.0))
        msgs.append(f'GIL-Intervall {sys.getswitchinterval() * 1000:g} ms')
        cpu = self._p('rt_cpu')
        if cpu >= 0:
            others = os.sched_getaffinity(0) - {cpu}
            if others:
                os.sched_setaffinity(0, others)
                msgs.append(f'Kerne ohne {cpu}')
        nice = self._p('main_nice')
        if nice:
            try:
                os.setpriority(os.PRIO_PROCESS, 0, nice)   # Linux: gilt für den aufrufenden Thread
                msgs.append(f'nice {nice}')
            except OSError as e:
                msgs.append(f'nice {nice} NICHT gesetzt ({e.strerror})')
        self.proc_status = ', '.join(msgs)

    def log_cycle_stats(self):
        w = self.bus.window_stats()
        if not w['cycles']:
            return
        msg = (f'EtherCAT letzte {self._p("stats_period"):g} s: {w["cycles"]} Zyklen, '
               f'{w["late"]} verspätet (> {1.5 * self._p("cycle_ms"):g} ms), {w["overruns"]} Überläufe, '
               f'{w["wkc_errors"]} WKC-Fehler, max. Periode {w["max_period"] * 1000:.1f} ms')
        if w['late'] or w['wkc_errors']:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)

    def _check_drive_params(self):
        """Gefährliche Treibereinstellungen ablehnen, Einheiten pro Umdrehung bestimmen."""
        problems = []
        closed = self._p('loop') == 'closed'
        for wh in self.wheels:
            run_mode = self.bus.read_param(wh.axis, PARAM_RUN_MODE)
            if run_mode != (2 if closed else 1):
                problems.append(f'Achse {wh.axis}: RunModeSet = {run_mode}, erwartet {2 if closed else 1} '
                                f'({self._p("loop")} loop)')
            if closed and self.bus.read_param(wh.axis, PARAM_SENSOR_ERR) != 1:
                problems.append(f'Achse {wh.axis}: SensorErr = 0 (Encoderfehler würden ignoriert)')
            current = self.bus.read_param(wh.axis, PARAM_MAX_CURRENT)
            if self._p('motor_current_ma') and current > self._p('motor_current_ma'):
                problems.append(f'Achse {wh.axis}: MaxCurrent = {current} mA > motor_current_ma')
            upr = self._p('units_per_rev') or self.bus.read_param(wh.axis, PARAM_DIV)
            if not 200 <= upr <= 1_000_000:
                problems.append(f'Achse {wh.axis}: unplausible Einheiten pro Umdrehung {upr}')
            wh.upr = upr
        if problems:
            raise Cl57eError('Treiberparameter: ' + '; '.join(problems))

    def _enable(self):
        try:
            self.bus.enable()
        except Cl57eError as e:
            self.get_logger().error(f'Freigabe fehlgeschlagen: {e}')
            return False, str(e)
        self.armed = False
        self.arm_since = self.get_clock().now()
        self.get_logger().info(f'Räder freigegeben. Fahrbefehle ab {self._p("arm_delay"):g} s Ruhe '
                               'oder nach einem Stopp-Befehl')
        return True, 'freigegeben'

    # ---------------------------------------------------------------- Callbacks
    def cmd_vel_callback(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().warn('Ungültiger /cmd_vel ignoriert', throttle_duration_sec=2.0)
            return
        now = self.get_clock().now()
        if not self.armed:
            if v == 0.0 and w == 0.0:
                self._arm()
            else:
                self.arm_since = now     # Anlaufsperre: Ruhezeit beginnt von vorn
                self.get_logger().warn('Fahrbefehl während der Anlaufsperre ignoriert', throttle_duration_sec=2.0)
                return
        self.cmd = (v, w)
        self.last_cmd_time = now

    def _arm(self):
        if not self.armed:
            self.armed = True
            self.get_logger().info('Fahrbefehle werden angenommen')

    def lock_callback(self, msg):
        self.lock_data = msg.data
        self.lock_msg_time = self.get_clock().now()

    def _check_lock(self, now):
        """Fahrsperre auswerten. Beim Aufheben neu scharf schalten (Anlaufsperre)."""
        if not self._p('lock_topic'):
            locked, reason = False, ''
        elif self.lock_msg_time is None:
            locked, reason = True, f'noch keine Nachricht auf {self._p("lock_topic")}'
        elif (now - self.lock_msg_time).nanoseconds * 1e-9 > self._p('lock_timeout'):
            locked, reason = True, f'{self._p("lock_topic")} seit über {self._p("lock_timeout"):g} s still'
        elif self.lock_data:
            locked, reason = True, f'{self._p("lock_topic")} = true'
        else:
            locked, reason = False, ''
        if locked != self.locked or reason != self.lock_reason:
            if locked:
                self.get_logger().warn(f'Fahrsperre: {reason}')
            else:
                self.get_logger().info('Fahrsperre aufgehoben')
                self.armed = False
                self.arm_since = now
        self.locked, self.lock_reason = locked, reason
        return locked

    def _check_senders(self, now):
        """Einmal pro Sekunde: sendet außer den erlaubten Nodes jemand auf /cmd_vel?"""
        if not self.allowed_senders:
            return False
        # sonst 1×/s, solange ein Name noch unbekannt ist alle 0,2 s (Startphase schneller vorbei)
        period = 0.2e9 if self.senders_waiting else 1e9
        if self.last_sender_check is None or (now - self.last_sender_check).nanoseconds > period:
            self.last_sender_check = now
            names = [i.node_name for i in self.get_publishers_info_by_topic('/cmd_vel')]
            blocked, foreign, waiting = self.sender_check.check(names, now.nanoseconds * 1e-9)
            if waiting and not self.senders_waiting:
                self.get_logger().info('Publisher auf /cmd_vel noch ohne Namen (Startphase, meist twist_mux) – '
                                       'warte, bis ROS ihn kennt')
            if foreign and foreign != self.foreign_senders:
                self.get_logger().error(f'Fremde Publisher auf /cmd_vel: {foreign} → Stopp. Nur über twist_mux '
                                        'fahren (z. B. teleop mit -r cmd_vel:=teleop/cmd_vel)')
            elif not blocked and self.senders_blocked:
                self.get_logger().info('Keine fremden Publisher mehr auf /cmd_vel' if self.foreign_senders
                                       else 'Publisher auf /cmd_vel geprüft: nur twist_mux')
                self.armed = False
                self.arm_since = now
            self.foreign_senders = foreign
            self.senders_blocked = blocked
            self.senders_waiting = waiting
        return self.senders_blocked

    def enable_callback(self, request, response):
        if request.data:
            response.success, response.message = self._enable()
        else:
            self.bus.disable()
            self.armed = False
            self.get_logger().info('Endstufen aus')
            response.success, response.message = True, 'Endstufen aus'
        return response

    # ---------------------------------------------------------------- Zyklus
    def update(self):
        """Zeitüberwachung um _update: Blockiert der ROS-Thread (Graph-Abfrage, Publish, GIL), steht im Log wo."""
        t0 = time.perf_counter()
        gap = None if self.last_update_t is None else t0 - self.last_update_t
        self.last_update_t = t0
        self.timing = {}
        try:
            self._update()
        finally:
            took = time.perf_counter() - t0
            limit = self._p('slow_update_ms') / 1000.0
            if limit > 0 and (took > limit or (gap is not None and gap > 2 * limit + 1.0 / self._p('rate'))):
                parts = ', '.join(f'{k} {v * 1000:.0f} ms' for k, v in self.timing.items())
                self.get_logger().warn(
                    f'ROS-Thread blockiert: update() {took * 1000:.0f} ms'
                    + (f', {gap * 1000:.0f} ms seit dem letzten Aufruf' if gap is not None else '')
                    + (f' ({parts})' if parts else ''), throttle_duration_sec=1.0)

    def _update(self):
        st = self.bus.status()
        if st['comm_lost']:
            raise Cl57eError(f'EtherCAT-Kommunikation verloren ({st["thread_error"] or "keine Antwort"})')
        if st['error'] and st['error'] != self.last_error:
            self.get_logger().error(f'{st["error"]} → Endstufen aus. Neu freigeben über diff_drive/enable')
        self.last_error = st['error']
        if st['watchdog'] and not self.was_watchdog:
            self.get_logger().warn('Bus-Watchdog: Sollwerte kamen nicht rechtzeitig, Räder gebremst')
        self.was_watchdog = st['watchdog']

        now = self.get_clock().now()
        t = time.perf_counter()
        blocked = self._check_lock(now)
        self.timing['Fahrsperre'] = time.perf_counter() - t
        t = time.perf_counter()
        blocked |= self._check_senders(now)
        self.timing['Publisher-Prüfung'] = time.perf_counter() - t
        if not self.armed and st['enabled'] and self.arm_since is not None \
                and (now - self.arm_since).nanoseconds * 1e-9 >= self._p('arm_delay'):
            self._arm()

        # Sollwerte
        v, w = self.cmd
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > self._p('cmd_vel_timeout'):
            v, w = 0.0, 0.0
        if not self.armed or not st['enabled'] or blocked:
            v, w = 0.0, 0.0
        wl, wr = wheel_speeds(v, w, self.track, self.radius)
        wl, wr, k = limit_wheel_speeds(wl, wr, self.w_max)
        if k < 1.0:
            self.get_logger().info(f'Fahrbefehl auf {k * 100:.0f} % skaliert (max_wheel_rpm)',
                                   throttle_duration_sec=5.0)
        self.bus.set_velocity({wh.axis: wh.rad_s_to_units(ws) for wh, ws in zip(self.wheels, (wl, wr))})

        if st['valid']:
            t = time.perf_counter()
            self.publish_state(st, now)
            self.timing['publish'] = time.perf_counter() - t

    def publish_state(self, st, now):
        t = st['t']
        dt = 0.0 if self.last_t is None else t - self.last_t
        d_angle = []
        for wh in self.wheels:
            units = st['actual'][wh.axis]
            da = 0.0 if wh.last_units is None else wh.units_to_rad(units - wh.last_units)
            wh.last_units = units
            d_angle.append(da)
        if dt <= 0.0 and self.last_t is not None:
            return                       # keine neue Antwort vom Bus seit dem letzten Mal
        self.last_t = t

        # Sprung im Schrittzähler (z. B. nach Treiberneustart) nicht als Fahrt werten
        limit = self.w_max * 3.0 * max(dt, 0.02) + 0.2
        if any(abs(da) > limit for da in d_angle):
            self.get_logger().warn(f'Sprung im Positionszähler ({d_angle[0]:+.2f}/{d_angle[1]:+.2f} rad), '
                                   'Odometrie-Schritt verworfen')
            d_angle = [0.0, 0.0]
        for wh, da in zip(self.wheels, d_angle):
            wh.angle += da
        ds, dyaw = self.odom.update(d_angle[0] * self.radius, d_angle[1] * self.radius)
        vx = ds / dt if dt > 0 else 0.0
        wz = dyaw / dt if dt > 0 else 0.0

        stamp = now.to_msg()
        qz, qw = math.sin(self.odom.yaw / 2.0), math.cos(self.odom.yaw / 2.0)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._p('odom_frame')
        msg.child_frame_id = self._p('base_frame')
        msg.pose.pose.position.x = self.odom.x
        msg.pose.pose.position.y = self.odom.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Diagonale x, y, z, roll, pitch, yaw. Nicht gemessene Achsen sehr unsicher, x/y/yaw grobe Schätzung (Open Loop)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = msg.pose.covariance[21] = msg.pose.covariance[28] = 1e6
        msg.pose.covariance[35] = 0.05
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz
        msg.twist.covariance[0] = 0.001
        msg.twist.covariance[7] = msg.twist.covariance[14] = msg.twist.covariance[21] = 1e6
        msg.twist.covariance[28] = 1e6
        msg.twist.covariance[35] = 0.01
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._p('odom_frame')
            tf.child_frame_id = self._p('base_frame')
            tf.transform.translation.x = self.odom.x
            tf.transform.translation.y = self.odom.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        js.name = [wh.joint for wh in self.wheels]
        js.position = [wh.angle for wh in self.wheels]
        js.velocity = [da / dt if dt > 0 else 0.0 for da in d_angle]
        self.joint_pub.publish(js)

    def shutdown(self):
        s = self.bus.stats
        self.bus.close()
        self.get_logger().info(f'Beendet. Zyklen {s["cycles"]}, Überläufe {s["overruns"]}, '
                               f'WKC-Fehler {s["wkc_errors"]}, max. Periode {s["max_period"] * 1000:.1f} ms')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DiffDriveNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Cl57eError as e:
        print(f'[diff_drive_node] FEHLER: {e}')
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
