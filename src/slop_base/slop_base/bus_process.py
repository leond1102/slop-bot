#!/usr/bin/env python3
"""EtherCAT-Busprozess des Slop-Bots: CL57E-4A im 2-ms-Takt, ohne ROS (kein rclpy, kein DDS).

    ros2 run slop_base with_netcaps ros2 launch slop_base diff_drive.launch.py      # startet ihn mit der Node
    ros2 run slop_base with_netcaps ros2 run slop_base ethercat_bus --config <diff_drive.yaml> [--gripper-config <gripper.yaml>]

Liest dieselbe Parameterdatei wie diff_drive_node (Block diff_drive_node/ros__parameters), öffnet den Bus, prüft die
Treiberparameter, fährt ihn in OP und bedient dann genau eine diff_drive_node über bus_link (Unix-Socket).
Mit --gripper-config bedient er zusätzlich die zwei Greifer-Achsen (Positionsfahrten, eigene Freigabe) für die
gripper_node. Der Zyklus läuft als SCHED_FIFO-Thread (rt_priority) auf Kern rt_cpu; der Hauptthread nimmt nur Socket-Nachrichten an.

Sicherheit: Watchdog (keine Sollwerte für cmd_timeout s → bremsen), Node still für node_timeout s → Endstufen aus,
Fehler an einer Achse → beide aus, EtherCAT-Verbindung verloren → Prozess endet (der Treiber meldet Alarm).
Beenden mit Strg+C/SIGTERM: bremsen, Endstufen aus, Bus auf INIT.
"""
import argparse
import gc
import math
import os
import signal
import sys
import time

import yaml

from slop_base.bus_link import DEFAULT_NAME, BusServer
from slop_base.cl57e_bus import (PARAM_DIV, PARAM_MAX_CURRENT, PARAM_RUN_MODE, PARAM_SENSOR_ERR, Cl57eBus,
                                 Cl57eError)

TWO_PI = 2.0 * math.pi

# Was der Busprozess aus diff_drive.yaml braucht (Standardwerte wie in diff_drive_node)
DEFAULTS = {
    'ifname': 'enP8p1s0', 'cycle_ms': 2.0, 'dc': False,
    'left_axis': 3, 'left_gear': 5.18, 'right_axis': 4, 'right_gear': 13.73,
    'units_per_rev': 0, 'wheel_acc': 6.0, 'loop': 'open', 'max_ferr_rev': 1.0, 'motor_current_ma': 0,
    'cmd_vel_timeout': 0.5, 'rt_priority': 80, 'rt_cpu': 7, 'gil_switch_ms': 0.5, 'stats_period': 10.0,
    'node_timeout': 2.0, 'bus_socket': DEFAULT_NAME,
}


# Was der Busprozess aus gripper.yaml braucht (Block gripper_node/ros__parameters)
GRIP_DEFAULTS = {'axis_left': 1, 'axis_right': 2, 'units_per_rev': 0, 'loop': 'open', 'motor_current_ma': 0,
                 'max_ferr_rev': 1.0, 'grip_timeout': 2.0}


def load_grip_config(path):
    """Parameter aus gripper.yaml (Block gripper_node/ros__parameters) mit Standardwerten ergänzt."""
    cfg = dict(GRIP_DEFAULTS)
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    block = data.get('gripper_node', data.get('/**', {})).get('ros__parameters', {})
    cfg.update({k: v for k, v in block.items() if k in GRIP_DEFAULTS})
    return cfg


def load_config(path):
    """Parameter aus diff_drive.yaml (Block diff_drive_node/ros__parameters) mit Standardwerten ergänzt."""
    cfg = dict(DEFAULTS)
    if path:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        block = data.get('diff_drive_node', data.get('/**', {})).get('ros__parameters', {})
        cfg.update({k: v for k, v in block.items() if k in DEFAULTS})
    return cfg


def log(level, text):
    print(f'[ethercat_bus] [{level}] [{time.time():.3f}] {text}', flush=True)


def check_drive_params(bus, cfg, axes):
    """Gefährliche Treibereinstellungen ablehnen; gibt Einheiten pro Motorumdrehung je Achse zurück."""
    problems, upr_by_axis = [], {}
    closed = cfg['loop'] == 'closed'
    if cfg['loop'] not in ('open', 'closed'):
        raise Cl57eError("loop muss 'open' oder 'closed' sein")
    for axis in axes:
        run_mode = bus.read_param(axis, PARAM_RUN_MODE)
        if run_mode != (2 if closed else 1):
            problems.append(f'Achse {axis}: RunModeSet = {run_mode}, erwartet {2 if closed else 1} ({cfg["loop"]} loop)')
        if closed and bus.read_param(axis, PARAM_SENSOR_ERR) != 1:
            problems.append(f'Achse {axis}: SensorErr = 0 (Encoderfehler würden ignoriert)')
        current = bus.read_param(axis, PARAM_MAX_CURRENT)
        if cfg['motor_current_ma'] and current > cfg['motor_current_ma']:
            problems.append(f'Achse {axis}: MaxCurrent = {current} mA > motor_current_ma')
        upr = cfg['units_per_rev'] or bus.read_param(axis, PARAM_DIV)
        if not 200 <= upr <= 1_000_000:
            problems.append(f'Achse {axis}: unplausible Einheiten pro Umdrehung {upr}')
        upr_by_axis[axis] = upr
    if problems:
        raise Cl57eError('Treiberparameter: ' + '; '.join(problems))
    return upr_by_axis


def setup_main_thread(cfg):
    """Hauptthread vom Echtzeit-Kern fernhalten, GIL schneller abgeben (der Zyklus-Thread setzt sich selbst)."""
    sys.setswitchinterval(max(1e-4, cfg['gil_switch_ms'] / 1000.0))
    cpu = cfg['rt_cpu']
    if cpu >= 0:
        others = os.sched_getaffinity(0) - {cpu}
        if others:
            os.sched_setaffinity(0, others)


def main(argv=None):
    parser = argparse.ArgumentParser(description='EtherCAT-Busprozess (CL57E-4A) für diff_drive_node')
    parser.add_argument('--config', default='', help='diff_drive.yaml (Block diff_drive_node/ros__parameters)')
    parser.add_argument('--gripper-config', default='', help='gripper.yaml: Greifer-Achsen mit bedienen (Block gripper_node)')
    args, _ = parser.parse_known_args(argv)      # --ros-args von launch_ros ignorieren
    cfg = load_config(args.config)

    stop = {'flag': False}

    def on_signal(signum, _frame):
        stop['flag'] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    setup_main_thread(cfg)
    gears = {cfg['left_axis']: cfg['left_gear'], cfg['right_axis']: cfg['right_gear']}
    gcfg = load_grip_config(args.gripper_config) if args.gripper_config else None
    grip_axes = [gcfg['axis_left'], gcfg['axis_right']] if gcfg else []
    try:
        bus = Cl57eBus(cfg['ifname'], list(gears), cfg['cycle_ms'] / 1000.0, cfg['dc'],
                       cmd_timeout_s=min(0.3, cfg['cmd_vel_timeout']), rt_priority=cfg['rt_priority'],
                       rt_cpu=cfg['rt_cpu'], grip_nums=grip_axes)
    except ValueError as e:
        log('ERROR', f'FEHLER: {e} (Räder {list(gears)}, Greifer {grip_axes})')
        return 1
    server = None
    reason = 'beendet'
    code = 0
    try:
        name = bus.open()
        upr = check_drive_params(bus, cfg, list(gears))
        grip_upr = {}
        if gcfg:
            try:
                grip_upr = check_drive_params(bus, gcfg, grip_axes)
            except Cl57eError as e:      # Greifer-Fehler darf die Räder nicht verhindern
                log('ERROR', f'Greifer abgeschaltet: {e}')
                bus.gaxes.clear()
                grip_axes = []
        bus.configure()
        bus.start()
        for axis in grip_axes:
            bus.grip_set_limits(axis, amax=20000.0, ferr_limit=gcfg['max_ferr_rev'] * grip_upr[axis])
        for axis, gear in gears.items():
            bus.set_limits(axis, amax=cfg['wheel_acc'] / TWO_PI * gear * upr[axis],
                           ferr_limit=cfg['max_ferr_rev'] * upr[axis])
        info = {'name': name, 'ifname': cfg['ifname'], 'upr': {str(a): u for a, u in upr.items()},
                'rt_status': bus.rt_status, 'cycle_ms': cfg['cycle_ms'],
                'grip_upr': {str(a): u for a, u in grip_upr.items()}}
        server = BusServer(bus, cfg['bus_socket'], info, node_timeout=cfg['node_timeout'],
                           log=lambda text: log('INFO', text),
                           grip_timeout=gcfg['grip_timeout'] if gcfg else 1.0)
        log('INFO', f'{name} an {cfg["ifname"]} in OP (CSP, {cfg["loop"]} loop), Achsen '
            + ', '.join(f'{a} (1:{g:g}, {upr[a]} Einh./U)' for a, g in gears.items())
            + (f'; Greifer-Achsen {grip_axes}' if grip_axes else '')
            + f'; Zyklus {cfg["cycle_ms"]:g} ms: {bus.rt_status}. Warte auf diff_drive_node')
        gc.freeze()                    # Angelegtes aus der GC nehmen: kürzere GC-Pausen
        next_stats = time.monotonic() + cfg['stats_period'] if cfg['stats_period'] > 0 else math.inf
        while not stop['flag']:
            server.step(0.01)
            if bus.comm_lost:
                reason = f'EtherCAT-Kommunikation verloren ({bus.status()["thread_error"] or "keine Antwort"})'
                log('ERROR', reason)
                code = 1
                break
            if time.monotonic() >= next_stats:
                next_stats += cfg['stats_period']
                w = bus.window_stats()
                text = (f'EtherCAT letzte {cfg["stats_period"]:g} s: {w["cycles"]} Zyklen, {w["late"]} verspätet '
                        f'(> {1.5 * cfg["cycle_ms"]:g} ms), {w["overruns"]} Überläufe, {w["wkc_errors"]} WKC-Fehler, '
                        f'max. Periode {w["max_period"] * 1000:.1f} ms')
                log('WARN' if w['late'] or w['wkc_errors'] else 'INFO', text)
    except Cl57eError as e:
        reason = str(e)
        log('ERROR', f'FEHLER: {e}')
        code = 1
    finally:
        if server is not None:
            server.notify_exit(reason)
        bus.close()
        if server is not None:
            server.close()
        s = bus.stats
        log('INFO', f'Beendet. Zyklen {s["cycles"]}, Überläufe {s["overruns"]}, WKC-Fehler {s["wkc_errors"]}, '
            f'max. Periode {s["max_period"] * 1000:.1f} ms')
    return code


if __name__ == '__main__':
    sys.exit(main())
