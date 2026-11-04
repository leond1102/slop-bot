"""Speicher der Kalibrierergebnisse, ohne ROS-Knoten (aber mit ament_index für den Pfad).

Die Ergebnisse liegen als YAML im Ordner calib/ des Workspace (nicht in install/, damit Bauen sie nicht löscht).
Pfad: Umgebungsvariable SLOP_CALIB_DIR, sonst <Workspace>/calib, sonst ~/slop-bot_ws/calib.
Jede Datei ist ein Dictionary je Sensor, z. B. gyro.yaml: {livox: {bias: [...], stamp: ...}, camera: {...}}.
"""
import datetime
import os

import yaml


def calib_dir():
    path = os.environ.get('SLOP_CALIB_DIR')
    if path:
        return os.path.expanduser(path)
    try:
        from ament_index_python.packages import get_package_prefix
        # <ws>/install/slop_calibration → <ws>/calib
        prefix = get_package_prefix('slop_calibration')
        return os.path.join(os.path.dirname(os.path.dirname(prefix)), 'calib')
    except Exception:
        return os.path.expanduser('~/slop-bot_ws/calib')


def _path(name):
    return os.path.join(calib_dir(), f'{name}.yaml')


def load(name):
    """Inhalt von calib/<name>.yaml oder None, wenn die Datei fehlt oder unlesbar ist."""
    try:
        with open(_path(name), encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError):
        return None
    return data if isinstance(data, dict) else None


def load_entry(name, sensor):
    """Eintrag eines Sensors aus calib/<name>.yaml oder None."""
    data = load(name)
    entry = data.get(sensor) if data else None
    return entry if isinstance(entry, dict) else None


def save_entry(name, sensor, entry):
    """Eintrag eines Sensors setzen (andere Sensoren in der Datei bleiben), Zeitstempel ergänzen. Gibt den Pfad zurück."""
    data = load(name) or {}
    data[sensor] = dict(entry, stamp=datetime.datetime.now().replace(microsecond=0).isoformat())
    os.makedirs(calib_dir(), exist_ok=True)
    path = _path(name)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
    os.replace(tmp, path)        # atomar: nie eine halb geschriebene Datei
    return path


def age_days(entry):
    """Alter eines Eintrags in Tagen oder None."""
    try:
        return (datetime.datetime.now() - datetime.datetime.fromisoformat(entry['stamp'])).total_seconds() / 86400.0
    except (KeyError, TypeError, ValueError):
        return None
