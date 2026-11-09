"""Was die Seite starten kann (config/web.yaml) – ohne ROS, damit testbar.

Befehle sind Listen (kein Shell-Text). Platzhalter $map, $name, $experience_costs, $safety, $maps_dir, $y, $width werden nur
mit geprüften Werten ersetzt: map muss eine vorhandene Karte in maps_dir sein, name nur Buchstaben/Ziffern/_/-, experience_costs
true/false, safety eine der SAFETY_LEVELS, y/width Zahlen in mm aus GRIP_RANGES (eingesetzt in Metern, für den Greifer).
So kann vom Browser aus nichts anderes ausgeführt werden als hier steht.
"""
import glob
import os
import re
from string import Template

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')
SAFETY_LEVELS = ('streng', 'locker')            # navigation.launch.py safety:=…, die erste ist der Standard
SAFETY_RE = re.compile(r'(?:^|\s)safety:=(\S+)')
GRIP_RANGES = {'y': (-200.0, 200.0), 'width': (1.0, 250.0)}   # mm; die genauen Grenzen prüft gripper_node


def load(path):
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    cfg['maps_dir'] = os.path.expanduser(cfg.get('maps_dir', '~/slop-bot_ws/maps'))
    ids = set()
    for kind in ('items', 'sequences', 'actions'):
        for e in cfg.get(kind, []):
            if e['id'] in ids:
                raise ValueError(f'doppelte id {e["id"]}')
            ids.add(e['id'])
    items = {i['id'] for i in cfg.get('items', [])}
    for s in cfg.get('sequences', []):
        for step in s['steps']:
            if step not in items:
                raise ValueError(f'{s["id"]}: unbekannter Schritt {step}')
    return cfg


def maps(maps_dir):
    """Karten (.yaml von map_saver), ohne Hilfsdateien wie .last_pose.yaml."""
    out = []
    for p in sorted(glob.glob(os.path.join(maps_dir, '*.yaml'))):
        name = os.path.basename(p)[:-5]
        if '.' not in name:
            out.append(name)
    return out


def params(cfg, raw):
    """Browser-Werte prüfen → Werte für die Platzhalter. Fehler → ValueError mit verständlichem Text."""
    raw = raw or {}
    out = {'maps_dir': cfg['maps_dir']}
    m = raw.get('map') or cfg.get('default_map')
    if m is not None:
        if m not in maps(cfg['maps_dir']):
            raise ValueError(f'Karte {m!r} gibt es nicht in {cfg["maps_dir"]}')
        out['map'] = os.path.join(cfg['maps_dir'], m + '.yaml')
    out['experience_costs'] = 'true' if raw.get('experience_costs') in (True, 'true', 1) else 'false'
    safety = raw.get('safety') or SAFETY_LEVELS[0]
    if safety not in SAFETY_LEVELS:
        raise ValueError(f'Sicherheitsstufe {safety!r} gibt es nicht ({" oder ".join(SAFETY_LEVELS)})')
    out['safety'] = safety
    if 'name' in raw:
        if not isinstance(raw['name'], str) or not NAME_RE.match(raw['name']):
            raise ValueError('Name nur aus Buchstaben, Ziffern, _ und - (höchstens 40 Zeichen)')
        base = os.path.join(cfg['maps_dir'], raw['name'])
        if any(os.path.exists(base + ext) for ext in ('.yaml', '.pgm', '.posegraph', '.data')):
            raise ValueError(f'Karte „{raw["name"]}“ gibt es schon – neuen Namen wählen (map_saver überschreibt ohne '
                             'Rückfrage)')
        out['name'] = raw['name']
    for key, (lo, hi) in GRIP_RANGES.items():          # Greifer: Eingabe in mm, Befehl in m
        if key in raw:
            try:
                mm = float(raw[key])
            except (TypeError, ValueError):
                raise ValueError(f'{key}: Zahl in mm angeben') from None
            if not lo <= mm <= hi:                     # schließt auch nan/inf aus
                raise ValueError(f'{key}: {lo:g} … {hi:g} mm')
            out[key] = f'{mm / 1000.0:.4f}'
    return out


def safety_of(cmdline):
    """Sicherheitsstufe aus einer Befehlszeile (navigation.launch.py … safety:=locker): ohne Angabe der Standard, bei
    mehreren die letzte (wie ros2 launch), unbekannter Wert → '?'."""
    found = SAFETY_RE.findall(cmdline or '')
    if not found:
        return SAFETY_LEVELS[0]
    return found[-1] if found[-1] in SAFETY_LEVELS else '?'


TRIGGER_RE = re.compile(r"success=(True|False), message=(['\"])(.*?)\2\)\s*$", re.S)


def trigger_reply(out):
    """Ausgabe von `ros2 service call … std_srvs/srv/Trigger` → (success, message) oder None, wenn es keine ist."""
    m = TRIGGER_RE.search(out or '')
    return (m.group(1) == 'True', m.group(3)) if m else None


def argv(cmd, values):
    """Platzhalter in einer Befehlsliste ersetzen; fehlt ein Wert, ValueError."""
    try:
        return [Template(str(a)).substitute(values) for a in cmd]
    except KeyError as e:
        raise ValueError(f'Angabe fehlt: {e.args[0]}') from None


# Alles, was bestimmt, welche Nodes sich gegenseitig sehen. Webserver und gestartete Programme müssen hier übereinstimmen.
DDS_KEYS = ('ROS_LOCALHOST_ONLY', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI', 'ROS_AUTOMATIC_DISCOVERY_RANGE')


def apply_dds(cfg_dds, environ):
    """DDS-Einstellungen aus web.yaml (dds:) auf die Umgebung anwenden: Wert = setzen, null = entfernen; nicht
    genannte Schlüssel bleiben wie geerbt. Gibt (wirksame Einstellungen, Liste der Änderungen) zurück.
    Grund: Der Webserver erbte sonst die Einstellungen des Terminals, aus dem er gestartet wurde (z. B. alte
    Testeinstellungen) – dann sahen sich Webseite und gestartete Programme nicht."""
    changes = []
    for key, value in (cfg_dds or {}).items():
        old = environ.get(key)
        if value is None:
            if old is not None:
                del environ[key]
                changes.append(f'{key} entfernt (war {old})')
        elif old != str(value):
            environ[key] = str(value)
            changes.append(f'{key}={value} (war {old if old is not None else "nicht gesetzt"})')
    return {k: environ[k] for k in DDS_KEYS if k in environ}, changes
