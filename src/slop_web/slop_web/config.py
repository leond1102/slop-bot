"""Was die Seite starten kann (config/web.yaml) – ohne ROS, damit testbar.

Befehle sind Listen (kein Shell-Text). Platzhalter $map, $name, $experience_costs, $maps_dir werden nur mit geprüften
Werten ersetzt: map muss eine vorhandene Karte in maps_dir sein, name nur Buchstaben/Ziffern/_/-, experience_costs
true/false. So kann vom Browser aus nichts anderes ausgeführt werden als hier steht.
"""
import glob
import os
import re
from string import Template

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


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
    if 'name' in raw:
        if not isinstance(raw['name'], str) or not NAME_RE.match(raw['name']):
            raise ValueError('Name nur aus Buchstaben, Ziffern, _ und - (höchstens 40 Zeichen)')
        base = os.path.join(cfg['maps_dir'], raw['name'])
        if any(os.path.exists(base + ext) for ext in ('.yaml', '.pgm', '.posegraph', '.data')):
            raise ValueError(f'Karte „{raw["name"]}“ gibt es schon – neuen Namen wählen (map_saver überschreibt ohne '
                             'Rückfrage)')
        out['name'] = raw['name']
    return out


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
