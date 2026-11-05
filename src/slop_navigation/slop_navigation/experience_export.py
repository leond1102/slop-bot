#!/usr/bin/env python3
"""Karte mit der Erfahrung aktualisieren: neue Karte aus <karte>.yaml/.pgm + <karte>.experience.npz.

    ros2 run slop_navigation experience_export ~/slop-bot_ws/maps/office-3.yaml                 # → office-3-neu.yaml
    ros2 run slop_navigation experience_export ~/slop-bot_ws/maps/office-3.yaml -o office-4     # eigener Name
    ros2 run slop_navigation experience_export ~/slop-bot_ws/maps/office-3.yaml --nur-zeigen    # nur zählen

Oft gesehene feste Zellen werden Wand, oft gesehene vielleicht/freie Zellen werden frei (Stühle gehören nicht in die
Karte, um die kümmern sich die weichen Kosten). Alles andere bleibt wie im Original. Die Originaldateien bleiben
unverändert; Erfahrung und letzte Pose werden für die neue Karte mitkopiert, damit das Lernen dort weitergeht.
Nur für Nav2/AMCL: slam_toolbox-Dateien (.posegraph/.data) entstehen dabei nicht.
"""
import argparse
import os
import shutil
import sys

import yaml

from slop_navigation.experience_core import ExperienceGrid, read_pgm, updated_map_image, write_pgm


def main(argv=None):
    ap = argparse.ArgumentParser(description='Karte mit der Erfahrungskarte aktualisieren (neue Dateien)')
    ap.add_argument('map', help='Karte (.yaml von map_saver_cli)')
    ap.add_argument('-o', '--out', help='Name der neuen Karte ohne Endung (Standard: <karte>-neu), im selben Ordner')
    ap.add_argument('--min-seen', type=float, default=10.0,
                    help='so oft muss eine Zelle gesehen sein, damit sie geändert wird (Standard 10)')
    ap.add_argument('--p-free', type=float, default=0.15)
    ap.add_argument('--p-fixed', type=float, default=0.80)
    ap.add_argument('--nur-zeigen', action='store_true', help='nur zählen, nichts schreiben')
    a = ap.parse_args(argv)

    map_yaml = os.path.abspath(os.path.expanduser(a.map))
    base, folder = os.path.splitext(map_yaml)[0], os.path.dirname(map_yaml)
    exp_path = base + '.experience.npz'
    if not os.path.exists(exp_path):
        sys.exit(f'Keine Erfahrung gefunden: {exp_path}')
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    img_path = meta['image'] if os.path.isabs(meta['image']) else os.path.join(folder, meta['image'])
    img = read_pgm(img_path)
    grid = ExperienceGrid.load(exp_path)
    if not grid.same_grid(img.shape[1], img.shape[0], meta['resolution'], meta['origin'][0], meta['origin'][1]):
        sys.exit('Erfahrung passt nicht zur Karte (Größe, Auflösung oder Ursprung verschieden)')

    new, n_occ, n_free = updated_map_image(img, grid, a.min_seen, a.p_free, a.p_fixed)
    res2 = grid.resolution ** 2
    print(f'Änderungen gegenüber {os.path.basename(map_yaml)} (Zellen mit ≥ {a.min_seen:g} Beobachtungen):')
    print(f'  neu belegt: {n_occ:6d} Zellen ({n_occ * res2:.2f} m²)')
    print(f'  neu frei:   {n_free:6d} Zellen ({n_free * res2:.2f} m²)')
    if a.nur_zeigen:
        return

    out = os.path.join(folder, a.out or os.path.basename(base) + '-neu')
    if os.path.exists(out + '.yaml'):
        sys.exit(f'{out}.yaml gibt es schon – anderen Namen mit -o wählen')
    write_pgm(out + '.pgm', new)
    meta_out = dict(meta, image=os.path.basename(out) + '.pgm')
    with open(out + '.yaml', 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta_out, f, default_flow_style=None, sort_keys=False)
    shutil.copy2(exp_path, out + '.experience.npz')
    if os.path.exists(base + '.last_pose.yaml'):
        shutil.copy2(base + '.last_pose.yaml', out + '.last_pose.yaml')
    print(f'Geschrieben: {out}.yaml / .pgm (+ Erfahrung und letzte Pose kopiert)')
    print(f'Benutzen:    ros2 launch slop_navigation navigation.launch.py map:={out}.yaml')


if __name__ == '__main__':
    main()
