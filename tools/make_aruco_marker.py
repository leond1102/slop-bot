#!/usr/bin/python3
"""ArUco-Marker zum Ausdrucken erzeugen (PNG mit 300 dpi, schwarzes Quadrat genau marker_cm groß, weißer Rand).

    python3 tools/make_aruco_marker.py                       # ID 1, ARUCO_ORIGINAL, 10 cm → marker_ARUCO_ORIGINAL_id1.png
    python3 tools/make_aruco_marker.py --id 2 --dict 4X4_50 --cm 12 -o dock2.png
Beim Drucken auf 100 % Größe stellen (nicht „an Seite anpassen“) und die Kantenlänge des schwarzen Quadrats nachmessen:
Sie muss zu marker_size in slop_dock/config/dock.yaml passen. Matt drucken (Glanz spiegelt), flach und senkrecht aufhängen.
"""
import argparse

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--id', type=int, default=1)
    ap.add_argument('--dict', default='ARUCO_ORIGINAL')
    ap.add_argument('--cm', type=float, default=10.0, help='Kantenlänge des schwarzen Quadrats in cm')
    ap.add_argument('--border-cm', type=float, default=2.5, help='weißer Rand in cm')
    ap.add_argument('-o', '--out')
    a = ap.parse_args()
    dpi = 300.0
    px = round(a.cm / 2.54 * dpi)
    border = round(a.border_cm / 2.54 * dpi)
    const = getattr(cv2.aruco, 'DICT_' + a.dict.upper())
    d = cv2.aruco.getPredefinedDictionary(const)
    img = cv2.aruco.generateImageMarker(d, a.id, px) if hasattr(cv2.aruco, 'generateImageMarker') \
        else cv2.aruco.drawMarker(d, a.id, px)
    img = cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    out = a.out or f'marker_{a.dict}_id{a.id}.png'
    # PNG mit Auflösung (pHYs), damit Druckprogramme die Größe richtig übernehmen
    cv2.imwrite(out, img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    total = img.shape[0] / dpi * 2.54
    print(f'{out}: ID {a.id}, {a.dict}, schwarzes Quadrat {a.cm:g} cm, gesamt {total:.1f} × {total:.1f} cm bei {dpi:.0f} dpi ({img.shape[0]} px)')


if __name__ == '__main__':
    main()
