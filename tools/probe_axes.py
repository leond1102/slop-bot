#!/usr/bin/env python3
"""
@file probe_axes.py
@brief Nur lesen: welche CiA-402-Objekte und PDOs des CL57E-4A sind für jede Achse 1..4 vorhanden?

Keine Bewegung, kein Schreiben. Zeigt, ob z.B. 0x6041 (Achse 1) / 0x6841 (Achse 2) / 0x7041 (Achse 3) ... lesbar sind
und welche PDOs (0x1600.., 0x1A00..) und Zuordnungen (0x1C12/0x1C13) der Treiber kennt.

  sudo ~/ethercat_venv/bin/python3 ~/slop-bot_ws/tools/probe_axes.py [--ifname enP8p1s0]
"""
import argparse
import struct
import sys

import pysoem

OBJ = [(0x6040, 'Controlword', '<H'), (0x6041, 'Statusword', '<H'), (0x6060, 'Mode of op', '<b'),
       (0x6061, 'Mode display', '<b'), (0x6064, 'Position', '<i'), (0x607A, 'Target pos', '<i')]


def rd(slave, index, sub, fmt):
    try:
        data = bytes(slave.sdo_read(index, sub))
        size = struct.calcsize(fmt)
        return struct.unpack(fmt, (data + b'\0' * size)[:size])[0]
    except Exception as e:
        return 'FEHLT' if 'does not exist' in str(e) else f'Fehler: {e}'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ifname', default='enP8p1s0')
    a = p.parse_args()
    m = pysoem.Master()
    m.open(a.ifname)
    try:
        if m.config_init() <= 0:
            print('Kein Slave gefunden')
            return 1
        s = m.slaves[0]
        m.read_state()
        print(f'{s.name}  Product 0x{s.id:08X}  AL-Zustand 0x{s.state:02X}')
        for n in range(1, 5):
            off = (n - 1) * 0x800
            print(f'\n--- Achse {n} (Offset 0x{off:04X}) ---')
            for idx, name, fmt in OBJ:
                v = rd(s, idx + off, 0, fmt)
                shown = f'0x{v:X}' if isinstance(v, int) and fmt == '<H' else v
                print(f'  0x{idx + off:04X} {name:13s} = {shown}')
            for base, name in ((0x1600, 'RxPDO'), (0x1A00, 'TxPDO')):
                pdo = base + (n - 1) * 0x10
                print(f'  0x{pdo:04X} {name} Einträge = {rd(s, pdo, 0, "<B")}')
        for assign in (0x1C12, 0x1C13):
            cnt = rd(s, assign, 0, '<B')
            items = [f'0x{rd(s, assign, i, "<H"):04X}' for i in range(1, cnt + 1)] if isinstance(cnt, int) else []
            print(f'\n0x{assign:04X} Zuordnung: {cnt} {items}')
        return 0
    finally:
        m.close()


if __name__ == '__main__':
    sys.exit(main())
