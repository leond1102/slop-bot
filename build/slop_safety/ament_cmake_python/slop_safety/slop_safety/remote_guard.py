"""Erkennt eine aktive RustDesk-Fernverbindung, ohne ROS (damit testbar).

Eine Sitzung gilt als aktiv, wenn
  1. der Verbindungsmanager läuft ("rustdesk --cm", startet RustDesk nur für eingehende Sitzungen), und
  2. eine TCP-Verbindung der Sitzung besteht (lokal Port 21118 = direkt, oder entfernt Port 21117 = Relay)
     und vom Client in den letzten max_silence_s Sekunden Daten oder ACKs kamen.

Punkt 2 macht die Erkennung schnell: Bricht das Netz weg, bleibt die TCP-Verbindung oft noch lange
"ESTABLISHED", aber lastrcv/lastack (aus "ss -ti") wachsen sofort. Der Client meldet sich im Betrieb
etwa jede Sekunde (gemessen 03.11.: lastrcv < 0,7 s).
Achtung: ss lässt lastrcv/lastack weg, wenn der Wert 0 ist (gerade eben Daten gekommen). Ein fehlendes
Feld in einer vorhandenen Infozeile heißt also 0 ms, nicht "still".

Nicht erkannt werden Sitzungen über UDP. Die führen zur Sperre (sicher, aber lästig).

LockDelay sperrt erst, wenn die Sitzung lock_delay_s lang ununterbrochen fehlt. Kurze Aussetzer
lösen so keine Sperre aus.
"""
import os
import re
import subprocess
from dataclasses import dataclass

_FIELD = re.compile(r'\b(lastrcv|lastack):(\d+)')


@dataclass
class Connection:
    local: str
    peer: str
    silence_ms: float      # Zeit seit den letzten Daten/ACKs vom Client


def cm_running(proc='/proc'):
    """True, wenn ein Prozess "rustdesk ... --cm" läuft."""
    for pid in os.listdir(proc):
        if not pid.isdigit():
            continue
        try:
            with open(os.path.join(proc, pid, 'cmdline'), 'rb') as f:
                args = f.read().split(b'\0')
        except OSError:
            continue        # Prozess inzwischen beendet oder kein Zugriff
        if args and b'rustdesk' in os.path.basename(args[0]) and b'--cm' in args[1:]:
            return True
    return False


def parse_ss(text):
    """Ausgabe von "ss -tniH state established ..." → Liste von Connection."""
    conns = []
    for line in text.splitlines():
        if not line.strip():
            continue
        if not line[0].isspace():            # Kopfzeile einer Verbindung: Recv-Q Send-Q Local Peer
            fields = line.split()
            if len(fields) >= 4:
                conns.append(Connection(fields[2], fields[3], float('inf')))
        elif conns:                            # eingerückte Infozeile zur letzten Verbindung
            found = dict(_FIELD.findall(line))
            # ss gibt lastrcv/lastack nur aus, wenn > 0. Fehlt eins, kam gerade eben etwas → 0 ms
            values = [float(found.get(name, 0)) for name in ('lastrcv', 'lastack')]
            conns[-1].silence_ms = min(conns[-1].silence_ms, min(values))
    return conns


def connections(direct_port=21118, relay_port=21117, timeout_s=1.0):
    """Bestehende TCP-Verbindungen einer RustDesk-Sitzung."""
    flt = f'( sport = :{direct_port} or dport = :{relay_port} )'
    out = subprocess.run(['ss', '-tniH', 'state', 'established', flt], capture_output=True, text=True,
                         timeout=timeout_s, check=True).stdout
    return parse_ss(out)


def session_state(direct_port=21118, relay_port=21117, max_silence_s=3.0, require_cm=True):
    """(aktiv, Text). Bei jedem Fehler: nicht aktiv."""
    cm = cm_running()
    conns = connections(direct_port, relay_port)
    alive = [c for c in conns if c.silence_ms <= max_silence_s * 1000.0]
    if require_cm and not cm:
        return False, 'kein RustDesk-Verbindungsmanager (--cm)'
    if not conns:
        return False, 'keine RustDesk-Verbindung'
    if not alive:
        quiet = min(c.silence_ms for c in conns) / 1000.0
        return False, f'RustDesk-Verbindung seit {quiet:.1f} s still (Netz weg?)'
    return True, 'RustDesk-Sitzung von ' + ', '.join(c.peer for c in alive)


class LockDelay:
    """Fahrfreigabe mit Verzögerung beim Sperren. Zeiten in Sekunden, monotone Uhr.

    Startet gesperrt. Ist die Sitzung da, wird sofort freigegeben. Fehlt sie danach, bleibt die
    Freigabe noch lock_delay_s bestehen und fällt erst weg, wenn die Sitzung so lange
    ununterbrochen fehlt. Kommt sie vorher zurück, beginnt die Wartezeit beim nächsten Fehlen neu.
    """

    def __init__(self, lock_delay_s=3.0):
        self.lock_delay_s = lock_delay_s
        self.allowed = False
        self.lost_since = None        # Zeitpunkt, seit dem die Sitzung fehlt (nur während Freigabe)

    def update(self, active, now):
        """Neues Prüfergebnis → True = Fahren erlaubt."""
        if active:
            self.allowed, self.lost_since = True, None
        elif self.allowed:
            if self.lost_since is None:
                self.lost_since = now
            if now - self.lost_since >= self.lock_delay_s:
                self.allowed, self.lost_since = False, None
        return self.allowed

    def remaining(self, now):
        """s bis zur Sperre, None wenn gerade nicht gewartet wird."""
        if self.lost_since is None:
            return None
        return max(0.0, self.lock_delay_s - (now - self.lost_since))
