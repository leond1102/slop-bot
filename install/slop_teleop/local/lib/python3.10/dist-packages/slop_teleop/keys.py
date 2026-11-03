"""Tastatur → Fahrbefehl, ohne ROS (damit testbar).

Ein Terminal meldet nur gedrückte Tasten, kein Loslassen. Hält man eine Taste, wiederholt das System
sie erst nach einer Verzögerung (meist 0,25–0,66 s), danach etwa alle 30–40 ms. Deshalb gilt eine
Taste als losgelassen (Totmann), wenn
  - nach dem ersten Druck first_timeout s lang keine Wiederholung kam, oder
  - nach einer Wiederholung repeat_timeout s lang keine weitere kam.
So fährt der Roboter beim Halten ohne Ruckeln und steht kurz nach dem Loslassen.
"""

# Taste → (vor/zurück, Drehung links/rechts), jeweils -1, 0 oder 1
MOVES = {
    'w': (1, 0), 'up': (1, 0),
    's': (-1, 0), 'down': (-1, 0),
    'a': (0, 1), 'left': (0, 1),
    'd': (0, -1), 'right': (0, -1),
    'q': (1, 1),        # Kurve vorwärts links
    'e': (1, -1),       # Kurve vorwärts rechts
}
STOP = {' ', 'x'}
FASTER = {'+'}
SLOWER = {'-'}

_ARROWS = {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}

HELP = """\
Slop-Bot Tastatursteuerung  (Taste HALTEN zum Fahren, loslassen = Stopp)

        q   w   e             ↑
        a   s   d         ←   ↓   →

  w / ↑   vorwärts           q   Kurve vorwärts links
  s / ↓   rückwärts          e   Kurve vorwärts rechts
  a / ←   links drehen       Leertaste / x   Stopp
  d / →   rechts drehen      + / -   schneller / langsamer
  Strg+C  beenden
"""


def parse_keys(data):
    """Bytes aus dem Terminal → Liste von Tastennamen ('w', 'up', ' ', ...).

    Pfeiltasten kommen als Escape-Folge (ESC [ A oder ESC O A, mit Modifikator ESC [ 1 ; 5 A).
    Andere Escape-Folgen (F-Tasten, Pos1, ...) werden komplett verworfen, damit ihre Zeichen nicht
    als Fahrtasten zählen. Buchstaben werden klein gemacht (Feststelltaste egal).
    """
    keys, i, n = [], 0, len(data)
    while i < n:
        b = data[i]
        if b == 0x1b:
            j = i + 1
            if j < n and data[j] in b'[O':
                j += 1
                while j < n and not 0x40 <= data[j] <= 0x7e:    # Parameter bis zum Endzeichen
                    j += 1
                if j < n and chr(data[j]) in _ARROWS:
                    keys.append(_ARROWS[chr(data[j])])
                j += 1
            i = j
            continue
        keys.append(chr(b).lower() if b < 0x80 else '')
        i += 1
    return [k for k in keys if k]


class KeyTeleop:
    """Zustand der Tastatursteuerung. Zeiten in Sekunden, beliebige monotone Uhr."""

    def __init__(self, first_timeout=0.7, repeat_timeout=0.25, scale=1.0, scale_step=0.1):
        self.first_timeout = first_timeout
        self.repeat_timeout = repeat_timeout
        self.scale = scale
        self.scale_step = scale_step
        self.release()

    def release(self):
        self.key = None
        self.move = (0, 0)
        self.last = None
        self.repeating = False

    def active(self, now):
        if self.key is None:
            return False
        timeout = self.repeat_timeout if self.repeating else self.first_timeout
        return now - self.last <= timeout

    def press(self, key, now):
        """Eine Taste verarbeiten. False bei unbekannter Taste."""
        if key in MOVES:
            if key == self.key and self.active(now):
                self.repeating = True            # Taste wird gehalten
            else:
                self.key, self.move, self.repeating = key, MOVES[key], False
            self.last = now
        elif key in STOP:
            self.release()
        elif key in FASTER:
            self.scale = min(1.0, round(self.scale + self.scale_step, 3))
        elif key in SLOWER:
            self.scale = max(self.scale_step, round(self.scale - self.scale_step, 3))
        else:
            return False
        return True

    def command(self, now):
        """(linear, angular) als Anteil der Höchstgeschwindigkeit, je -1…1. (0, 0) = Stopp."""
        if not self.active(now):
            self.release()
            return 0.0, 0.0
        return self.move[0] * self.scale, self.move[1] * self.scale
