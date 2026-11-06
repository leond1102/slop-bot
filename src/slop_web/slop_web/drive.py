"""Fahren aus dem Browser mit Totmann – ohne ROS, damit testbar.

Der Browser sendet, solange eine Taste gehalten wird, alle ~0,1 s (v, w) als Anteil -1…1. Kommt deadman s lang nichts
(Taste losgelassen, Funkloch, Handy gesperrt, Tab zu), werden stop_hold s lang Nullbefehle gesendet, danach nichts mehr.
Dann läuft der teleop-Eingang in twist_mux ab (0,5 s) und die Navigation darf wieder fahren – wie bei keyboard_teleop.
"""
from dataclasses import dataclass


def clamp(x, lo=-1.0, hi=1.0):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return 0.0
    if x != x:                     # NaN
        return 0.0
    return max(lo, min(hi, x))


@dataclass
class DriveLimits:
    max_linear: float = 0.18       # m/s bei 100 %
    max_angular: float = 0.4       # rad/s bei 100 %
    deadman: float = 0.3           # s ohne neuen Befehl → Stopp
    stop_hold: float = 0.3         # s Nullbefehle danach


class Drive:
    def __init__(self, limits=None):
        self.limits = limits or DriveLimits()
        self.v = self.w = 0.0
        self.last = None           # Zeit des letzten Befehls
        self.idle_since = None     # Zeit, ab der Nullbefehle laufen

    def command(self, v, w, scale, now):
        """Befehl vom Browser: v, w als Anteil -1…1, scale 0,1…1 (Tempo-Regler)."""
        s = clamp(scale, 0.1, 1.0)
        self.v = clamp(v) * s * self.limits.max_linear
        self.w = clamp(w) * s * self.limits.max_angular
        self.last = now
        if self.v == 0.0 and self.w == 0.0:
            self._idle(now)
        else:
            self.idle_since = None

    def halt(self, now):
        """Sofort anhalten (Taste los, Verbindung weg, Notaus)."""
        self.v = self.w = 0.0
        if self.last is not None:
            self._idle(now)

    def _idle(self, now):
        if self.idle_since is None:
            self.idle_since = now

    def output(self, now):
        """(v, w) zum Senden oder None (nichts senden)."""
        if self.last is None:
            return None
        if self.idle_since is None and now - self.last > self.limits.deadman:
            self.v = self.w = 0.0
            self.idle_since = self.last + self.limits.deadman
        if self.idle_since is not None:
            if now - self.idle_since <= self.limits.stop_hold:
                return 0.0, 0.0
            self.last = self.idle_since = None
            return None
        return self.v, self.w

    @property
    def active(self):
        return self.last is not None and self.idle_since is None
