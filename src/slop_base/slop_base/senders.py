"""Prüfung der Publisher auf /cmd_vel, ohne ROS (damit testbar).

Direkt nach dem Start kennt ROS von einem neuen Publisher oft erst den Teilnehmer, nicht den Node-Namen.
get_publishers_info_by_topic liefert dann '_NODE_NAME_UNKNOWN_' (meist ist das twist_mux selbst).
Ein solcher Eintrag sperrt das Fahren weiter (sicher), gilt aber erst nach `grace` s als Fehler.
"""

UNKNOWN_NODE = '_NODE_NAME_UNKNOWN_'


class SenderCheck:

    def __init__(self, allowed, grace=3.0):
        self.allowed = set(allowed)
        self.grace = grace
        self.unknown_since = None

    def check(self, names, now):
        """names: Node-Namen der Publisher, now: Zeit [s].

        Rückgabe (blockieren, fremde, wartet): blockieren = nicht fahren; fremde = Namen für die Fehlermeldung
        (inkl. UNKNOWN_NODE erst nach grace s); wartet = nur unbekannte Namen, noch in der Schonfrist.
        """
        foreign = sorted({n for n in names if n not in self.allowed and n != UNKNOWN_NODE})
        unknown = UNKNOWN_NODE in names
        if not unknown:
            self.unknown_since = None
            return bool(foreign), foreign, False
        if self.unknown_since is None:
            self.unknown_since = now
        if now - self.unknown_since >= self.grace:
            return True, foreign + [UNKNOWN_NODE], False
        return True, foreign, not foreign
