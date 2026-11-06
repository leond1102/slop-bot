"""Sensor-Wächter ohne ROS (damit testbar): Liefern alle Quellen der Collision Monitore frische Daten?

Der Collision Monitor von Nav2 (Humble) ignoriert eine Quelle, deren letzte Daten älter als source_timeout (1 s)
sind oder für die das TF fehlt, und stoppt dabei NICHT. Fällt das Lidar aus, führe Nav2 ohne Lidar-Schutz weiter.
Der Wächter sperrt Nav2 vorher über twist_mux (slop_safety/sensor_guard_node.py).

Eine Quelle ist in Ordnung, wenn
  1. schon Daten kamen,
  2. die letzte Nachricht vor höchstens max_age s ankam (monotone Uhr, unempfindlich gegen Uhrsprünge),
  3. ihr Zeitstempel höchstens max_age s alt ist (ROS-Zeit, so prüft es auch der Collision Monitor) und
  4. der Zeitstempel nicht mehr als max_future s in der Zukunft liegt (die TF-Abfrage dafür schlüge fehl).

Sperren sofort, freigeben erst, wenn alle Quellen release_after s lang ununterbrochen in Ordnung waren.
Startet gesperrt.
"""


class Source:
    """Eine überwachte Quelle. Zeiten in Sekunden."""

    def __init__(self, label, max_age, max_future=0.2):
        self.label = label              # Anzeigename (Topic)
        self.max_age = max_age
        self.max_future = max_future
        self.stamp = None               # ROS-Zeit aus dem Header der letzten Nachricht
        self.received = None            # monotone Zeit des Empfangs
        self.worst_age = 0.0            # höchstes Alter seit reset_stats() (zum Einstellen von max_age)

    def update(self, stamp, received):
        self.stamp, self.received = stamp, received

    def age(self, now_ros):
        """Alter des Zeitstempels (negativ = Zukunft), None ohne Daten."""
        return None if self.stamp is None else now_ros - self.stamp

    def problem(self, now_ros, now_mono):
        """None = in Ordnung, sonst ein kurzer Text."""
        if self.received is None:
            return 'noch keine Daten'
        silent = now_mono - self.received
        if silent > self.max_age:
            return f'seit {silent:.1f} s still'
        if self.stamp <= 0.0:
            return 'ohne Zeitstempel'
        age = now_ros - self.stamp
        self.worst_age = max(self.worst_age, age)
        if age > self.max_age:
            return f'Daten {age:.2f} s alt'
        if age < -self.max_future:
            return f'Zeitstempel {-age:.2f} s in der Zukunft'
        return None

    def reset_stats(self):
        self.worst_age = 0.0


class Guard:
    """Sperrentscheidung über alle Quellen."""

    def __init__(self, sources, release_after=1.0):
        self.sources = sources          # Liste von Source
        self.release_after = release_after
        self.locked = True
        self.ok_since = None            # monotone Zeit, seit der alle Quellen in Ordnung sind
        self.problems = {}              # label → Text, Stand der letzten Prüfung

    def update(self, now_ros, now_mono):
        """Alle Quellen prüfen → True = gesperrt."""
        self.problems = {}
        for s in self.sources:
            p = s.problem(now_ros, now_mono)
            if p is not None:
                self.problems[s.label] = p
        if self.problems:
            self.locked, self.ok_since = True, None
        else:
            if self.ok_since is None:
                self.ok_since = now_mono
            if self.locked and now_mono - self.ok_since >= self.release_after:
                self.locked = False
        return self.locked

    def remaining(self, now_mono):
        """s bis zur Freigabe, wenn alles in Ordnung ist und nur noch gewartet wird, sonst None."""
        if not self.locked or self.ok_since is None:
            return None
        return max(0.0, self.release_after - (now_mono - self.ok_since))

    def status(self, now_mono):
        """Ein Satz für Log und Anzeige."""
        if not self.locked:
            return 'frei'
        if self.problems:
            return 'gesperrt: ' + ', '.join(f'{k} {v}' for k, v in self.problems.items())
        return f'gesperrt: alle Quellen frisch, Freigabe in {self.remaining(now_mono):.1f} s'
