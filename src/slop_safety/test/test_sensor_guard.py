"""Tests für den Sensor-Wächter (ohne ROS). Zeiten in Sekunden; ROS-Zeit und monotone Uhr laufen hier gleich."""
from slop_safety.sensor_guard import Guard, Source

T0 = 1000.0


def make(max_age=0.4, release_after=1.0):
    a, b = Source('/scan', max_age), Source('/camera/obstacles', max_age)
    return a, b, Guard([a, b], release_after)


def feed(sources, t, age=0.05):
    """Alle Quellen bekommen zur Zeit t eine Nachricht, deren Zeitstempel age s alt ist."""
    for s in sources:
        s.update(t - age, t)


def test_starts_locked_without_data():
    a, b, g = make()
    assert g.update(T0, T0)
    assert g.problems == {'/scan': 'noch keine Daten', '/camera/obstacles': 'noch keine Daten'}
    assert g.status(T0).startswith('gesperrt: /scan noch keine Daten')


def test_release_only_after_release_after():
    a, b, g = make()
    for k in range(10):                       # 0 … 0,9 s lang frisch: gesperrt
        t = T0 + k * 0.1
        feed([a, b], t)
        assert g.update(t, t)
    assert g.remaining(t) is not None
    assert 'Freigabe in 0.1 s' in g.status(t)
    t = T0 + 1.0                              # 1,0 s → frei
    feed([a, b], t)
    assert not g.update(t, t)
    assert g.status(t) == 'frei'
    assert g.remaining(t) is None


def released(a, b, g):
    """Bis zur Freigabe füttern. Rückgabe: Zeit der Freigabe, zu der auch die letzte Nachricht kam."""
    for k in range(100):
        t = T0 + k * 0.1
        feed([a, b], t)
        if not g.update(t, t):
            return t
    raise AssertionError('keine Freigabe')


def test_silent_source_locks_immediately():
    a, b, g = make()
    t = released(a, b, g)
    feed([b], t + 0.1)                         # nur die Kamera sendet weiter
    feed([b], t + 0.3)
    assert not g.update(t + 0.3, t + 0.3)      # Scan 0,3 s still: noch in Ordnung
    feed([b], t + 0.5)
    assert g.update(t + 0.5, t + 0.5)          # 0,5 s > 0,4 s
    assert list(g.problems) == ['/scan']
    assert g.problems['/scan'].startswith('seit 0.5 s still')


def test_relock_needs_full_release_time_again():
    a, b, g = make()
    t = released(a, b, g)
    assert g.update(t + 0.5, t + 0.5)          # beide still → gesperrt
    t0 = t + 0.6
    for k in range(10):                        # 0 … 0,9 s frisch: noch gesperrt
        t = t0 + k * 0.1
        feed([a, b], t)
        assert g.update(t, t)
    feed([a, b], t0 + 1.0)
    assert not g.update(t0 + 1.0, t0 + 1.0)


def test_short_dropout_during_release_restarts_wait():
    a, b, g = make()
    for k in range(6):
        t = T0 + k * 0.1
        feed([a, b], t)
        assert g.update(t, t)
    t += 0.1
    feed([b], t)
    assert g.update(t + 0.5, t + 0.5)          # Scan fällt kurz aus
    assert g.remaining(t + 0.5) is None
    t += 0.6
    feed([a, b], t)
    assert g.update(t, t)
    assert g.remaining(t) == 1.0


def test_old_stamp_locks_although_messages_arrive():
    """Daten kommen pünktlich an, tragen aber alte Zeitstempel: der Collision Monitor würde sie ignorieren."""
    a, b, g = make()
    t = released(a, b, g)
    feed([a, b], t, age=0.35)
    assert not g.update(t, t)
    feed([a, b], t + 0.1, age=0.6)
    assert g.update(t + 0.1, t + 0.1)
    assert g.problems['/scan'] == 'Daten 0.60 s alt'


def test_future_stamp_locks():
    a, b, g = make()
    t = released(a, b, g)
    feed([a, b], t, age=-0.1)                   # 0,1 s in der Zukunft: erlaubt
    assert not g.update(t, t)
    feed([a, b], t + 0.1, age=-0.5)
    assert g.update(t + 0.1, t + 0.1)
    assert g.problems['/scan'] == 'Zeitstempel 0.50 s in der Zukunft'


def test_zero_stamp_locks():
    a, b, g = make()
    released(a, b, g)
    a.update(0.0, T0 + 10)
    b.update(T0 + 10, T0 + 10)
    assert g.update(T0 + 10, T0 + 10)
    assert g.problems == {'/scan': 'ohne Zeitstempel'}


def test_clock_jump_of_ros_time_counts_as_old_data():
    """ROS-Zeit springt vor (z. B. NTP): Zeitstempel wirken alt → gesperrt, wie es auch der Collision Monitor sähe."""
    a, b, g = make()
    t = released(a, b, g)
    feed([a, b], t)
    assert g.update(t + 5.0, t + 0.05)
    assert g.problems['/scan'].startswith('Daten 5.0')


def test_worst_age_statistics():
    a, b, g = make()
    t = released(a, b, g)
    a.reset_stats()
    feed([a, b], t, age=0.1)
    g.update(t + 0.2, t + 0.2)
    assert abs(a.worst_age - 0.3) < 1e-9
    a.reset_stats()
    assert a.worst_age == 0.0
