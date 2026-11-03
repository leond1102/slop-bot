"""RustDesk-Erkennung mit Beispielausgaben, ohne ROS und ohne laufendes RustDesk."""
import os

from slop_safety.remote_guard import LockDelay, cm_running, parse_ss

# Echte Ausgabe vom 03.11. (direkte LAN-Sitzung), gekürzt
SS_ACTIVE = (
    '0      0      [::ffff:192.168.20.133]:21118 [::ffff:192.168.20.129]:51955\n'
    '\t cubic wscale:8,7 rto:204 rtt:3.1/1.2 mss:1448 lastsnd:148 lastrcv:144 lastack:144 pacing_rate 10Mbps\n'
)
SS_TWO = SS_ACTIVE + (
    '0      0      192.168.20.133:40000 1.2.3.4:21117\n'
    '\t cubic lastsnd:9000 lastrcv:8000 lastack:7000\n'
)


def test_parse_active():
    conns = parse_ss(SS_ACTIVE)
    assert len(conns) == 1
    assert conns[0].peer == '[::ffff:192.168.20.129]:51955'
    assert conns[0].silence_ms == 144


def test_parse_two_connections():
    conns = parse_ss(SS_TWO)
    assert [c.silence_ms for c in conns] == [144, 7000]


def test_parse_empty_and_missing_info():
    assert parse_ss('') == []
    conns = parse_ss('0 0 a:21118 b:1\n')
    assert conns[0].silence_ms == float('inf')       # ganz ohne Infozeile → gilt als still


def test_parse_missing_fields_mean_zero():
    # ss lässt lastrcv/lastack weg, wenn sie 0 sind (03.11. beobachtet, löste Fehlsperren aus)
    conns = parse_ss('0 0 a:21118 b:1\n\t cubic rto:236 lastsnd:4 pacing_rate 5918272bps\n')
    assert conns[0].silence_ms == 0
    conns = parse_ss('0 0 a:21118 b:1\n\t cubic lastsnd:4 lastrcv:120\n')   # nur lastack fehlt
    assert conns[0].silence_ms == 0


def _fake_proc(tmp_path, cmdlines):
    for pid, args in enumerate(cmdlines, 100):
        d = tmp_path / str(pid)
        d.mkdir()
        (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in args) + b'\0')
    (tmp_path / 'self').mkdir()
    return str(tmp_path)


def test_cm_running(tmp_path):
    proc = _fake_proc(tmp_path, [['/usr/share/rustdesk/rustdesk', '--server'],
                                 ['/usr/share/rustdesk/rustdesk', '--cm']])
    assert cm_running(proc)


def test_cm_not_running(tmp_path):
    proc = _fake_proc(tmp_path, [['/usr/share/rustdesk/rustdesk', '--server'],
                                 ['/usr/bin/vim', 'notes--cm.txt'], ['/bin/grep', 'rustdesk', '--cm']])
    assert not cm_running(proc)
    assert os.path.isdir(proc)


def test_lock_delay_starts_locked_and_unlocks_at_once():
    d = LockDelay(3.0)
    assert not d.update(False, 0.0)
    assert not d.update(False, 10.0)                  # nie eine Sitzung gehabt → bleibt gesperrt
    assert d.update(True, 10.2)


def test_lock_delay_short_dropout_does_not_lock():
    d = LockDelay(3.0)
    d.update(True, 0.0)
    assert d.update(False, 1.0)                       # Aussetzer beginnt
    assert d.remaining(2.0) == 2.0
    assert d.update(False, 3.8)
    assert d.update(True, 3.9)                        # zurück vor Ablauf
    assert d.remaining(4.0) is None
    assert d.update(False, 5.0)                       # neuer Aussetzer: Wartezeit von vorn
    assert d.update(False, 7.9)


def test_lock_delay_locks_after_continuous_loss():
    d = LockDelay(3.0)
    d.update(True, 0.0)
    for t in [0.2, 1.0, 2.0, 3.0]:
        assert d.update(False, t)
    assert not d.update(False, 3.2)                   # 3 s ununterbrochen weg
    assert not d.update(False, 4.0)
    assert d.update(True, 4.2)                        # Freigabe sofort wieder
