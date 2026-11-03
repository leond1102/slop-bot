"""Tastenlogik der Tastatursteuerung, ohne ROS und ohne Terminal."""
from slop_teleop.keys import KeyTeleop, parse_keys


def test_parse_letters_and_arrows():
    assert parse_keys(b'wW \x1b[A\x1b[D\x1bOB') == ['w', 'w', ' ', 'up', 'left', 'down']


def test_parse_arrow_with_modifier_and_other_escapes():
    # Strg+→, F1, Bild-auf: Zeichen der Folgen dürfen nicht als Fahrtasten zählen
    assert parse_keys(b'\x1b[1;5C\x1bOP\x1b[5~d') == ['right', 'd']
    assert parse_keys(b'\x1b') == []                 # einzelnes ESC
    assert parse_keys(b'\x1b[') == []                # abgeschnittene Folge


def test_single_press_drives_until_first_timeout():
    t = KeyTeleop(first_timeout=0.7, repeat_timeout=0.25)
    t.press('w', 0.0)
    assert t.command(0.5) == (1.0, 0.0)              # Wiederholverzögerung abwarten
    assert t.command(0.71) == (0.0, 0.0)             # keine Wiederholung → losgelassen


def test_held_key_stops_soon_after_release():
    t = KeyTeleop(first_timeout=0.7, repeat_timeout=0.25)
    for now in [0.0, 0.5, 0.53, 0.56]:               # Druck, dann Wiederholungen
        t.press('a', now)
    assert t.command(0.7) == (0.0, 1.0)
    assert t.command(0.82) == (0.0, 0.0)             # 0,26 s nach letzter Wiederholung


def test_stop_and_switch():
    t = KeyTeleop()
    t.press('w', 0.0)
    t.press('d', 0.05)                               # Wechsel: sofort neue Richtung
    assert t.command(0.1) == (0.0, -1.0)
    t.press(' ', 0.1)
    assert t.command(0.1) == (0.0, 0.0)
    t.press('e', 1.0)
    assert t.command(1.0) == (1.0, -1.0)


def test_released_key_pressed_again_counts_as_new_press():
    t = KeyTeleop(first_timeout=0.7, repeat_timeout=0.25)
    t.press('w', 0.0)
    t.press('w', 0.5)                                # Wiederholung
    t.press('w', 5.0)                                # neuer Druck → wieder langer Timeout
    assert t.command(5.6) == (1.0, 0.0)


def test_scale_limits():
    t = KeyTeleop(scale=1.0, scale_step=0.1)
    t.press('+', 0.0)
    assert t.scale == 1.0
    for _ in range(20):
        t.press('-', 0.0)
    assert t.scale == 0.1
    t.press('s', 0.0)
    assert t.command(0.0) == (-0.1, 0.0)


def test_unknown_key():
    t = KeyTeleop()
    assert t.press('z', 0.0) is False
    assert t.command(0.0) == (0.0, 0.0)
