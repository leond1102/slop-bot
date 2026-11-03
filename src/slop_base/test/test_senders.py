"""Prüfung der /cmd_vel-Publisher, ohne ROS."""
from slop_base.senders import UNKNOWN_NODE, SenderCheck


def test_only_twist_mux_is_fine():
    c = SenderCheck(['twist_mux'])
    assert c.check(['twist_mux'], 0.0) == (False, [], False)


def test_foreign_blocks_immediately():
    c = SenderCheck(['twist_mux'])
    assert c.check(['twist_mux', 'teleop'], 0.0) == (True, ['teleop'], False)


def test_unknown_name_blocks_but_waits_during_grace():
    c = SenderCheck(['twist_mux'], grace=3.0)
    assert c.check([UNKNOWN_NODE], 0.0) == (True, [], True)
    assert c.check([UNKNOWN_NODE], 2.9) == (True, [], True)
    # Name aufgelöst → frei
    assert c.check(['twist_mux'], 3.1) == (False, [], False)


def test_unknown_name_becomes_error_after_grace():
    c = SenderCheck(['twist_mux'], grace=3.0)
    c.check([UNKNOWN_NODE], 0.0)
    assert c.check([UNKNOWN_NODE], 3.0) == (True, [UNKNOWN_NODE], False)


def test_grace_restarts_after_resolution():
    c = SenderCheck(['twist_mux'], grace=3.0)
    c.check([UNKNOWN_NODE], 0.0)
    c.check(['twist_mux'], 1.0)
    assert c.check([UNKNOWN_NODE], 10.0) == (True, [], True)


def test_foreign_plus_unknown_reports_foreign():
    c = SenderCheck(['twist_mux'])
    assert c.check([UNKNOWN_NODE, 'rogue'], 0.0) == (True, ['rogue'], False)
