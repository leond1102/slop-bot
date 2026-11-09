"""Greifergeometrie: Objektmitte/-breite in Y ↔ Backenpositionen, ohne ROS."""
import pytest

from slop_base.gripper_geometry import Geometry, GripperError


def test_centered_object_symmetric():
    g = Geometry(open_width=0.200, travel=0.178, squeeze=0.0)
    pl, pr = g.jaws_from_object(0.0, 0.040)
    assert pl == pytest.approx(0.080) and pr == pytest.approx(0.080)


def test_offset_object_and_back():
    for plus in (False, True):
        g = Geometry(open_width=0.200, travel=0.178, squeeze=0.0, left_is_plus_y=plus)
        pl, pr = g.jaws_from_object(0.030, 0.040)
        y, w = g.object_from_jaws(pl, pr)
        assert y == pytest.approx(0.030) and w == pytest.approx(0.040)


def test_left_minus_y_means_left_jaw_goes_further_for_plus_y():
    g = Geometry(open_width=0.200, travel=0.178, squeeze=0.0, left_is_plus_y=False)
    pl, pr = g.jaws_from_object(0.030, 0.040)      # Objekt Richtung +Y → -Y-Backe (links) fährt weiter
    assert pl == pytest.approx(0.110) and pr == pytest.approx(0.050)


def test_squeeze_closes_both_jaws_half():
    g = Geometry(open_width=0.200, travel=0.178, squeeze=0.001)
    pl, pr = g.jaws_from_object(0.0, 0.040)
    assert pl == pytest.approx(0.0805) and pr == pytest.approx(0.0805)


def test_shift_keeps_width():
    for plus in (False, True):
        g = Geometry(open_width=0.200, travel=0.178, squeeze=0.0, left_is_plus_y=plus)
        pl, pr = g.jaws_from_object(0.0, 0.040)
        nl, nr = g.shift(pl, pr, -0.025)
        y, w = g.object_from_jaws(nl, nr)
        assert y == pytest.approx(-0.025) and w == pytest.approx(0.040)


def test_limits():
    g = Geometry(open_width=0.200, travel=0.178)
    with pytest.raises(GripperError):
        g.jaws_from_object(0.0, 0.250)       # breiter als offen
    with pytest.raises(GripperError):
        g.jaws_from_object(0.090, 0.010)     # zu weit außen
    with pytest.raises(GripperError):
        g.jaws_from_object(0.0, 0.0)
    pl, pr = g.jaws_from_object(0.0, 0.040)
    with pytest.raises(GripperError):
        g.shift(pl, pr, 0.090)
    with pytest.raises(GripperError):
        Geometry(open_width=0.0, travel=0.178)
