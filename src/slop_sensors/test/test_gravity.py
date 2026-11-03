"""Einbaulage aus der Schwerkraft, ohne ROS."""
import math

import pytest

from slop_sensors.gravity import StillWindow, quat_from_rpy, roll_pitch_from_up, rotate


def up_in_sensor(roll, pitch, yaw):
    """Vektor nach oben (Basis z) im Sensorframe, wenn der Sensor mit rpy in der Basis sitzt."""
    x, y, z, w = quat_from_rpy(roll, pitch, yaw)
    return rotate((-x, -y, -z, w), (0.0, 0.0, 1.0))   # inverse Drehung


def test_upright():
    assert roll_pitch_from_up(0, 0, 9.81) == pytest.approx((0.0, 0.0))


def test_upside_down():
    roll, pitch = roll_pitch_from_up(0, 0, -1.0)
    assert abs(roll) == pytest.approx(math.pi) and pitch == pytest.approx(0.0)


def test_units_do_not_matter():
    assert roll_pitch_from_up(0.1, 0.2, 0.97) == pytest.approx(roll_pitch_from_up(0.981, 1.962, 9.5157))


@pytest.mark.parametrize('roll,pitch,yaw', [
    (0.0, math.radians(-8.6), 0.0),       # Kamera nach oben geneigt
    (math.radians(15), math.radians(-20), math.radians(90)),
    (math.radians(-30), math.radians(40), math.radians(-135)),
    (math.radians(170), math.radians(10), math.radians(30)),
])
def test_round_trip_independent_of_yaw(roll, pitch, yaw):
    assert roll_pitch_from_up(*up_in_sensor(roll, pitch, yaw)) == pytest.approx((roll, pitch), abs=1e-9)


@pytest.mark.parametrize('rpy', [(0.1, -0.2, 0.3), (-1.0, 0.5, -2.8), (3.0, 0.0, 1.0)])
def test_rpy_from_quat_round_trip(rpy):
    from slop_sensors.gravity import rpy_from_quat
    assert rpy_from_quat(quat_from_rpy(*rpy)) == pytest.approx(rpy)


def test_quat_matches_known_values():
    # 90° um z
    assert quat_from_rpy(0, 0, math.pi / 2) == pytest.approx((0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    # RealSense: camera_link → camera_imu_optical_frame ist rpy (-90°, 0, -90°)
    q = quat_from_rpy(-math.pi / 2, 0, -math.pi / 2)
    # optische z-Achse (nach vorn) = camera_link x, optische y-Achse (nach unten) = -z
    assert rotate(q, (0, 0, 1)) == pytest.approx((1, 0, 0), abs=1e-12)
    assert rotate(q, (0, 1, 0)) == pytest.approx((0, 0, -1), abs=1e-12)


def test_measured_camera_case():
    # Gemessen am 2026-11-03: Schwerkraft im camera_imu_optical_frame
    q = quat_from_rpy(-math.pi / 2, 0, -math.pi / 2)
    a_link = rotate(q, (-0.054, -9.624, 1.453))
    roll, pitch = roll_pitch_from_up(*a_link)
    assert math.degrees(roll) == pytest.approx(0.3, abs=0.5)
    assert math.degrees(pitch) == pytest.approx(-8.6, abs=0.5)   # nach oben geneigt


def test_still_window():
    w = StillWindow(duration=1.0, min_samples=10)
    out = None
    for i in range(101):
        out = w.add(i * 0.01, (0.0, 0.0, 1.0 + 0.001 * (i % 3))) or out
    still, mean, rms = out
    assert still and mean[2] == pytest.approx(1.001, abs=1e-3) and rms < 0.1


def test_still_window_rejects_motion():
    w = StillWindow(duration=1.0, min_samples=10)
    out = None
    for i in range(101):
        ang = math.radians(20) * math.sin(i * 0.2)    # schwankt um ±20°
        out = w.add(i * 0.01, (math.sin(ang), 0.0, math.cos(ang))) or out
    assert out[0] is False
