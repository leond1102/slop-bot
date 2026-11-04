"""IMU-Aufbereitung für den EKF, ohne Hardware und ohne ROS."""
import pytest

from slop_localization.imu import G, GyroBias, accel_g_to_ms2, diag_covariance


def test_accel_g_to_ms2():
    assert accel_g_to_ms2((0.0, 0.0, 1.0)) == (0.0, 0.0, G)
    assert accel_g_to_ms2((1.0, -2.0, 0.5), 10.0) == (10.0, -20.0, 5.0)


def test_diag_covariance():
    cov = diag_covariance(0.1)
    assert cov[0] == cov[4] == cov[8] == pytest.approx(0.01)
    assert sum(cov) == pytest.approx(0.03)


def _feed(bias, n, dt, gyro):
    done = False
    for i in range(n):
        done = bias.add(i * dt, gyro) or done
    return done


def test_bias_from_still_robot():
    bias = GyroBias(duration=1.0, max_std=0.02)
    assert not bias.ready
    assert _feed(bias, 300, 0.005, (0.001, -0.002, 0.003))
    assert bias.ready
    assert bias.bias == pytest.approx((0.001, -0.002, 0.003))
    assert bias.correct((0.011, 0.0, 0.103)) == pytest.approx((0.010, 0.002, 0.100))


def test_bias_rejected_while_moving():
    bias = GyroBias(duration=1.0, max_std=0.02)
    for i in range(300):
        bias.add(i * 0.005, (0.0, 0.0, 0.5 if i % 2 else -0.5))   # dreht hin und her
    assert not bias.ready and bias.rejected >= 1


def test_uncalibrated_gyro_passes_through():
    assert GyroBias(1.0, 0.02).correct((0.1, 0.2, 0.3)) == (0.1, 0.2, 0.3)


def test_restart_keeps_old_bias_until_new_one():
    bias = GyroBias(duration=1.0, max_std=0.02)
    _feed(bias, 300, 0.005, (0.0, 0.0, 0.01))
    bias.restart()
    assert bias.bias == pytest.approx((0.0, 0.0, 0.01))
    _feed(bias, 300, 0.005, (0.0, 0.0, 0.02))
    assert bias.bias == pytest.approx((0.0, 0.0, 0.02))


def test_set_bias_is_ready_immediately():
    bias = GyroBias(duration=5.0, max_std=0.02)
    bias.set([0.001, 0.002, 0.003])
    assert bias.ready
    assert bias.correct((0.011, 0.012, 0.013)) == pytest.approx((0.010, 0.010, 0.010))


def test_bias_stays_fixed_after_calibration():
    """Eine gleichmäßige Drehung nach der Kalibrierung darf nicht als neuer Bias gelten."""
    bias = GyroBias(duration=1.0, max_std=0.02)
    _feed(bias, 300, 0.005, (0.0, 0.0, 0.001))
    assert not _feed(bias, 2000, 0.005, (0.0, 0.0, 0.5))
    assert bias.bias == pytest.approx((0.0, 0.0, 0.001))
    bias.restart()                                  # erst auf Anforderung wird neu gemessen
    assert _feed(bias, 300, 0.005, (0.0, 0.0, 0.002))
    assert bias.bias == pytest.approx((0.0, 0.0, 0.002))
