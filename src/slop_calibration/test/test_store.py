"""Speicher der Kalibrierergebnisse, ohne ROS-Knoten."""
import os

import pytest

from slop_calibration import store


@pytest.fixture
def calib(tmp_path, monkeypatch):
    monkeypatch.setenv('SLOP_CALIB_DIR', str(tmp_path / 'calib'))
    return tmp_path / 'calib'


def test_missing_file_is_none(calib):
    assert store.load('gyro') is None
    assert store.load_entry('gyro', 'livox') is None


def test_save_and_load_keeps_other_sensors(calib):
    store.save_entry('gyro', 'livox', {'bias': [0.001, -0.002, 0.003]})
    store.save_entry('gyro', 'camera', {'bias': [0.0, 0.0, 0.1]})
    assert store.load_entry('gyro', 'livox')['bias'] == [0.001, -0.002, 0.003]
    assert store.load_entry('gyro', 'camera')['bias'] == [0.0, 0.0, 0.1]
    assert 'stamp' in store.load_entry('gyro', 'livox')


def test_overwrite_entry(calib):
    store.save_entry('gyro', 'livox', {'bias': [1.0, 1.0, 1.0]})
    store.save_entry('gyro', 'livox', {'bias': [2.0, 2.0, 2.0]})
    assert store.load_entry('gyro', 'livox')['bias'] == [2.0, 2.0, 2.0]
    assert not any(f.endswith('.tmp') for f in os.listdir(calib))


def test_broken_file_is_none(calib):
    os.makedirs(calib)
    (calib / 'gyro.yaml').write_text(': : nicht [gültig')
    assert store.load('gyro') is None


def test_age(calib):
    store.save_entry('gyro', 'livox', {'bias': [0, 0, 0]})
    assert 0.0 <= store.age_days(store.load_entry('gyro', 'livox')) < 0.01
    assert store.age_days({}) is None
