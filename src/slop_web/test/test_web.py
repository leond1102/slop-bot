"""Weboberfläche ohne ROS: Totmann, Eingabeprüfung, Prozesse starten/stoppen/übernehmen."""
import os
import subprocess
import sys
import time

import pytest

from slop_web import config as webconfig
from slop_web.drive import Drive, DriveLimits
from slop_web.processes import ProcessManager

PY = sys.executable


# ---------------------------------------------------------------------- Totmann
def test_drive_deadman_and_stop_hold():
    d = Drive(DriveLimits(max_linear=0.2, max_angular=0.4, deadman=0.3, stop_hold=0.3))
    assert d.output(0.0) is None                                 # nichts gedrückt: nichts senden
    d.command(1, -1, 0.5, 0.0)
    assert d.output(0.1) == pytest.approx((0.1, -0.2))
    assert d.output(0.35) == (0.0, 0.0)                          # Befehle bleiben aus → Nullbefehle
    assert d.output(0.55) == (0.0, 0.0)
    assert d.output(0.65) is None                                # danach still (Navigation darf)
    d.command(1, 0, 1.0, 1.0)
    assert d.output(1.05) == pytest.approx((0.2, 0.0))           # neuer Befehl fährt wieder


def test_drive_release_halt_and_bad_input():
    d = Drive(DriveLimits(max_linear=0.2, max_angular=0.4))
    d.command(1, 0, 1.0, 0.0)
    d.command(0, 0, 1.0, 0.05)                                   # Taste los
    assert d.output(0.06) == (0.0, 0.0) and d.output(0.4) is None
    d.command(5, float('nan'), 9, 1.0)                           # Unsinn vom Browser wird begrenzt
    assert d.output(1.01) == pytest.approx((0.2, 0.0))
    d.halt(1.02)
    assert d.output(1.03) == (0.0, 0.0)
    d.command('x', None, 'y', 2.0)
    assert d.output(2.01) in (None, (0.0, 0.0))


# ---------------------------------------------------------------------- Eingaben
@pytest.fixture
def cfg(tmp_path):
    maps = tmp_path / 'maps'
    maps.mkdir()
    for n in ('office-3.yaml', 'office-3.last_pose.yaml', 'lab.yaml'):
        (maps / n).write_text('x')
    path = tmp_path / 'web.yaml'
    path.write_text(f'''
maps_dir: {maps}
default_map: office-3
items:
  - {{id: nav, label: Nav, cmd: [ros2, launch, x, nav.launch.py, 'map:=$map', 'costs:=$experience_costs']}}
sequences:
  - {{id: all, label: Alles, steps: [nav]}}
actions:
  - {{id: save, label: Speichern, cmds: [[ros2, run, saver, -f, '$maps_dir/$name']]}}
''')
    return webconfig.load(str(path))


def test_maps_and_params(cfg):
    assert webconfig.maps(cfg['maps_dir']) == ['lab', 'office-3']           # ohne .last_pose.yaml
    v = webconfig.params(cfg, {'map': 'lab', 'experience_costs': True})
    assert webconfig.argv(cfg['items'][0]['cmd'], v)[-2:] == [f'map:={cfg["maps_dir"]}/lab.yaml', 'costs:=true']
    assert webconfig.params(cfg, {})['map'].endswith('office-3.yaml')        # Standardkarte


@pytest.mark.parametrize('raw', [{'map': '../../etc/passwd'}, {'map': 'gibtsnicht'}, {'name': 'a b'},
                                 {'name': '$(reboot)'}, {'name': 'x' * 41}, {'name': 5}])
def test_params_reject_bad_input(cfg, raw):
    with pytest.raises(ValueError):
        webconfig.params(cfg, raw)


def test_action_needs_name(cfg):
    with pytest.raises(ValueError):
        webconfig.argv(cfg['actions'][0]['cmds'][0], webconfig.params(cfg, {}))
    out = webconfig.argv(cfg['actions'][0]['cmds'][0], webconfig.params(cfg, {'name': 'office-4'}))
    assert out[-1] == f'{cfg["maps_dir"]}/office-4'


def test_config_rejects_unknown_step(tmp_path):
    p = tmp_path / 'w.yaml'
    p.write_text('items: [{id: a, label: A, cmd: [x]}]\nsequences: [{id: s, label: S, steps: [a, b]}]\n')
    with pytest.raises(ValueError):
        webconfig.load(str(p))


# ---------------------------------------------------------------------- Prozesse
def wait_for(pred, timeout=10.0, pm=None):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pm is not None:
            pm.tick()
        if pred():
            return True
        time.sleep(0.05)
    return False


def pm_for(tmp_path, **kw):
    return ProcessManager(str(tmp_path / 'logs'), str(tmp_path / 'state' / 'state.json'), [], **kw)


# Prozess mit Kind (wie ros2 launch → Node); beide schreiben ins Log und beenden sich bei SIGINT
WORKER = '''
import signal, subprocess, sys, time
child = subprocess.Popen([sys.executable, "-c", "import time\\nwhile True: time.sleep(0.1)"])
print("bereit", flush=True)
try:
    while True: time.sleep(0.1)
except KeyboardInterrupt:
    print("Strg+C erhalten", flush=True)
    child.wait()
'''
# ignoriert SIGINT (wie ein hängender Treiber) → muss per SIGTERM/SIGKILL weg
STUBBORN = 'import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\nprint("stur", flush=True)\nwhile True: time.sleep(0.1)'


def test_start_stop_like_ctrl_c(tmp_path):
    pm = pm_for(tmp_path)
    env = dict(os.environ)
    pm.start('a', [PY, '-c', WORKER], env)
    assert wait_for(lambda: 'bereit' in pm.tail('a').splitlines(), pm=pm)
    assert wait_for(lambda: len(pm.runs['a'].members) >= 2, pm=pm)          # Elternteil + Kind erfasst
    with pytest.raises(RuntimeError):
        pm.start('a', [PY, '-c', WORKER], env)                               # läuft schon
    assert pm.stop('a')
    assert wait_for(lambda: pm.state('a')[0] == 'beendet', pm=pm)
    assert 'Strg+C erhalten' in pm.tail('a')
    assert pm.runs['a'].sent == 'SIGINT'                                     # ohne Eskalation


def test_stubborn_process_escalates(tmp_path):
    pm = pm_for(tmp_path, stop_term=0.5, stop_kill=1.0)
    pm.start('b', [PY, '-c', STUBBORN], dict(os.environ))
    assert wait_for(lambda: 'stur' in pm.tail('b').splitlines(), pm=pm)
    pm.stop('b')
    assert wait_for(lambda: pm.state('b')[0] == 'beendet', timeout=5, pm=pm)
    assert pm.runs['b'].sent in ('SIGTERM', 'SIGKILL')


def test_adopt_after_restart_and_external(tmp_path):
    pm = pm_for(tmp_path)
    pm.start('a', [PY, '-c', WORKER], dict(os.environ))
    assert wait_for(lambda: 'bereit' in pm.tail('a').splitlines(), pm=pm)
    # „Fremder“ Prozess mit passender Befehlszeile (wie im Terminal gestartet)
    ext = subprocess.Popen([PY, '-c', 'import time; time.sleep(30)', 'ros2 launch fake_pkg extern.launch.py'])
    try:
        pm2 = pm_for(tmp_path)                                               # Webserver neu gestartet
        assert pm2.is_running('a')
        pm2.tick(['extern.launch.py', 'nichts.launch.py'])
        assert ext.pid in pm2.external['extern.launch.py']
        assert pm2.external['nichts.launch.py'] == []
        assert not pm2.stop('fremd')                                         # Unbekanntes wird nicht angefasst
        assert pm2.stop('a')
        assert wait_for(lambda: pm2.state('a')[0] == 'beendet', pm=pm2)
        assert ext.poll() is None                                            # Fremder läuft weiter
    finally:
        ext.kill()
        ext.wait()
        pm.runs['a'].popen.wait(timeout=5)                                   # Zombie einsammeln


# ---------------------------------------------------------------------- DDS-Einstellungen
def test_apply_dds_overrides_terminal_settings():
    env = {'ROS_LOCALHOST_ONLY': '0', 'FASTRTPS_DEFAULT_PROFILES_FILE': '/x/udp.xml', 'ROS_DOMAIN_ID': '80', 'HOME': '/h'}
    eff, changes = webconfig.apply_dds({'ROS_LOCALHOST_ONLY': '1', 'FASTRTPS_DEFAULT_PROFILES_FILE': None,
                                        'CYCLONEDDS_URI': None}, env)
    assert env['ROS_LOCALHOST_ONLY'] == '1' and 'FASTRTPS_DEFAULT_PROFILES_FILE' not in env
    assert eff == {'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '80'}          # nicht genannt: bleibt (Tests)
    assert len(changes) == 2                                                  # beide Abweichungen gemeldet
    _, again = webconfig.apply_dds({'ROS_LOCALHOST_ONLY': '1'}, env)
    assert again == []                                                        # passt schon: nichts zu melden
