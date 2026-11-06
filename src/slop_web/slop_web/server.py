#!/usr/bin/env python3
"""Slop-Bot Weboberfläche: Starten/Stoppen, Fahren, Notaus – im Browser, auch am Handy (Tailnet).

    ros2 run slop_web web                      # http://<tailscale-ip>:8080  (Handy: …:8080/handy)
    ros2 run slop_web web --config eigene.yaml

Lauscht nur auf 127.0.0.1 und der Tailscale-Adresse (config/web.yaml: listen), nicht im übrigen Netz. Websockets von
fremden Seiten werden abgewiesen (Origin-Prüfung von tornado).

Gestartete Programme laufen weiter, wenn der Webserver beendet wird; beim nächsten Start übernimmt er sie wieder.
"""
import argparse
import asyncio
import json
import logging
import os
import signal
import subprocess
import threading
import time

import rclpy
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from slop_web import config as webconfig
from slop_web.drive import DriveLimits
from slop_web.processes import ProcessManager
from slop_web.ros_bridge import Bridge

log = logging.getLogger('slop_web')


# ---------------------------------------------------------------------- Umgebung für gestartete Programme
def desktop_env(uid):
    """DISPLAY/XAUTHORITY der Desktop-Sitzung (für RViz), aus einem laufenden gnome-shell/Xorg des Nutzers."""
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            if os.stat(f'/proc/{pid}').st_uid != uid:
                continue
            with open(f'/proc/{pid}/comm') as f:
                if f.read().strip() not in ('gnome-shell', 'Xorg', 'gnome-session-b'):
                    continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = dict(kv.split(b'=', 1) for kv in f.read().split(b'\0') if b'=' in kv)
        except OSError:
            continue
        if b'DISPLAY' in env:
            out = {'DISPLAY': env[b'DISPLAY'].decode()}
            if b'XAUTHORITY' in env:
                out['XAUTHORITY'] = env[b'XAUTHORITY'].decode()
            return out
    fallback = f'/run/user/{uid}/gdm/Xauthority'
    return {'DISPLAY': ':0', **({'XAUTHORITY': fallback} if os.path.exists(fallback) else {})}


def child_env():
    """Saubere Umgebung wie ein neues Terminal mit ROS (ohne venv, ohne ~/ros2_ws), plus Desktop für RViz.
    DDS-Einstellungen genau wie beim Webserver selbst (nach apply_dds), damit er sieht, was er startet."""
    uid = os.getuid()
    keep = ('HOME', 'USER', 'LOGNAME', 'LANG', 'LANGUAGE', 'LC_ALL', 'TZ', 'ETHERCAT_VENV') + webconfig.DDS_KEYS
    env = {k: os.environ[k] for k in keep if k in os.environ}
    env.update(PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
               XDG_RUNTIME_DIR=f'/run/user/{uid}', PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
    env.update(desktop_env(uid))
    return env


def tailscale_ip():
    try:
        out = subprocess.run(['tailscale', 'ip', '-4'], capture_output=True, text=True, timeout=5).stdout.split()
        return out[0] if out else None
    except (OSError, subprocess.TimeoutExpired):
        return None


# ---------------------------------------------------------------------- Kern
class Web:
    def __init__(self, cfg, bridge):
        self.cfg, self.bridge = cfg, bridge
        home = os.path.expanduser('~')
        self.pm = ProcessManager(os.path.join(home, '.ros', 'log', 'slop_web'),
                                 os.path.join(home, '.ros', 'slop_web', 'state.json'),
                                 [cfg['ros_setup'], os.path.expanduser(cfg['workspace_setup'])])
        self.items = {i['id']: i for i in cfg.get('items', [])}
        self.seqs = {s['id']: s for s in cfg.get('sequences', [])}
        self.actions = {a['id']: a for a in cfg.get('actions', [])}
        self.seq_state = {}             # Abfolge → Text („startet Kamera …“)
        self.busy_actions = set()
        self.clients = set()
        self.action_log = os.path.join(home, '.ros', 'log', 'slop_web', 'aktionen.log')

    # ---------------- Status
    def tick(self):
        self.pm.tick([i['detect'] for i in self.items.values() if i.get('detect')])

    def external(self, id):
        pat = self.items[id].get('detect')
        return self.pm.external.get(pat, []) if pat else []

    def running(self, id):
        return self.pm.is_running(id) or bool(self.external(id))

    def item_status(self, id):
        state, run = self.pm.state(id)
        ext = self.external(id)
        if state in ('aus', 'beendet') and ext:
            state = 'extern'
        out = {'state': state, 'external': ext}
        ready = self.items[id].get('ready')
        if ready:
            out['data'] = self.bridge.fresh(ready, 2.0)      # kommen die erwarteten Daten bei der Seite an?
            out['ready'] = ready
        if run is not None:
            out.update(pid=run.pid, started=run.started, exit_code=run.exit_code, signal=run.sent,
                       last=(self.pm.tail(id, 1) if state in ('läuft', 'stoppt', 'beendet') else ''))
        return out

    def status(self):
        return {'t': 'status', 'robot': self.bridge.status(), 'seq': self.seq_state,
                'items': {id: self.item_status(id) for id in self.items}, 'busy': sorted(self.busy_actions),
                'now': time.time()}

    def broadcast(self, msg):
        data = json.dumps(msg)
        for c in list(self.clients):
            try:
                c.write_message(data)
            except tornado.websocket.WebSocketClosedError:
                self.clients.discard(c)

    def notify(self, text, level='info'):
        log.info(text) if level != 'error' else log.warning(text)
        self.broadcast({'t': 'msg', 'level': level, 'text': text})

    # ---------------- Starten / Stoppen
    def start_item(self, id, raw):
        item = self.items[id]
        argv = webconfig.argv(item['cmd'], webconfig.params(self.cfg, raw))    # Eingaben zuerst prüfen
        if self.pm.is_running(id):
            raise ValueError(f'{item["label"]} läuft schon')
        if self.external(id):
            raise ValueError(f'{item["label"]} läuft schon (im Terminal gestartet)')
        for c in item.get('conflicts', []):
            if self.running(c):
                raise ValueError(f'Erst „{self.items[c]["label"]}“ stoppen (geht nicht gleichzeitig)')
        self.pm.start(id, argv, child_env())
        self.notify(f'{item["label"]} gestartet')

    async def wait_ready(self, id):
        item = self.items[id]
        topic, timeout = item.get('ready'), float(item.get('ready_timeout', 30))
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if not self.running(id):
                raise ValueError(f'{item["label"]} hat sich beim Start beendet – Log ansehen')
            if topic is None or self.bridge.fresh(topic, 1.0):
                return
            await asyncio.sleep(0.5)
        raise ValueError(f'{item["label"]}: nach {timeout:g} s keine Daten auf {topic} – Log ansehen')

    async def run_sequence(self, id, raw):
        seq = self.seqs[id]
        if self.seq_state.get(id):
            raise ValueError(f'{seq["label"]} startet schon')
        webconfig.params(self.cfg, raw)                     # Eingaben vorab prüfen
        try:
            for step in seq['steps']:
                label = self.items[step]['label']
                if self.running(step):
                    continue
                self.seq_state[id] = f'startet {label} …'
                self.start_item(step, raw)
                self.seq_state[id] = f'warte auf {label} …'
                await self.wait_ready(step)
            self.notify(f'{seq["label"]}: alles läuft')
        finally:
            self.seq_state.pop(id, None)

    def stop(self, id):
        if id in self.seqs:
            for step in reversed(self.seqs[id]['steps']):
                if self.pm.is_running(step):
                    self.pm.stop(step)
            return
        if id not in self.items:
            raise ValueError(f'unbekannt: {id}')
        if not self.pm.stop(id):
            if self.external(id):
                raise ValueError(f'{self.items[id]["label"]} wurde im Terminal gestartet – dort mit Strg+C beenden')
            raise ValueError(f'{self.items[id]["label"]} läuft nicht')
        self.notify(f'{self.items[id]["label"]} wird gestoppt (wie Strg+C)')

    def stop_all(self):
        order = [s for seq in self.seqs.values() for s in reversed(seq['steps'])] + list(self.items)
        for id in dict.fromkeys(order):                     # Antrieb zuletzt, Reihenfolge ohne Doppelte
            if self.pm.is_running(id):
                self.pm.stop(id)

    # ---------------- Aktionen
    def _run_cmds(self, argvs):
        setup = ' && '.join(f'source {s}' for s in self.pm.setup_scripts)
        lines = []
        for argv in argvs:
            try:
                p = subprocess.run(['bash', '-c', f'{setup} && exec "$@"', 'slop_web', *argv], env=child_env(),
                                   capture_output=True, text=True, timeout=25, stdin=subprocess.DEVNULL)
                out, ok = (p.stdout + p.stderr).strip(), p.returncode == 0
            except subprocess.TimeoutExpired:
                out, ok = 'keine Antwort nach 25 s (läuft der Dienst?)', False
            with open(self.action_log, 'a', encoding='utf-8') as f:
                f.write(f'# {time.strftime("%Y-%m-%d %H:%M:%S")}  {" ".join(argv)}\n{out}\n')
            lines.append(('✓ ' if ok else '✗ ') + (out.splitlines()[-1] if out else ' '.join(argv[:4])))
        return lines

    async def run_action(self, id, raw):
        action = self.actions[id]
        if id in self.busy_actions:
            raise ValueError(f'{action["label"]} läuft noch')
        if action.get('needs_name') and not raw.get('name'):
            raise ValueError('Bitte einen Namen angeben')
        values = webconfig.params(self.cfg, raw)
        argvs = [webconfig.argv(c, values) for c in action['cmds']]
        self.busy_actions.add(id)
        try:
            lines = await asyncio.get_running_loop().run_in_executor(None, self._run_cmds, argvs)
        finally:
            self.busy_actions.discard(id)
        ok = all(line.startswith('✓') for line in lines)
        self.notify(f'{action["label"]}: ' + ' | '.join(lines), 'info' if ok else 'error')

    # ---------------- Nachrichten vom Browser
    async def handle(self, client, msg):
        """Kurz halten: alles, was dauert, über background()."""
        t = msg.get('t')
        if t == 'drive':
            if self.bridge.drive_command(msg.get('v', 0), msg.get('w', 0), msg.get('scale', 0.5)):
                client.driving = True
            else:
                client.write_message(json.dumps({'t': 'msg', 'level': 'error', 'text': 'Notaus aktiv – erst aufheben'}))
        elif t == 'halt':
            self.bridge.drive_halt()
        elif t == 'estop':
            self.bridge.set_estop(True)
            self.notify('NOTAUS ausgelöst (Hand-Stopp-Sperre gesetzt, Nav2-Ziele abgebrochen)', 'error')
        elif t == 'release':
            self.bridge.set_estop(False)
            self.notify('Notaus aufgehoben')
        elif t == 'start':
            id = msg.get('id')
            if id in self.seqs:
                self.background(client, self.run_sequence(id, msg.get('params')))
            elif id in self.items:
                self.start_item(id, msg.get('params'))
            else:
                raise ValueError(f'unbekannt: {id}')
        elif t == 'stop':
            self.stop(msg.get('id'))
        elif t == 'stop_all':
            self.stop_all()
            self.notify('Alles wird gestoppt (Antrieb zuletzt)')
        elif t == 'action':
            if msg.get('id') not in self.actions:
                raise ValueError('unbekannte Aktion')
            self.background(client, self.run_action(msg['id'], msg.get('params') or {}))

    def background(self, client, coro):
        """Lange Vorgänge (Abfolgen, Dienstaufrufe) nicht abwarten: tornado liefert die nächste Nachricht derselben
        Verbindung erst, wenn on_message fertig ist – ein Notaus darf nie hinter einem Start warten."""
        async def run():
            try:
                await coro
            except ValueError as e:
                self.notify(str(e), 'error')
            except Exception as e:
                log.exception('Hintergrundaufgabe')
                self.notify(f'Fehler: {e}', 'error')
        asyncio.ensure_future(run())

    def client_config(self):
        c = self.cfg
        return {
            'items': [{k: i.get(k) for k in ('id', 'label', 'hint', 'uses_map', 'uses_experience_costs')}
                      for i in c.get('items', [])],
            'sequences': [{'id': s['id'], 'label': s['label'], 'steps': s['steps']} for s in c.get('sequences', [])],
            'actions': [{k: a.get(k) for k in ('id', 'label', 'hint', 'needs_name', 'confirm')}
                        for a in c.get('actions', [])],
            'maps': webconfig.maps(c['maps_dir']), 'default_map': c.get('default_map'),
            'drive': c.get('drive', {}),
            'dds': c.get('dds_effective', {}),
        }


# ---------------------------------------------------------------------- Tornado
class PageHandler(tornado.web.RequestHandler):
    def initialize(self, path):
        self.path = path

    def get(self):
        self.set_header('Cache-Control', 'no-store')
        with open(self.path, 'rb') as f:
            self.write(f.read())


class ConfigHandler(tornado.web.RequestHandler):
    def initialize(self, web):
        self.web = web

    def get(self):
        self.set_header('Cache-Control', 'no-store')
        self.write(self.web.client_config())


class LogHandler(tornado.web.RequestHandler):
    def initialize(self, web):
        self.web = web

    def get(self, id):
        if id not in self.web.items:
            raise tornado.web.HTTPError(404)
        self.set_header('Content-Type', 'text/plain; charset=utf-8')
        self.set_header('Cache-Control', 'no-store')
        self.write(self.web.pm.tail(id, 300) or '(noch kein Log)')


class Socket(tornado.websocket.WebSocketHandler):
    # check_origin bleibt wie in tornado: nur Seiten vom selben Host dürfen sich verbinden
    def initialize(self, web):
        self.web = web
        self.driving = False

    def open(self):
        self.web.clients.add(self)
        self.write_message(json.dumps(self.web.status()))

    async def on_message(self, data):
        try:
            msg = json.loads(data)
            await self.web.handle(self, msg)
        except ValueError as e:                             # inkl. JSON-Fehler
            self.write_message(json.dumps({'t': 'msg', 'level': 'error', 'text': str(e)}))
        except Exception as e:                              # nie den Server wegen einer Nachricht verlieren
            log.exception('Nachricht %r', data)
            self.write_message(json.dumps({'t': 'msg', 'level': 'error', 'text': f'Fehler: {e}'}))

    def on_close(self):
        self.web.clients.discard(self)
        if self.driving:
            self.web.bridge.drive_halt()                    # Verbindung weg → sofort anhalten


def make_app(web, www):
    page = os.path.join(www, 'index.html')
    return tornado.web.Application([
        (r'/', PageHandler, {'path': page}),
        (r'/handy', PageHandler, {'path': page}),
        (r'/api/config', ConfigHandler, {'web': web}),
        (r'/api/log/([a-z_]+)', LogHandler, {'web': web}),
        (r'/ws', Socket, {'web': web}),
        (r'/static/(.*)', tornado.web.StaticFileHandler, {'path': www}),
    ])


def main(argv=None):
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    share = get_package_share_directory('slop_web')
    ap = argparse.ArgumentParser(description='Slop-Bot Weboberfläche')
    ap.add_argument('--config', default=os.path.join(share, 'config', 'web.yaml'))
    args = ap.parse_args(argv)
    cfg = webconfig.load(args.config)

    # DDS-Einstellungen festlegen, BEVOR rclpy startet (gelten für den Webserver und alles, was er startet)
    cfg['dds_effective'], changes = webconfig.apply_dds(cfg.get('dds'), os.environ)
    for c in changes:
        log.warning('DDS-Einstellung aus web.yaml statt aus dem Terminal: %s', c)
    log.info('DDS: %s', ', '.join(f'{k}={v}' for k, v in cfg['dds_effective'].items()) or 'Standard')

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    d = cfg.get('drive', {})
    bridge = Bridge(DriveLimits(max_linear=d.get('max_linear', 0.18), max_angular=d.get('max_angular', 0.4)))
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    web = Web(cfg, bridge)
    server = tornado.httpserver.HTTPServer(make_app(web, os.path.join(share, 'www')))
    port, bound = int(cfg.get('port', 8080)), set()

    def bind():
        for addr in cfg.get('listen', ['127.0.0.1']):
            addr = tailscale_ip() if addr == 'tailscale' else addr
            if addr and addr not in bound:
                try:
                    server.listen(port, address=addr)
                    bound.add(addr)
                    log.info('Weboberfläche: http://%s:%d   (Handy: http://%s:%d/handy)', addr, port, addr, port)
                except OSError as e:
                    log.warning('Kann nicht auf %s:%d lauschen: %s', addr, port, e)

    bind()
    loop = tornado.ioloop.IOLoop.current()
    tornado.ioloop.PeriodicCallback(web.tick, 500).start()
    tornado.ioloop.PeriodicCallback(lambda: web.broadcast(web.status()), 500).start()
    tornado.ioloop.PeriodicCallback(bind, 15000).start()   # Tailscale kam später hoch → nachholen

    def shutdown(*_):
        loop.add_callback_from_signal(loop.stop)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        loop.start()
    finally:
        bridge.drive_halt()
        bridge.send_drive()
        executor.shutdown()
        bridge.destroy_node()
        rclpy.try_shutdown()
        log.info('Weboberfläche beendet (gestartete Programme laufen weiter)')


if __name__ == '__main__':
    main()
