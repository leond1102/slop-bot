"""Startet und stoppt Launch-Dateien wie im Terminal – ohne ROS, damit testbar.

Start: bash lädt die ROS-Umgebung und ersetzt sich durch den Befehl (eigene Sitzung, Ausgabe direkt in eine Logdatei).
Stopp wie Strg+C im Terminal: SIGINT an alle eigenen Prozesse des Baums, nach stop_term s SIGTERM, nach stop_kill s SIGKILL.

Grundsatz: Es werden nur Prozesse angefasst, die von hier gestartet wurden (Nachfahren des gestarteten Prozesses oder
in seiner Sitzung) und dem eigenen Nutzer gehören. Root-Prozesse (sudo) beenden sich, wenn ihr Kind endet. Im Terminal
gestartete Programme werden nur erkannt („extern“), nie beendet.

Der Zustand liegt in state_file: Startet der Webserver neu, übernimmt er laufende Prozesse wieder (PID + Startzeit
müssen passen, sonst war es eine wiederverwendete PID).
"""
import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field

CLK_TCK = os.sysconf('SC_CLK_TCK')


# ---------------------------------------------------------------------- /proc
def proc_stat(pid, proc='/proc'):
    """(ppid, sid, starttime) oder None. Startzeit in Ticks seit dem Booten (bleibt je Prozess gleich)."""
    try:
        with open(f'{proc}/{pid}/stat', 'rb') as f:
            data = f.read().decode(errors='replace')
    except OSError:
        return None
    rest = data[data.rindex(')') + 2:].split()      # Name kann Leerzeichen und Klammern enthalten
    # rest[0] = state, [1] = ppid, [2] = pgrp, [3] = session, ... [19] = starttime
    if rest[0] == 'Z':
        return None                                  # Zombie: läuft nicht mehr
    return int(rest[1]), int(rest[3]), int(rest[19])


def proc_uid(pid, proc='/proc'):
    try:
        return os.stat(f'{proc}/{pid}').st_uid
    except OSError:
        return None


def proc_cmdline(pid, proc='/proc'):
    try:
        with open(f'{proc}/{pid}/cmdline', 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='replace').strip()
    except OSError:
        return ''


def all_pids(proc='/proc'):
    return [int(p) for p in os.listdir(proc) if p.isdigit()]


def snapshot(proc='/proc'):
    """{pid: (ppid, sid)} aller laufenden Prozesse."""
    out = {}
    for pid in all_pids(proc):
        st = proc_stat(pid, proc)
        if st:
            out[pid] = (st[0], st[1])
    return out


def tree(root, sids, snap):
    """Prozesse, die zu einem Start gehören: root, seine Nachfahren und alles in den bekannten Sitzungen."""
    children = {}
    for pid, (ppid, _) in snap.items():
        children.setdefault(ppid, []).append(pid)
    found, todo = set(), [root] if root in snap else []
    while todo:
        p = todo.pop()
        if p not in found:
            found.add(p)
            todo.extend(children.get(p, []))
    found |= {pid for pid, (_, sid) in snap.items() if sid in sids}
    return found


# ---------------------------------------------------------------------- Prozesse
@dataclass
class Run:
    id: str
    pid: int
    starttime: int
    log: str
    started: float                              # time.time()
    cmd: str
    sids: set = field(default_factory=set)
    stop_since: float = None                    # time.monotonic() beim Stopp-Befehl
    sent: str = ''                              # zuletzt gesendetes Signal
    exit_code: int = None
    ended: float = None
    popen: object = None                        # nur für selbst gestartete (nicht übernommene) Prozesse
    members: set = field(default_factory=set)   # zuletzt gefundene Prozesse (tick)
    reported: bool = False                      # Ende schon gemeldet (Webseite)

    def to_json(self):
        return {'id': self.id, 'pid': self.pid, 'starttime': self.starttime, 'log': self.log,
                'started': self.started, 'cmd': self.cmd, 'sids': sorted(self.sids)}


class ProcessManager:
    def __init__(self, log_dir, state_file, setup_scripts, stop_term=15.0, stop_kill=25.0, proc='/proc'):
        self.log_dir, self.state_file = log_dir, state_file
        self.setup_scripts = setup_scripts
        self.stop_term, self.stop_kill = stop_term, stop_kill
        self.proc = proc
        self.uid = os.getuid()
        self.runs = {}                          # id → Run (laufend oder gerade beendet)
        self.external = {}                      # Muster → PIDs fremder Starts (tick)
        os.makedirs(log_dir, exist_ok=True)
        os.makedirs(os.path.dirname(state_file), exist_ok=True)
        self._adopt()

    # ---------------- Start
    def start(self, id, argv, env):
        if self.is_running(id):
            raise RuntimeError(f'{id} läuft schon')
        log = os.path.join(self.log_dir, f'{id}-{time.strftime("%Y%m%d-%H%M%S")}.log')
        setup = ' && '.join(f'source {s}' for s in self.setup_scripts)
        script = f'{setup} && exec "$@"' if setup else 'exec "$@"'
        with open(log, 'ab') as out:
            out.write(f'# {time.strftime("%Y-%m-%d %H:%M:%S")}  {" ".join(argv)}\n'.encode())
            out.flush()
            p = subprocess.Popen(['bash', '-c', script, 'slop_web', *argv], stdin=subprocess.DEVNULL, stdout=out,
                                 stderr=subprocess.STDOUT, env=env, cwd=env.get('HOME', '/'), start_new_session=True)
        st = proc_stat(p.pid, self.proc)
        run = Run(id, p.pid, st[2] if st else 0, log, time.time(), ' '.join(argv), {p.pid}, popen=p, members={p.pid})
        self.runs[id] = run
        self._save()
        return run

    # ---------------- Stopp
    def stop(self, id):
        run = self.runs.get(id)
        if run is None or not self.is_running(id):
            return False
        if run.stop_since is None:
            run.stop_since = time.monotonic()
            self._signal(run, signal.SIGINT)
        return True

    def tick(self, patterns=()):
        """Regelmäßig aufrufen (≈ 2 Hz): Prozesse erfassen, Sitzungen merken, Stopp eskalieren, Beendete erkennen,
        fremde Starts zu den Mustern suchen."""
        snap = snapshot(self.proc)
        changed = False
        for run in list(self.runs.values()):
            if run.ended is not None:
                continue
            members = self._members(run, snap)
            run.members = members
            run.sids |= {snap[p][1] for p in members if p in snap}
            if not members:
                run.ended = time.time()
                run.exit_code = run.popen.poll() if run.popen else None
                changed = True
                continue
            if run.stop_since is not None:
                dt = time.monotonic() - run.stop_since
                if dt > self.stop_kill and run.sent != 'SIGKILL':
                    self._signal(run, signal.SIGKILL, snap)
                elif dt > self.stop_term and run.sent == 'SIGINT':
                    self._signal(run, signal.SIGTERM, snap)
        own = self.own_pids()
        self.external = {pat: self.find_external(pat, own, snap) for pat in patterns}
        if changed:
            self._save()

    def _alive_root(self, run):
        st = proc_stat(run.pid, self.proc)
        return st is not None and st[2] == run.starttime

    def _members(self, run, snap):
        root = run.pid if self._alive_root(run) else -1
        return tree(root, run.sids, snap)

    def _signal(self, run, sig, snap=None):
        snap = snap if snap is not None else snapshot(self.proc)
        own = [p for p in self._members(run, snap) if proc_uid(p, self.proc) == self.uid]
        for p in own:
            try:
                os.kill(p, sig)
            except (ProcessLookupError, PermissionError):
                pass
        run.sent = signal.Signals(sig).name
        return own

    # ---------------- Abfragen
    def is_running(self, id):
        """Stand des letzten tick (gerade gestartete Prozesse gelten bis dahin als laufend)."""
        run = self.runs.get(id)
        return run is not None and run.ended is None and bool(run.members)

    def state(self, id):
        """'aus' | 'läuft' | 'stoppt' | 'beendet' (+ Run)."""
        run = self.runs.get(id)
        if run is None:
            return 'aus', None
        if run.ended is not None:
            return 'beendet', run
        return ('stoppt' if run.stop_since is not None else 'läuft'), run

    def own_pids(self):
        pids = set()
        for run in self.runs.values():
            if run.ended is None:
                pids |= run.members
        return pids

    def find_external(self, pattern, exclude, snap=None):
        """PIDs fremder Starts (z. B. im Terminal), deren Befehlszeile pattern enthält."""
        out = []
        for pid in (snap if snap is not None else all_pids(self.proc)):
            if pid in exclude:
                continue
            cmd = proc_cmdline(pid, self.proc)
            if pattern in cmd and ('ros2 launch' in cmd or 'ros2 run' in cmd):
                out.append(pid)
        return out

    def tail(self, id, lines=200):
        run = self.runs.get(id)
        if run is None or not os.path.exists(run.log):
            return ''
        with open(run.log, 'rb') as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 200 * lines))
            data = f.read().decode(errors='replace')
        return '\n'.join(data.splitlines()[-lines:])

    # ---------------- Zustand
    def _save(self):
        data = {id: r.to_json() for id, r in self.runs.items() if r.ended is None}
        tmp = self.state_file + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(data, f)
        os.replace(tmp, self.state_file)

    def _adopt(self):
        try:
            with open(self.state_file, encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        snap = snapshot(self.proc)
        for id, d in data.items():
            run = Run(id, d['pid'], d['starttime'], d['log'], d['started'], d['cmd'], set(d.get('sids', [])))
            run.members = self._members(run, snap)
            if run.members:
                self.runs[id] = run
        self._save()
