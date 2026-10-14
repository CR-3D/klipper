# Test harness: run klippy against fake mcus and talk to its API socket
#
# Copyright (C) 2026  CR-3D
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, time, json, socket, subprocess, signal, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..', 'src')
BUILD = os.environ.get('ERRSIM_BUILD', '/tmp/klipper_errsim')
KLIPPY = os.path.join(BUILD, 'klippy')
PY = os.path.join(BUILD, 'venv', 'bin', 'python')

class Harness:
    def __init__(self, name, cfg, mcus=('main',), start_mcus=None):
        self.name = name
        self.dir = os.path.join(BUILD, 'run', name)
        shutil.rmtree(self.dir, ignore_errors=True)
        os.makedirs(self.dir)
        self.procs = {}
        self.mcus = mcus
        for m in mcus:
            subprocess.check_call([PY, os.path.join(HERE, 'gen_dict.py'),
                                   SRC, self.path(m + '.dict'), m])
        for m in (mcus if start_mcus is None else start_mcus):
            self.start_mcu(m)
        with open(self.path('printer.cfg'), 'w') as f:
            f.write(cfg.replace('@DIR@', self.dir))
        self.uds = '/tmp/kl_uds_%s_%d' % (name, os.getpid())
        self.klippy = self.sock = None
        self.reqid = 0
        self.buf = b""
        self.responses = []
    def path(self, n):
        return os.path.join(self.dir, n)
    def link(self, m):
        return '/tmp/klipper_host_fake_%s_%d' % (m, os.getpid())
    def start_mcu(self, m):
        ctl = self.path(m + '.ctl')
        self.procs[m] = subprocess.Popen(
            [PY, os.path.join(HERE, 'fakemcu.py'), KLIPPY,
             self.path(m + '.dict'), self.link(m), ctl])
        for i in range(50):
            if os.path.exists(self.link(m)):
                break
            time.sleep(0.05)
    def ctl(self, m, line):
        fd = os.open(self.path(m + '.ctl'), os.O_WRONLY)
        os.write(fd, (line + "\n").encode())
        os.close(fd)
    def flog(self, m):
        with open(self.link(m) + '.log') as f:
            return f.read()
    def start_klippy(self):
        self.klippy = subprocess.Popen(
            [PY, os.path.join(KLIPPY, 'klippy.py'), self.path('printer.cfg'),
             '-l', self.path('klippy.log'), '-a', self.uds],
            stdout=subprocess.DEVNULL,
            stderr=open(self.path('stderr.txt'), 'w'))
        self.connect()
    def connect(self):
        if self.sock is not None:
            self.sock.close()
        for i in range(300):
            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(self.uds)
                break
            except OSError:
                time.sleep(0.1)
        self.sock.settimeout(0.2)
        self.buf = b""
        self.request("gcode/subscribe_output", {"response_template": {}})
    def _read(self, timeout):
        end = time.time() + timeout
        msgs = []
        while time.time() < end:
            try:
                data = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                raise OSError("api socket closed")
            self.buf += data
            while b'\x03' in self.buf:
                m, self.buf = self.buf.split(b'\x03', 1)
                msg = json.loads(m)
                msgs.append(msg)
                if 'params' in msg and 'response' in msg['params']:
                    self.responses.append(msg['params']['response'])
            if any(m.get('id') == self.reqid for m in msgs):
                break
        return msgs
    def request(self, method, params=None, timeout=10.):
        self.reqid += 1
        req = {'id': self.reqid, 'method': method, 'params': params or {}}
        self.sock.sendall(json.dumps(req).encode() + b'\x03')
        for msg in self._read(timeout):
            if msg.get('id') == self.reqid:
                return msg
        return None
    def gcode(self, script, timeout=30.):
        return self.request("gcode/script", {"script": script}, timeout)
    def query(self, objs):
        r = self.request("objects/query", {"objects": {o: None for o in objs}})
        return r['result']['status']
    def info(self):
        return self.request("info")['result']
    def wait_state(self, state, timeout=20.):
        end = time.time() + timeout
        i = None
        while time.time() < end:
            try:
                i = self.info()
            except (OSError, TypeError):
                time.sleep(0.3)
                self.connect()
                continue
            if i['state'] == state:
                return i
            time.sleep(0.2)
        raise Exception("state %s not reached: %s" % (state, i))
    def pump(self, t):
        self._read(t)
    def log(self):
        with open(self.path('klippy.log')) as f:
            return f.read()
    def stop(self):
        if self.klippy is not None:
            self.klippy.send_signal(signal.SIGTERM)
            try:
                self.klippy.wait(5)
            except Exception:
                self.klippy.kill()
        for p in self.procs.values():
            p.kill()
        for m in self.mcus:
            try:
                os.unlink(self.link(m))
            except OSError:
                pass
