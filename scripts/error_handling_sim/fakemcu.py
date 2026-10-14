#!/usr/bin/env python3
# Minimal fake klipper mcu on a pty (test helper)
#
#  fakemcu.py <klippy dir> <dict file> <link path> <control fifo>
#
# Control commands (one per line, written to the fifo):
#   adc <pin> <value 0..1>    set the adc reading of a pin
#   i2c_fail <n>              fail the next n i2c transfers (START_NACK)
#   unplug                    close the pty (mcu disappears)
#   plug                      create a new pty (mcu powered up again)
#   shutdown                  firmware shutdown "Timer too close"
import sys, os, time, json, zlib, select, tty, struct, errno, logging

sys.path.insert(0, sys.argv[1])
import msgproto

FREQ = 72000000

class FakeMCU:
    def __init__(self, dict_file, link, ctl):
        self.raw_dict = open(dict_file, 'rb').read()
        self.zdict = zlib.compress(self.raw_dict)
        self.mp = msgproto.MessageParser()
        self.mp.process_identify(self.raw_dict, decompress=False)
        self.link, self.ctl = link, ctl
        self.start = time.monotonic()
        self.master = None
        self.adc_values = {}
        self.i2c_fail = 0
        self.log = open(link + ".log", "w")
        if not os.path.exists(ctl):
            os.mkfifo(ctl)
        self.ctl_fd = os.open(ctl, os.O_RDONLY | os.O_NONBLOCK)
        self.ctl_buf = b""
        self.reset_state()
        self.plug()
    def note(self, msg):
        self.log.write("%.3f %s\n" % (time.monotonic() - self.start, msg))
        self.log.flush()
    def reset_state(self):
        self.boot = time.monotonic()
        self.is_config = 0
        self.crc = 0
        self.shutdown_id = None
        self.next_seq = 0
        self.inbuf = b""
        self.oid_pins = {}
        self.analog = {}
        self.i2c_regs = {}
    def clock(self):
        return int((time.monotonic() - self.boot) * FREQ)
    # pty handling
    def plug(self):
        master, slave = os.openpty()
        tty.setraw(slave)
        self.master, self.slave = master, slave
        try:
            os.unlink(self.link)
        except OSError:
            pass
        os.symlink(os.ttyname(slave), self.link)
        self.reset_state()
        self.note("plugged %s" % (os.ttyname(slave),))
    def unplug(self):
        if self.master is None:
            return
        try:
            os.unlink(self.link)
        except OSError:
            pass
        os.close(self.master)
        os.close(self.slave)
        self.master = None
        self.note("unplugged")
    # Sending
    def send(self, name, **params):
        mf = self.mp.messages_by_name[name]
        for pname, t in mf.param_names:
            if isinstance(t, msgproto.Enumeration) and isinstance(
                    params.get(pname), str):
                pass
        cmd = mf.encode_by_name(**params)
        self.send_block(cmd)
    def send_block(self, cmd):
        if self.master is None:
            return
        out = [5 + len(cmd), 0x10 | self.next_seq] + list(cmd)
        out += msgproto.crc16_ccitt(out)
        block = out + [0x7e]
        try:
            os.write(self.master, bytes(bytearray(block)))
        except OSError:
            pass
    def do_shutdown(self, reason):
        if self.shutdown_id is not None:
            return
        self.shutdown_id = reason
        self.note("shutdown: %s" % (reason,))
        self.send("shutdown", clock=self.clock() & 0xffffffff,
                  static_string_id=reason)
    # Receiving
    def handle_data(self, data):
        self.inbuf += data
        while 1:
            ret = self.mp.check_packet(bytearray(self.inbuf))
            if ret == 0:
                return
            if ret < 0:
                # resync
                pos = self.inbuf.find(b'\x7e')
                self.inbuf = b"" if pos < 0 else self.inbuf[pos+1:]
                continue
            block = bytearray(self.inbuf[:ret])
            self.inbuf = self.inbuf[ret:]
            seq = block[1] & 0x0f
            if seq != self.next_seq:
                self.send_block([])   # nak
                continue
            self.next_seq = (seq + 1) & 0x0f
            self.process_block(block)
            self.send_block([])   # ack (always sent after the responses)
    def process_block(self, block):
        pos = 2
        sent = False
        self.out = []
        while pos < len(block) - 3:
            msgid, ppos = self.mp.msgid_parser.parse(block, pos)
            mf = self.mp.messages_by_id.get(msgid)
            if mf is None:
                self.note("unknown msgid %d" % (msgid,))
                break
            params, pos = mf.parse(block, pos)
            self.handle_cmd(mf.name, params)
        for name, params in self.out:
            self.send(name, **params)
            sent = True
        return sent
    def reply(self, name, **params):
        self.out.append((name, params))
    IN_SHUTDOWN = {'identify', 'get_uptime', 'get_clock', 'get_config',
                   'emergency_stop', 'clear_shutdown', 'config_reset',
                   'reset', 'get_canbus_id'}
    def handle_cmd(self, name, p):
        if name not in ('get_clock',):
            self.note("cmd %s %s" % (name, p))
        if self.shutdown_id is not None and name not in self.IN_SHUTDOWN:
            self.reply("is_shutdown", static_string_id=self.shutdown_id)
            return
        if name == 'identify':
            off, cnt = p['offset'], p['count']
            self.reply("identify_response", offset=off,
                       data=self.zdict[off:off+cnt])
        elif name == 'get_uptime':
            c = self.clock()
            self.reply("uptime", high=c >> 32, clock=c & 0xffffffff)
        elif name == 'get_clock':
            self.reply("clock", clock=self.clock() & 0xffffffff)
        elif name == 'get_config':
            self.reply("config", is_config=self.is_config, crc=self.crc,
                       is_shutdown=int(self.shutdown_id is not None),
                       move_count=200)
        elif name == 'finalize_config':
            self.is_config = 1
            self.crc = p['crc']
        elif name == 'emergency_stop':
            self.do_shutdown("Command request")
        elif name == 'clear_shutdown':
            self.shutdown_id = None
        elif name == 'config_reset':
            if self.shutdown_id is None:
                self.do_shutdown("Command request")
                return
            seq = self.next_seq
            self.reset_state()
            self.next_seq = seq
            self.note("config_reset")
        elif name == 'reset':
            # Reboot: the pty stays, all state is lost
            self.note("reset")
            seq = self.next_seq
            self.reset_state()
            self.next_seq = seq
        elif name == 'config_analog_in':
            self.oid_pins[p['oid']] = p['pin']
        elif name == 'query_analog_in':
            oid = p['oid']
            self.analog[oid] = dict(p, next_time=time.monotonic() + 0.1,
                                    invalid=0)
        elif name == 'config_i2c':
            self.i2c_regs[p['oid']] = None
        elif name == 'i2c_transfer':
            self.handle_i2c(p)
        elif name == 'stepper_get_position':
            self.reply("stepper_position", oid=p['oid'], pos=0)
        elif name == 'neopixel_send':
            self.reply("neopixel_result", oid=p['oid'], success=1)
        elif name == 'get_canbus_status':
            self.reply("canbus_status", rx_error=0, tx_error=0, tx_retries=0,
                       canbus_bus_state="active")
        elif name == 'debug_read':
            # stm32g0 temperature calibration (TS_CAL1 / TS_CAL2)
            vals = {0x1FFF75A8: 1030, 0x1FFF75CA: 1370}
            self.reply("debug_result", val=vals.get(p['addr'], 0))
        elif name == 'endstop_query_state':
            self.reply("endstop_state", oid=p['oid'], homing=0, next_clock=0,
                       pin_value=0)
    # BME280 emulation
    BME_CAL1 = struct.pack('<HhhHhhhhhhhh', 27504, 26435, -1000, 36477,
                           -10685, 3024, 2855, 140, -7, 15500, -14600, 6000)
    def handle_i2c(self, p):
        oid = p['oid']
        if self.i2c_fail > 0:
            self.i2c_fail -= 1
            self.reply("i2c_response", oid=oid, i2c_bus_status="START_NACK",
                       response=b"")
            return
        write, rlen = bytearray(p['write']), p['read_len']
        resp = b""
        if rlen:
            reg = write[0]
            regs = {0xd0: b'\x60', 0xf3: b'\x00',
                    0x88: self.BME_CAL1 + b'\x00\x4b',
                    0xe1: b'\x00' * 16,
                    0xf7: bytes([0x65, 0x5a, 0xc0, 0x7e, 0xed, 0x00,
                                 0x80, 0x00])}
            resp = regs.get(reg, b'\x00' * rlen)[:rlen]
            resp += b'\x00' * (rlen - len(resp))
        self.reply("i2c_response", oid=oid, i2c_bus_status="SUCCESS",
                   response=resp)
    # Periodic analog reports
    def do_timers(self):
        now = time.monotonic()
        self.out = []
        for oid, a in self.analog.items():
            if a['sample_count'] == 0 or now < a['next_time']:
                continue
            a['next_time'] = now + a['rest_ticks'] / float(FREQ)
            pin = self.oid_pins.get(oid)
            value = self.adc_values.get(pin, 0.5)
            raw = int(value * 4095 * a['sample_count'])
            raw = max(0, min(0xffff, raw))
            if a['min_value'] <= raw <= a['max_value']:
                a['invalid'] = 0
            else:
                a['invalid'] += 1
                if a['invalid'] >= a['range_check_count']:
                    self.do_shutdown("ADC out of range")
            if self.shutdown_id is not None:
                continue
            nbytes = a['bytes_per_report'] // 2
            vals = struct.pack('<H', raw) * nbytes
            self.reply("analog_in_state", oid=oid,
                       next_clock=self.clock() & 0xffffffff, values=vals)
        for name, params in self.out:
            self.send(name, **params)
    def handle_ctl(self, line):
        self.note("ctl %s" % (line,))
        parts = line.split()
        if not parts:
            return
        if parts[0] == 'adc':
            self.adc_values[parts[1]] = float(parts[2])
        elif parts[0] == 'i2c_fail':
            self.i2c_fail = int(parts[1])
        elif parts[0] == 'unplug':
            self.unplug()
        elif parts[0] == 'plug':
            self.plug()
        elif parts[0] == 'shutdown':
            self.do_shutdown("Timer too close")
    def run(self):
        while 1:
            fds = [self.ctl_fd]
            if self.master is not None:
                fds.append(self.master)
            r, w, x = select.select(fds, [], [], 0.02)
            if self.master is not None and self.master in r:
                try:
                    data = os.read(self.master, 4096)
                except OSError as e:
                    data = b""
                if data:
                    self.handle_data(data)
            if self.ctl_fd in r:
                data = os.read(self.ctl_fd, 4096)
                if not data:
                    os.close(self.ctl_fd)
                    self.ctl_fd = os.open(self.ctl, os.O_RDONLY | os.O_NONBLOCK)
                self.ctl_buf += data
                while b"\n" in self.ctl_buf:
                    line, self.ctl_buf = self.ctl_buf.split(b"\n", 1)
                    self.handle_ctl(line.decode().strip())
            self.do_timers()

if __name__ == '__main__':
    FakeMCU(sys.argv[2], sys.argv[3], sys.argv[4]).run()
