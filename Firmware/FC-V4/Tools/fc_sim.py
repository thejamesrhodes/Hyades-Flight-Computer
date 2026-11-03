"""
fc_sim.py -- a software stand-in for the FC-V4 USB console.

Speaks the same protocol as the firmware (echo, status line, [EVT] events,
help/ver/stream/faults/tasks/gps/imu/log/fw/reboot/inject, LOGDATA pull with
real 48-byte records, the fw update handshake, USB detach on reset) so the
console GUI can be tried and tested without a board:

    python fc_console.py --sim

Not a flight model: attitude is a slow wobble, nothing here validates GNC.
"""
from __future__ import annotations

import binascii
import math
import struct
import threading
import time
import zlib

from fc_link import MONITORS, PHASES, STATE, SYNC0, SYNC1, MSG_STATE, MSG_IMU
import random

BUILD = "sim0000"
LOG_CAP = 384 * 1024
REC = 48


def _fx(v: float, dp: int) -> str:
    return "nan" if v != v else f"{v:+.{dp}f}"


class SimDevice:
    """Board state; outlives a simulated USB detach (reboot, reflash)."""

    def __init__(self, binary: bool = False, boot_s: float = 2.5) -> None:
        self.binary = binary
        self.boot_s = boot_s
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        self.out = bytearray()
        self.present = True
        self.port: SimPort | None = None
        self.log = bytearray()
        self.logging = False
        self.reset_cause = "POR"
        self.builds = BUILD
        self.fw_writes = 0
        self._boot()
        threading.Thread(target=self._tick, name="fc-sim", daemon=True).start()

    # -- state ------------------------------------------------------------------
    def _boot(self) -> None:
        self.t0 = time.monotonic()
        self.phase = "BOOT"
        self.mode = "OFF"
        self.active = self.latched = 0
        self.stream = True
        self.line = ""
        self.fw_want = -1
        self.fw_buf = bytearray()
        self.fw_crc = 0
        self.fw_deadline = 0.0
        self.imu_fail = False
        self.ovr = 0
        self.frame = 0
        self.banner_sent = False
        self.settle_t = 0.0
        self.capture = False

    def _emit(self, s: str | bytes) -> None:
        """Caller holds the lock."""
        if self.port is None:
            return
        self.out += s.encode() if isinstance(s, str) else s
        self.cv.notify_all()

    def _say(self, s: str) -> None:
        self._emit(s + "\r\n")

    def _event(self, text: str) -> None:
        ms = int((time.monotonic() - self.t0) * 1000)
        self._say(f"[EVT] t={ms} ms {self.phase}: {text}")
        if self.logging:
            r = bytearray(REC)
            r[0], r[1], r[2] = 0xE5, 1, PHASES.index(self.phase)
            struct.pack_into("<I", r, 4, ms * 1000)
            r[20:20 + min(len(text), 25)] = text.encode()[:25]
            self._log_rec(r)

    def _go(self, ph: str) -> None:
        if ph != self.phase:
            old, self.phase = self.phase, ph
            self._event(f"{old}->{ph}")

    def _log_rec(self, r: bytearray) -> None:
        if len(self.log) + REC > LOG_CAP:
            self.logging = False
            return
        struct.pack_into("<H", r, 46, binascii.crc_hqx(bytes(r[:46]), 0xFFFF))
        self.log += r

    def detach(self, down_s: float = 1.5, cause: str = "SOFT") -> None:
        """USB goes away (reset); it comes back after down_s."""
        self.present = False
        if self.port:
            self.port.dead = True
            self.port = None

        def back():
            time.sleep(down_s)
            with self.lock:
                self.reset_cause = cause
                self._boot()
                self.present = True
        threading.Thread(target=back, daemon=True).start()

    # -- 100 Hz-ish tick ------------------------------------------------------
    def _tick(self) -> None:
        last_status = 0.0
        while True:
            time.sleep(0.02)
            with self.lock:
                if self.port is None or not self.present:
                    continue
                if self.fw_want >= 0 and time.monotonic() > self.fw_deadline:
                    self._say(f"FWABORT timeout after {len(self.fw_buf)} of {self.fw_want} bytes")
                    self.fw_want = -1
                if not self.banner_sent:
                    self.banner_sent = True
                    self._say(f"\r\n=== Hyades FC-V4 (F411 96 MHz) build {self.builds}, reset: {self.reset_cause} ===")
                    self._say(f"[BOOT] log {len(self.log)}/{LOG_CAP} B used. Type 'help' for commands.")
                    self._say("[BOOT] GPS probe: PA10 rx1 float 0e | PA15 tx1 float 0e | PA3 rx2 high 1843e ~9600 | PA2 tx2 float 0e")
                    self._say("[BOOT] GPS port1 9600 Bd 0 B, port2 9600 Bd 0 B, 0 NMEA -> module transmitting")
                    self._event("boot")
                    self._go("CAL")
                    self._say("[BOOT] calibrating gyro - keep still")
                t = time.monotonic() - self.t0
                if self.phase == "CAL" and t > self.boot_s * 0.4:
                    self._say("[BOOT] gyro bias dps +0.120 -0.034 +0.051 | seed RPY deg +0.4 -1.1 +0.0")
                    self._go("SETTLE")
                if self.phase == "SETTLE" and t > self.boot_s and time.monotonic() - self.settle_t > 1.0:
                    self._go("READY")
                    self.mode = "FULL"
                if self.imu_fail and self.phase not in ("LOCKOUT", "CATO"):
                    self.active |= 1
                    self.latched |= 1
                    self._event("TRIP imu_read")
                    self._go("LOCKOUT")
                    self.mode = "OFF"
                self.frame += 2
                if self.capture:
                    self._capture_tick()
                    continue
                if self.fw_want >= 0:
                    continue                                    # firmware is busy receiving
                roll, pitch, yaw = 3 * math.sin(t * 0.7), 2 * math.sin(t * 0.4), (t * 5) % 360 - 180
                fins = [0.0] * 4 if self.mode == "OFF" else [0.2 * math.sin(t * 1.3 + k) for k in range(4)]
                if self.logging and self.frame % 2 == 0:
                    r = bytearray(REC)
                    r[0], r[1], r[2], r[3] = 0x51, PHASES.index(self.phase), ("OFF", "RDAMP", "FULL").index(self.mode), 3
                    struct.pack_into("<I", r, 4, int(t * 1e6))
                    struct.pack_into("<4h", r, 10, 32767, 0, 0, 0)
                    struct.pack_into("<3h", r, 18, int(roll * 10), int(pitch * 10), 0)
                    struct.pack_into("<3h", r, 24, 0, 0, 981)
                    struct.pack_into("<4b", r, 36, *[int(f * 127) for f in fins])
                    struct.pack_into("<HH", r, 40, 910, self.active)
                    self._log_rec(r)
                if not self.stream:
                    continue
                if self.binary:
                    pay = STATE.pack(int(t * 1e6), self.frame, 1, 0, 0, 0, roll, pitch, yaw,
                                     0.5, -0.2, 5.0, 0, 0, 9.81, 0, 0, 0, 0, 0, 0, *fins,
                                     0x0003 | (2 << 10), 910)
                    body = bytes([MSG_STATE, len(pay)]) + pay
                    self._emit(bytes([SYNC0, SYNC1]) + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF)))
                elif time.monotonic() - last_status >= 0.2:
                    last_status = time.monotonic()
                    gps = "none"
                    self._say(
                        f"t={int(t)}.{int(t * 100) % 100:02d} {self.phase} {self.mode} f={self.active:02X}/{self.latched:02X} "
                        f"RPY {_fx(roll, 1)} {_fx(pitch, 1)} {_fx(yaw, 1)} w {_fx(0.5, 1)} {_fx(-0.2, 1)} {_fx(5.0, 1)} "
                        f"|a|={_fx(1.0, 2)} fin {' '.join(_fx(f, 2) for f in fins)} "
                        f"imu={'FAIL' if self.imu_fail else 'OK'}({int(self.imu_fail)}) loop=910 ovr={self.ovr} "
                        f"gps={gps} s0 alt=+0.0 rx=0 nmea=0 log={len(self.log) * 100 // LOG_CAP}%")

    # -- 'imu capture': synthetic white + random-walk gyro, white accel, 16-bit counts
    SIM_ODR = 224.2
    SIM_GYRO_N = 2.3e-4        # rad/s/sqrt(Hz)
    SIM_ACC_N = 1.5e-3         # m/s^2/sqrt(Hz)

    def _capture_tick(self) -> None:
        now = time.monotonic()
        due = int((now - self.cap_t0) * self.SIM_ODR) - self.cap_n
        g_lsb = 16.0                                      # LSB/dps at +/-2048 dps
        a_lsb = 2048.0                                    # LSB/g at +/-16 g
        sg = self.SIM_GYRO_N * (self.SIM_ODR ** 0.5) * 57.29578 * g_lsb
        sa = self.SIM_ACC_N * (self.SIM_ODR ** 0.5) / 9.80665 * a_lsb
        for _ in range(max(0, due)):
            self.cap_n += 1
            self.cap_rw += random.gauss(0.0, 0.002)
            g = [int(round(random.gauss(5.0 + self.cap_rw, sg))) for _ in range(3)]
            a = [int(round(random.gauss(0.0, sa))), int(round(random.gauss(0.0, sa))),
                 int(round(random.gauss(a_lsb, sa)))]
            pay = struct.pack("<Ih3h3h", self.cap_n & 0xFFFFFF, 25 * 256, *a, *g)
            body = bytes([MSG_IMU, len(pay)]) + pay
            self._emit(bytes([SYNC0, SYNC1]) + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF)))

    # -- input ----------------------------------------------------------------
    def rx(self, data: bytes) -> None:
        with self.lock:
            if self.fw_want >= 0:
                take = data[:self.fw_want - len(self.fw_buf)]
                self.fw_buf += take
                self.fw_deadline = time.monotonic() + 5.0
                if len(self.fw_buf) >= self.fw_want:
                    self._fw_done()
                return
            for c in data.decode("ascii", "replace"):
                if c in "\r\n":
                    if self.line:
                        cmd, self.line = self.line, ""
                        self._emit("\r\n")
                        self._dispatch(cmd.split())
                elif c in "\b\x7f":
                    if self.line:
                        self.line = self.line[:-1]
                        self._emit("\b \b")
                elif " " <= c < "\x7f":
                    self.line += c
                    self._emit(c)
                if self.fw_want >= 0:
                    return

    def _fw_done(self) -> None:
        img = bytes(self.fw_buf)
        self.fw_want = -1
        self._say(f"FWRECV {len(img)}")
        got = zlib.crc32(img) & 0xFFFFFFFF
        if got != self.fw_crc:
            self._say(f"FWCRC bad got={got:08X} want={self.fw_crc:08X}")
            return
        self._say("FWCRC ok")
        sp, rv = struct.unpack_from("<II", img, 0)
        if not (0x20000000 <= sp <= 0x20020000 and 0x08000000 <= rv < 0x08020000):
            self._say("FWVEC bad sp/reset")
            return
        self._say("FWVEC ok")
        self._say("FWPROG")
        build = img[0x198:0x198 + 32].split(b"\0")[0].decode("ascii", "replace")
        self.builds = build or BUILD
        self.fw_writes += 1
        threading.Timer(0.3, lambda: self.detach(2.0, "SOFT")).start()

    def _dispatch(self, a: list[str]) -> None:
        cmd = a[0]
        arg = a[1] if len(a) > 1 else ""
        flying = self.phase in ("BOOST", "COAST", "DESCENT")
        if cmd == "help":
            for n, h in (("help", "this list"), ("ver", "build id"), ("stream", "[on|off] periodic status"),
                         ("faults", "FDIR monitors"), ("tasks", "per-task timing"), ("gps", "GNSS detail"),
                         ("imu", "IMU diagnostics"), ("log", "[info|start|stop|erase|pull] flight log"),
                         ("fw", "<size> <crc32hex> USB firmware update"), ("reboot", "reset"),
                         ("inject", "wdg|phase|imu bench fault injection")):
                self._say(f"  {n:<7} {h}")
        elif cmd == "ver":
            self._say(f"Hyades FC-V4 (STM32F411, QMI8658A) build {self.builds} 12:00:00, last reset: {self.reset_cause}")
        elif cmd == "stream":
            if arg:
                self.stream = arg != "off"
            self._say(f"stream {'on' if self.stream else 'off'}")
        elif cmd == "faults" and arg == "clear":
            if flying or self.phase == "CATO":
                self._say("ERR not in flight")
                return
            self.imu_fail = False
            self.active = self.latched = 0
            if self.phase == "LOCKOUT":
                self.phase = "SETTLE"
                self.settle_t = time.monotonic()
            self._event("faults cleared (operator)")
            self._say("faults clear: OK")
        elif cmd == "faults":
            self._say(f"phase {self.phase}  ctrl {self.mode}  latch {self.latched:08X}")
            self._say("  monitor    act lat  trips  count  last_r")
            for i, n in enumerate(MONITORS):
                self._say(f"  {n:<10} {(self.active >> i) & 1}   {(self.latched >> i) & 1}       0      0  +0.000")
        elif cmd == "tasks":
            self._say("  task      period  last_us  wcet_us   (frame 10000 us, budget 8000 us)")
            for n, us in (("imu", 310), ("est", 280), ("fdir", 12), ("ctrl", 90), ("log", 40), ("telem", 60)):
                self._say(f"  {n:<9} {1:5d}  {us:7d}  {us + 40:7d}")
            self._say(f"  overruns {self.ovr}, usb tx dropped 0 B, rx dropped 0 B")
        elif cmd == "gps" and arg == "raw":
            self._say("  $GNGGA,120000.000,,,,,0,00,25.5,,,,,,*7A")
            self._say("  $GNRMC,120000.000,V,,,,,,,031126,,,N,V*2C")
        elif cmd == "gps" and arg == "probe":
            self._say("GPS probe: PA10 rx1 float 0e | PA15 tx1 float 0e | PA3 rx2 high 1843e ~9600 | PA2 tx2 float 0e")
            self._say("GPS port1 9600 Bd 0 B, port2 9600 Bd 0 B, 0 NMEA -> module transmitting")
        elif cmd == "gps":
            self._say("gps fix=0 sats=0 hdop=+99.9 alt=+0.0 m utc=000000  lat/lon(x1e4)=+0,+0")
            self._say("    bytes=0 nmea=0 gga=0 rmc=0 bad=0 overrun=0")
        elif cmd == "imu" and arg == "capture":
            if self.capture:
                self._say("ERR capture running ('reboot' to end)")
                return
            self.capture, self.cap_t0, self.cap_n, self.cap_rw = True, time.monotonic(), 0, 0.0
            self.stream = False
            self._say("IMUCAP v1 odr_mHz=224200 a_lsb_per_g=2048 g_lsb_per_dps_x1000=16000 "
                      "ctrl=40 35 75 00 77 00 03 build=sim0000 (binary IMU frames follow; 'reboot' to end)")
        elif cmd == "imu":
            self._say("imu QMI8658A whoami=0x05 rev=0x7C reads=1234 i2c_err=0 torn=0/0 stale=0")
        elif cmd == "log":
            if arg == "start":
                self.logging = len(self.log) + REC <= LOG_CAP
                if self.logging:
                    r = bytearray(REC)
                    r[0] = 0xB7
                    struct.pack_into("<H", r, 10, 50)
                    r[12:12 + len(self.builds)] = self.builds.encode()[:32]
                    self._log_rec(r)
                self._say(f"log start: {'OK' if self.logging else 'FAILED (full? log erase)'}")
                if self.logging:
                    self._event("log start")
            elif arg == "stop":
                self._event("log stop")
                self.logging = False
                self._say("log stop: OK")
            elif arg == "erase":
                if flying:
                    self._say("ERR not in flight")
                    return
                self._say("log erase: erasing 384 KB (~3-6 s, fins centred)...")
                self.logging = False

                def done():
                    time.sleep(1.5)
                    with self.lock:
                        self.log = bytearray()
                        self._say("log erase: OK")
                        self._event("log erased")
                threading.Thread(target=done, daemon=True).start()
            elif arg == "pull":
                data = bytes(self.log)
                self._say(f"LOGDATA {len(data)}")
                self._emit(data)
                self._say(f"LOGEND {zlib.crc32(data) & 0xFFFFFFFF:08X}")
            else:
                self._say(f"log {'active' if self.logging else 'idle'}, {len(self.log)}/{LOG_CAP} B used "
                          f"({len(self.log) // REC} records), 0 write errors")
        elif cmd == "fw":
            if flying:
                self._say("FWABORT in-flight")
                return
            try:
                size, crc = int(a[1]), int(a[2], 16)
            except (IndexError, ValueError):
                self._say("FWABORT bad-size")
                return
            if not 1024 <= size <= 96 * 1024:
                self._say(f"FWABORT size-out-of-range {size} (max {96 * 1024})")
                return
            self.logging = False
            self.fw_want, self.fw_crc, self.fw_buf = size, crc, bytearray()
            self.fw_deadline = time.monotonic() + 5.0
            self._say(f"FWREADY {size}")
        elif cmd == "reboot":
            if flying:
                self._say("ERR not in flight")
                return
            self._say("rebooting")
            threading.Timer(0.2, lambda: self.detach(1.5, "SOFT")).start()
        elif cmd == "inject":
            if arg == "wdg":
                self._say("inject wdg: stalling the frame (expect watchdog reset)")
                threading.Timer(0.2, lambda: self.detach(1.5, "IWDG")).start()
            elif arg == "phase":
                self._say("inject phase: flipped one bit of the phase word")
                self._event("phase word corrupt")
                self._go("LOCKOUT")
                self.mode = "OFF"
            elif arg == "imu":
                self._say("inject imu: forcing IMU read failures")
                self.imu_fail = True
            else:
                self._say("usage: inject wdg|phase|imu")
        else:
            self._say(f"ERR unknown command '{cmd}' (help)")


class SimPort:
    """Just enough of serial.Serial for fc_link.Board."""

    def __init__(self, dev: SimDevice) -> None:
        self.dev = dev
        self.dead = False
        self.dtr = True

    def read(self, n: int) -> bytes:
        d = self.dev
        with d.cv:
            if self.dead:
                raise OSError("device detached")
            if not d.out:
                d.cv.wait(0.05)
            if self.dead:
                raise OSError("device detached")
            out = bytes(d.out[:n])
            del d.out[:n]
            return out

    def write(self, data: bytes) -> int:
        if self.dead:
            raise OSError("device detached")
        self.dev.rx(bytes(data))
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.dead = True
        with self.dev.lock:
            if self.dev.port is self:
                self.dev.port = None


def make(binary: bool = False, boot_s: float = 2.5):
    """(finder, opener, device) for fc_link.Board."""
    dev = SimDevice(binary=binary, boot_s=boot_s)

    def finder():
        return "SIM" if dev.present else None

    def opener(_port):
        if not dev.present or dev.port is not None:
            raise OSError("busy")
        p = SimPort(dev)
        with dev.lock:
            dev.port = p
            dev.out = bytearray()
        return p
    return finder, opener, dev
