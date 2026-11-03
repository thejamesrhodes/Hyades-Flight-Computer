"""
fc_link.py -- the FC-V4 USB link, independent of any GUI (pyserial only).

    StreamDemux   bytes -> text lines, 0xAA55 binary frames, LOGDATA captures
    parse_status  the 5 Hz text status line -> dict
    parse_event   "[EVT] t=.. ms PHASE: text" -> dict
    Board         one background thread owns the port: finds 0483:5740, opens
                  it, reconnects after a reset/reflash, and hands everything
                  it decodes to a callback. Worker threads use expect()/
                  capture() for the fw and log-pull protocols.

Firmware references: Core/Src/console.c (line editor, echo), app.c (status
line, events, commands), telemetry.h (binary frames), flashlog.c (pull),
fwupdate.c (fw protocol).
"""
from __future__ import annotations

import binascii
import queue
import re
import struct
import threading
import time

VID, PID = 0x0483, 0x5740
PHASES = ("BOOT", "CAL", "SETTLE", "READY", "BOOST", "COAST", "DESCENT", "LANDED", "LOCKOUT", "CATO")
IN_FLIGHT = ("BOOST", "COAST", "DESCENT")             # Phase_InFlight(): firmware refuses fw/reboot/erase
ADVISORY = ("gps_link",)                             # SEV_INFO in fdir.c: reported, never acted on
MONITORS = ("imu_read", "accel_rng", "gyro_rng", "loop_time", "overrun", "est_nan", "est_tilt", "gps_link")
COMMANDS = ("help", "ver", "stream on", "stream off", "faults", "tasks", "gps", "imu",
            "log info", "log start", "log stop", "log erase", "log pull", "reboot",
            "inject wdg", "inject phase", "inject imu")
VER_PREFIX = "Hyades FC-V4"

# Binary STATE frame (telemetry.h): AA 55 id len payload crc16(id,len,payload)
SYNC0, SYNC1, MSG_STATE = 0xAA, 0x55, 0x01
STATE = struct.Struct("<II4f3f3f3f3f3f4fHH")         # 104 bytes
STATE_FIELDS = ("t_us", "frame", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw", "wx", "wy", "wz",
                "ax", "ay", "az", "mx", "my", "mz", "bx", "by", "bz", "f0", "f1", "f2", "f3",
                "health", "loop_us")
MAX_LINE = 1024


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def num(tok: str) -> float:
    """Con_Fx output ('+1.25', '-0.3', 'nan', '+ovf') -> float."""
    try:
        return float(tok)
    except ValueError:
        return float("inf") if "ovf" in tok else float("nan")


def fault_names(mask: int) -> list[str]:
    return [n for i, n in enumerate(MONITORS) if (mask >> i) & 1]


# ---------------------------------------------------------------------------
# Line parsers
# ---------------------------------------------------------------------------
_STATUS = re.compile(
    r"t=(?P<t>\d+\.\d+) (?P<phase>\w+) (?P<mode>\w+) f=(?P<fa>[0-9A-F]+)/(?P<fl>[0-9A-F]+) "
    r"RPY (?P<roll>\S+) (?P<pitch>\S+) (?P<yaw>\S+) w (?P<wx>\S+) (?P<wy>\S+) (?P<wz>\S+) "
    r"\|a\|=(?P<an>\S+) fin (?P<f0>\S+) (?P<f1>\S+) (?P<f2>\S+) (?P<f3>\S+) "
    r"imu=(?P<imu>\w+)\((?P<imu_fails>\d+)\) loop=(?P<loop>\d+) ovr=(?P<ovr>\d+) "
    r"gps=(?P<gps>\w+) s(?P<sats>\d+) alt=(?P<alt>\S+) rx=(?P<gps_rx>\d+) nmea=(?P<nmea>\d+) "
    r"log=(?P<log>\d+)%")
_EVENT = re.compile(r"\[EVT\] t=(?P<ms>\d+) ms (?P<phase>\w+): (?P<text>.*)$")
_BUILD = re.compile(r"build (\S+)")
_FLOATS = ("roll", "pitch", "yaw", "wx", "wy", "wz", "an", "f0", "f1", "f2", "f3", "alt")
_INTS = ("imu_fails", "loop", "ovr", "sats", "gps_rx", "nmea", "log")


def parse_status(line: str) -> dict | None:
    m = _STATUS.search(line)              # search: tolerate echoed text in front
    if not m:
        return None
    d = m.groupdict()
    out = {"t": float(d["t"]), "phase": d["phase"], "mode": d["mode"], "imu": d["imu"], "gps": d["gps"],
           "active": int(d["fa"], 16), "latched": int(d["fl"], 16)}
    out.update({k: num(d[k]) for k in _FLOATS})
    out.update({k: int(d[k]) for k in _INTS})
    return out


def parse_event(line: str) -> dict | None:
    m = _EVENT.search(line)
    if not m:
        return None
    return {"ms": int(m["ms"]), "phase": m["phase"], "text": m["text"]}


def parse_build(line: str) -> str | None:
    """Build id from a `ver` reply or the boot banner."""
    if VER_PREFIX not in line:
        return None
    m = _BUILD.search(line)
    return m.group(1).rstrip(",") if m else None


def decode_state(payload: bytes) -> dict:
    return dict(zip(STATE_FIELDS, STATE.unpack(payload)))


# ---------------------------------------------------------------------------
# Byte stream -> events
# ---------------------------------------------------------------------------
class StreamDemux:
    """Split the CDC byte stream into text lines and binary frames.

    Text is 7-bit ASCII, so 0xAA can only start a frame; a candidate that
    fails its CRC is dropped one byte at a time (resync). After arm_capture(),
    a "LOGDATA <n>" line switches to raw capture of exactly n bytes (the log
    image contains every byte value, so it must bypass the frame scanner).
    feed() returns a list of ("line", str) / ("frame", id, payload) /
    ("capture", bytes) events.
    """

    def __init__(self, resync: bool = False) -> None:
        self.resync = resync               # drop the tail of a line already in flight at open
        self.buf = bytearray()
        self.line = bytearray()
        self.armed = False
        self.cap_want = -1                 # >= 0 while capturing
        self.cap = bytearray()
        self.bad_frames = 0

    def arm_capture(self) -> None:
        self.armed, self.cap_want, self.cap = True, -1, bytearray()

    def disarm(self) -> None:
        self.armed, self.cap_want = False, -1

    def capture_progress(self) -> tuple[int, int]:
        return len(self.cap), max(self.cap_want, 0)

    def flush(self) -> list:
        """Emit a pending unterminated line (called when the link goes idle)."""
        if self.line and self.cap_want < 0:
            s = self._text(self.line)
            self.line.clear()
            return [("line", s)] if s else []
        return []

    @staticmethod
    def _text(b: bytes) -> str:
        return b.decode("ascii", "replace").replace("\r", "").rstrip()

    def feed(self, data: bytes) -> list:
        ev: list = []
        self.buf += data
        b = self.buf
        if self.resync:
            nl = b.find(b"\n")
            if nl < 0:
                b.clear()
                return ev
            del b[:nl + 1]
            self.resync = False
        while b:
            if self.cap_want >= 0:                              # raw capture
                take = min(self.cap_want - len(self.cap), len(b))
                self.cap += b[:take]
                del b[:take]
                if len(self.cap) >= self.cap_want:
                    ev.append(("capture", bytes(self.cap)))
                    self.cap_want, self.armed = -1, False
                continue
            nl, aa = b.find(b"\n"), b.find(SYNC0)
            if aa >= 0 and (nl < 0 or aa < nl):                 # frame candidate first
                self.line += b[:aa]
                del b[:aa]
                if len(b) < 2 or (b[1] == SYNC1 and len(b) < 4):
                    break                                       # wait for the header
                if b[1] != SYNC1:
                    del b[0]
                    self.bad_frames += 1
                    continue
                n = 4 + b[3] + 2
                if len(b) < n:
                    break
                body = bytes(b[2:n - 2])
                if crc16(body) == struct.unpack_from("<H", b, n - 2)[0]:
                    ev.append(("frame", body[0], body[2:]))
                    del b[:n]
                else:
                    del b[0]
                    self.bad_frames += 1
            elif nl >= 0:
                self.line += b[:nl]
                del b[:nl + 1]
                s = self._text(self.line)
                self.line.clear()
                if s:
                    ev.append(("line", s))
                    if self.armed and s.startswith("LOGDATA "):
                        try:
                            self.cap_want, self.cap = int(s.split()[1]), bytearray()
                        except (IndexError, ValueError):
                            self.armed = False
                        if self.cap_want == 0:
                            ev.append(("capture", b""))
                            self.cap_want, self.armed = -1, False
            else:
                self.line += b
                b.clear()
                if len(self.line) > MAX_LINE:                   # runaway: show it anyway
                    ev.extend(self.flush())
        return ev


# ---------------------------------------------------------------------------
# Link thread
# ---------------------------------------------------------------------------
def find_port() -> str | None:
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def open_serial(port: str):
    import serial
    s = serial.Serial(port, 115200, timeout=0.05, write_timeout=5.0)
    s.dtr = True                       # firmware only transmits to an open host
    return s


class Board:
    """Owns the port. on_event(kind, *args) is called on the link thread:

        ("link", state, detail)   state: "waiting" | "connected" | "busy"
        ("line", text)            one received text line
        ("frame", msg_id, payload)
        ("capture", bytes)        a completed LOGDATA payload
    """

    def __init__(self, on_event, port: str | None = None, finder=find_port, opener=open_serial):
        self.on_event = on_event
        self.fixed_port = port
        self._find = (lambda: port) if port else finder
        self._open = opener
        self._ser = None
        self._wlock = threading.Lock()
        self._stop = threading.Event()
        self.connected = threading.Event()
        self.port_name: str | None = None
        self.rx_bytes = self.tx_bytes = 0
        self.demux = StreamDemux(resync=True)
        self._proto: queue.Queue = queue.Queue()
        self._proto_on = threading.Event()
        self._cap: queue.Queue = queue.Queue()
        self._thread = threading.Thread(target=self._run, name="fc-link", daemon=True)
        self._thread.start()

    # -- thread ---------------------------------------------------------------
    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._close(quiet=True)

    def _close(self, quiet: bool = False) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:                                   # noqa: BLE001
                pass
        self._ser = None
        was = self.connected.is_set()
        self.connected.clear()
        self.port_name = None
        self.demux = StreamDemux(resync=True)
        if was and not quiet:
            self.on_event("link", "waiting", "board disconnected")

    def _run(self) -> None:
        last_rx = time.monotonic()
        announced = None
        while not self._stop.is_set():
            if self._ser is None:
                port = None
                try:
                    port = self._find()
                except Exception:                               # noqa: BLE001
                    pass
                if not port:
                    if announced != "waiting":
                        self.on_event("link", "waiting", "no board (USB 0483:5740)")
                        announced = "waiting"
                    time.sleep(0.3)
                    continue
                try:
                    self._ser = self._open(port)
                except Exception as exc:                        # noqa: BLE001
                    # "Access is denied" = another program holds the port. Anything
                    # else (port vanished mid re-enumeration) is transient: keep waiting.
                    if "denied" in str(exc).lower() or "busy" in str(exc).lower():
                        if announced != "busy":
                            self.on_event("link", "busy", f"{port}: in use by another program")
                            announced = "busy"
                    time.sleep(0.5)
                    continue
                self.port_name = port
                self.connected.set()
                announced = "connected"
                self.on_event("link", "connected", port)
            try:
                data = self._ser.read(4096)
            except Exception:                                   # noqa: BLE001
                self._close()
                announced = None
                time.sleep(0.3)
                continue
            now = time.monotonic()
            if data:
                last_rx = now
                self.rx_bytes += len(data)
                events = self.demux.feed(data)
            elif now - last_rx > 0.25:
                events = self.demux.flush()
            else:
                events = []
            for e in events:
                if e[0] == "line" and self._proto_on.is_set():
                    self._proto.put(e[1])
                elif e[0] == "capture":
                    self._cap.put(e[1])
                self.on_event(*e)

    # -- writing --------------------------------------------------------------
    def write_raw(self, data: bytes) -> bool:
        ser = self._ser
        if ser is None:
            return False
        try:
            with self._wlock:
                ser.write(data)
                ser.flush()
            self.tx_bytes += len(data)
            return True
        except Exception:                                       # noqa: BLE001
            return False

    def send_line(self, cmd: str) -> bool:
        return self.write_raw(cmd.encode("ascii", "replace") + b"\r")

    # -- protocol helpers (worker threads only) -------------------------------
    def proto_begin(self) -> None:
        while True:
            try:
                self._proto.get_nowait()
            except queue.Empty:
                break
        self._proto_on.set()

    def proto_end(self) -> None:
        self._proto_on.clear()

    def expect(self, prefixes: tuple[str, ...], timeout: float) -> str | None:
        """First line containing one of the prefixes (echo may precede it)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self._proto.get(timeout=0.05)
            except queue.Empty:
                if not self.connected.is_set():
                    return None
                continue
            for p in prefixes:
                i = line.find(p)
                if i >= 0:
                    return line[i:]
        return None

    def arm_capture(self) -> None:
        while not self._cap.empty():
            self._cap.get_nowait()
        self.demux.arm_capture()

    def wait_capture(self, timeout: float, progress=None) -> bytes | None:
        """Wait for the armed capture; gives up if no byte arrives for `timeout` s."""
        last, t_last = -1, time.monotonic()
        while True:
            try:
                return self._cap.get(timeout=0.1)
            except queue.Empty:
                pass
            got, want = self.demux.capture_progress()
            if got != last:
                last, t_last = got, time.monotonic()
                if progress and want:
                    progress(got, want)
            elif time.monotonic() - t_last > timeout or not self.connected.is_set():
                self.demux.disarm()
                return None

    def wait_disconnect(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self.connected.is_set():
                return True
            time.sleep(0.05)
        return False

    def wait_connect(self, timeout: float) -> bool:
        return self.connected.wait(timeout)
