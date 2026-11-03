"""Shared helpers for the FC-V4 USB tools (pyserial only)."""
from __future__ import annotations

import sys
import time

VID, PID = 0x0483, 0x5740          # ST VCP (USB CDC) as enumerated by the F411


def die(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(1)


def require_serial():
    try:
        import serial  # noqa: F401
    except ImportError:
        die("pyserial is required:  pip install pyserial")


def find_port(explicit: str | None) -> str | None:
    from serial.tools import list_ports
    if explicit:
        return explicit
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def wait_port(explicit: str | None, timeout_s: float) -> str | None:
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        p = find_port(explicit)
        if p:
            return p
        time.sleep(0.25)
    return None


def open_port(port: str):
    import serial
    s = serial.Serial(port, 115200, timeout=0.2)   # baud ignored by CDC
    s.dtr = True                                   # firmware sends only with DTR set
    return s


def send_cmd(ser, cmd: str) -> None:
    ser.write((cmd + "\r").encode("ascii"))
    ser.flush()


def read_line(ser, deadline: float) -> str | None:
    buf = bytearray()
    while time.monotonic() < deadline:
        c = ser.read(1)
        if not c:
            continue
        if c == b"\n":
            return buf.decode("ascii", "replace").strip()
        if c != b"\r":
            buf += c
    return None


def wait_for(ser, prefixes: tuple[str, ...], timeout_s: float, echo: bool = True) -> str | None:
    """Return the first line starting with any prefix; print others if echo."""
    deadline = time.monotonic() + timeout_s
    while True:
        line = read_line(ser, deadline)
        if line is None:
            return None
        for p in prefixes:
            idx = line.find(p)          # tolerate echoed command text before it
            if idx >= 0:
                return line[idx:]
        if echo and line:
            print(f"   {line}")


def quiet(ser) -> None:
    """Pause the periodic status stream and drop anything already buffered."""
    send_cmd(ser, "stream off")
    time.sleep(0.3)
    ser.reset_input_buffer()
