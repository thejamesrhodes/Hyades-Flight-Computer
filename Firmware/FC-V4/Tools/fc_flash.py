#!/usr/bin/env python3
"""
fc_flash.py -- flash the FC-V4 over its USB port (no ST-LINK).

    python fc_flash.py                       # Debug/Flight Controller V4.elf
    python fc_flash.py --elf path/to.elf --port COM8

Builds the flash image from the ELF's loadable segments (no objcopy/.bin
needed), sends `fw <size> <crc32>`, streams the image, and waits for the board
to reprogram itself and re-enumerate. Firmware side: Core/Src/fwupdate.c.

Recovery: a flashed image that does not boot leaves nothing to talk to over
USB -- reflash over SWD (STM32CubeIDE, or
  STM32_Programmer_CLI -c port=SWD -w "Flight Controller V4.elf" -v -rst ).
Requires: pip install pyserial
"""
from __future__ import annotations

import argparse
import os
import struct
import time
import zlib

import fc_common as fc

APP_BASE, APP_SIZE = 0x08000000, 128 * 1024
BUILD_ID_OFFSET = 0x198                    # .build_id right after the F411 vector table


def elf_to_image(path: str) -> bytes:
    """Concatenate PT_LOAD segments by physical (load) address into one image."""
    data = open(path, "rb").read()
    if data[:4] != b"\x7fELF" or data[4] != 1 or data[5] != 1:
        fc.die(f"{path}: not a 32-bit little-endian ELF")
    e_phoff, = struct.unpack_from("<I", data, 28)
    e_phentsize, e_phnum = struct.unpack_from("<HH", data, 42)
    segs = []
    for i in range(e_phnum):
        p_type, p_offset, _vaddr, p_paddr, p_filesz = struct.unpack_from("<IIIII", data, e_phoff + i * e_phentsize)
        if p_type == 1 and p_filesz > 0 and APP_BASE <= p_paddr < APP_BASE + APP_SIZE:
            segs.append((p_paddr, data[p_offset:p_offset + p_filesz]))
    if not segs:
        fc.die("no loadable flash segments in the ELF")
    end = max(a + len(b) for a, b in segs)
    img = bytearray(b"\xff" * (end - APP_BASE))
    for a, b in segs:
        img[a - APP_BASE:a - APP_BASE + len(b)] = b
    return bytes(img)


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", default=os.path.join(here, "..", "Debug", "Flight Controller V4.elf"))
    ap.add_argument("--port", help="serial port (default: find 0483:5740)")
    args = ap.parse_args()
    fc.require_serial()

    img = elf_to_image(args.elf)
    crc = zlib.crc32(img) & 0xFFFFFFFF
    build = img[BUILD_ID_OFFSET:BUILD_ID_OFFSET + 32].split(b"\0")[0].decode("ascii", "replace")
    port = fc.find_port(args.port) or fc.die("board not found (USB connected? COM port?)")
    print(f"port  : {port}")
    print(f"image : {len(img)} bytes  crc32={crc:08x}  build '{build}'")

    with fc.open_port(port) as ser:
        fc.quiet(ser)
        fc.send_cmd(ser, f"fw {len(img)} {crc:08x}")
        line = fc.wait_for(ser, ("FWREADY", "FWABORT"), 5.0)
        if not line or not line.startswith("FWREADY"):
            fc.die(f"board refused: {line or 'no reply'}")
        print("<- " + line)
        t0 = time.monotonic()
        for i in range(0, len(img), 4096):
            ser.write(img[i:i + 4096])
        ser.flush()
        print(f"sent {len(img)} bytes in {time.monotonic() - t0:.2f} s")
        for want in ("FWRECV", "FWCRC", "FWVEC", "FWPROG"):
            line = fc.wait_for(ser, (want, "FWABORT"), 10.0)
            print("<- " + (line or "(timeout)"))
            if not line or line.startswith("FWABORT") or " bad" in line:
                fc.die("update aborted; the old firmware is still intact")

    print("programming... waiting for the board to come back")
    time.sleep(2.0)
    port = fc.wait_port(args.port, 40.0) or fc.die("board did not re-enumerate: check it, recover over SWD")
    time.sleep(1.0)
    with fc.open_port(port) as ser:
        fc.quiet(ser)
        fc.send_cmd(ser, "ver")
        line = fc.wait_for(ser, ("Hyades FC-V4",), 5.0, echo=False)
        print("<- " + (line or "(no reply to 'ver')"))
        fc.send_cmd(ser, "stream on")
        if line and build and build not in line:
            fc.die("running build does not match the image")
    print("done")


if __name__ == "__main__":
    main()
