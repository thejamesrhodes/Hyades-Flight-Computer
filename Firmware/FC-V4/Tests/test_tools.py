"""Host tests for the USB tools (stdlib unittest; no board, no GUI).

    python3 Tests/test_tools.py

Covers fc_link (stream demux, parsers) and runs the link + protocol code
end to end against fc_sim: connect, command echo, log start/pull/decode,
firmware update handshake with a real ELF image, reset and reconnect.
"""
import binascii
import json
import os
import struct
import sys
import tempfile
import time
import unittest
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Tools"))
import fc_link as link                                          # noqa: E402
import fc_sim                                                   # noqa: E402

STATUS = ("t=12.34 READY FULL f=00/40 RPY +1.2 -0.3 +45.0 w +0.1 -0.2 +0.3 |a|=+1.00 "
          "fin +0.10 -0.20 +0.00 nan imu=OK(0) loop=910 ovr=2 gps=none s0 alt=+0.0 rx=0 nmea=0 log=3%")


def frame(payload: bytes, msg=link.MSG_STATE) -> bytes:
    body = bytes([msg, len(payload)]) + payload
    return b"\xaa\x55" + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


class Parsers(unittest.TestCase):
    def test_status(self):
        st = link.parse_status(STATUS)
        self.assertEqual((st["phase"], st["mode"], st["latched"]), ("READY", "FULL", 0x40))
        self.assertAlmostEqual(st["yaw"], 45.0)
        self.assertTrue(st["f3"] != st["f3"])                   # 'nan' -> NaN
        self.assertEqual((st["loop"], st["ovr"], st["log"]), (910, 2, 3))
        self.assertEqual(link.fault_names(st["latched"]), ["est_tilt"])
        self.assertIsNone(link.parse_status("phase READY  ctrl FULL"))

    def test_event_build(self):
        ev = link.parse_event("[EVT] t=5120 ms SETTLE: SETTLE->READY")
        self.assertEqual(ev, {"ms": 5120, "phase": "SETTLE", "text": "SETTLE->READY"})
        self.assertEqual(link.parse_build("=== Hyades FC-V4 (F411 96 MHz) build 1a2b3c+, reset: POR ==="), "1a2b3c+")
        self.assertEqual(link.parse_build("Hyades FC-V4 (STM32F411, QMI8658A) build abc 12:00:00, last reset: SOFT"),
                         "abc")
        self.assertEqual(link.num("+ovf"), float("inf"))


class FaultSummary(unittest.TestCase):
    def test_priority_and_no_stuck_states(self):
        bit = {n: 1 << i for i, n in enumerate(link.MONITORS)}
        self.assertEqual(link.fault_summary(0, 0, "READY")[:2], ("none", "ok"))
        self.assertEqual(link.fault_summary(bit["gps_link"], 0, "READY")[:2], ("gps_link advisory", "info"))
        t, s, full = link.fault_summary(bit["gps_link"] | bit["est_tilt"], 0, "SETTLE")
        self.assertEqual((t, s), ("est_tilt active  +1 more", "warn"))            # SOFT before INFO
        t, s, full = link.fault_summary(bit["gps_link"] | bit["est_tilt"], bit["imu_read"], "LOCKOUT")
        self.assertEqual(s, "bad")
        self.assertTrue(t.startswith("imu_read latched"))                         # HARD first
        self.assertEqual(full[0], "imu_read latched")
        a = bit["imu_read"]
        self.assertTrue(link.fault_summary(a, a, "LOCKOUT")[0].startswith("imu_read LATCHED"))
        # LOCKOUT with nothing latched (watchdog reset / phase upset) is never "none".
        self.assertEqual(link.fault_summary(0, 0, "LOCKOUT")[1], "bad")
        self.assertEqual(link.fault_summary(0, 0xFFFFFFFF, "READY")[:2], ("LATCH CORRUPT", "bad"))
        # overrun is HARD on the pad, SOFT in flight.
        self.assertEqual(link.fault_summary(bit["overrun"], 0, "COAST")[1], "warn")


class Demux(unittest.TestCase):
    def test_lines_split_anywhere(self):
        d = link.StreamDemux()
        data = b"hello\r\nwor" + b"ld\r\n"
        ev = []
        for i in range(len(data)):                              # one byte at a time
            ev += d.feed(data[i:i + 1])
        self.assertEqual(ev, [("line", "hello"), ("line", "world")])

    def test_frames_interleaved_and_resync(self):
        pay = link.STATE.pack(*([1, 2] + [0.5] * 23 + [3, 910]))
        good = frame(pay)
        bad = bytearray(good)
        bad[10] ^= 0xFF                                         # corrupt -> dropped, stream resyncs
        d = link.StreamDemux()
        ev = d.feed(b"ab" + good + b"cd\r\n" + bytes(bad) + good[:7])
        ev += d.feed(good[7:] + b"ef\r\n")
        kinds = [e[0] for e in ev]
        self.assertEqual(kinds.count("frame"), 2)
        self.assertIn(("line", "abcd"), ev)
        self.assertEqual(link.decode_state(ev[0][2])["loop_us"], 910)
        self.assertGreater(d.bad_frames, 0)

    def test_capture_binary_payload(self):
        blob = bytes(range(256)) * 3 + b"\n\xaa\x55\x01"       # every byte value, incl. sync and newline
        d = link.StreamDemux()
        d.arm_capture()
        msg = f"LOGDATA {len(blob)}\r\n".encode() + blob + b"LOGEND 0000\r\n"
        ev = []
        for i in range(0, len(msg), 37):
            ev += d.feed(msg[i:i + 37])
        self.assertEqual(ev[0], ("line", f"LOGDATA {len(blob)}"))
        self.assertEqual(ev[1], ("capture", blob))
        self.assertEqual(ev[2], ("line", "LOGEND 0000"))
        # Unarmed, the same header is just a line.
        self.assertEqual(link.StreamDemux().feed(b"LOGDATA 3\r\n"), [("line", "LOGDATA 3")])


class EndToEnd(unittest.TestCase):
    """fc_link.Board against the simulator: the same calls fc_console makes."""

    def setUp(self):
        finder, opener, self.dev = fc_sim.make(boot_s=0.5)
        self.lines = []
        self.links = []

        def on_event(kind, *a):
            if kind == "line":
                self.lines.append(a[0])
            elif kind == "link":
                self.links.append(a[0])
        self.b = link.Board(on_event, finder=finder, opener=opener)
        self.assertTrue(self.b.wait_connect(3))

    def tearDown(self):
        self.b.stop()

    def ask(self, cmd, prefixes, timeout=3.0):
        self.b.proto_begin()
        self.b.send_line(cmd)
        r = self.b.expect(prefixes, timeout)
        self.b.proto_end()
        return r

    def test_status_and_commands(self):
        time.sleep(1.2)
        self.assertTrue(any(link.parse_status(s) for s in self.lines))
        self.assertTrue(any(link.parse_event(s) for s in self.lines))
        self.assertIn("build sim0000", self.ask("ver", (link.VER_PREFIX,)))
        self.assertTrue(self.ask("bogus", ("ERR",)).startswith("ERR unknown"))
        self.ask("inject imu", ("inject imu",))
        time.sleep(0.2)
        self.assertEqual(self.dev.phase, "LOCKOUT")
        self.assertEqual(self.ask("faults clear", ("faults clear:", "ERR")), "faults clear: OK")
        self.assertEqual(self.dev.phase, "SETTLE")
        self.assertIn("bogus", self.lines)                      # firmware echo arrives as its own line

    def test_log_pull_decode(self):
        import fc_pull
        self.ask("log start", ("log start:",))
        time.sleep(0.6)
        self.ask("log stop", ("log stop:",))
        self.ask("stream off", ("stream off",))
        self.b.arm_capture()
        self.b.proto_begin()                                    # once: LOGEND may follow at once
        self.b.send_line("log pull")
        n = int(self.b.expect(("LOGDATA",), 3.0).split()[1])
        raw = self.b.wait_capture(2.0)
        self.assertEqual(len(raw), n)
        trailer = self.b.expect(("LOGEND",), 2.0)
        self.assertEqual(int(trailer.split()[1], 16), zlib.crc32(raw) & 0xFFFFFFFF)
        out = tempfile.mkdtemp()
        fc_pull.decode(raw, out)
        with open(os.path.join(out, "session1_state.csv")) as fh:
            self.assertGreater(len(fh.readlines()), 5)

    def test_imu_capture_to_noise_fit(self):
        """'imu capture' -> MSG_IMU frames -> .imu/.json (fc_imucap format) -> fc_noise recovers the white level."""
        try:
            import numpy  # noqa: F401
        except ImportError:
            self.skipTest("numpy not installed")
        import fc_imucap
        import fc_noise
        frames = []
        self.b.on_event = lambda kind, *a: frames.append(a[1]) if kind == "frame" and a[0] == link.MSG_IMU else None
        line = self.ask("imu capture", ("IMUCAP", "ERR"))
        self.assertTrue(line.startswith("IMUCAP"), line)
        meta = fc_imucap.parse_imucap(line)
        self.assertEqual((meta["gyro_fs_dps"], meta["accel_fs_g"], meta["lpf_pct_odr"]), (2048, 16, 13.37))
        time.sleep(6.0)
        self.assertEqual(self.ask("imu capture", ("ERR",)), "ERR capture running ('reboot' to end)")
        stem = os.path.join(tempfile.mkdtemp(), "cap")
        with open(stem + ".imu", "wb") as fh:
            fh.write(b"".join(frames))
        with open(stem + ".json", "w") as fh:
            json.dump(meta, fh)
        _meta, ts, _t, _a, g = fc_noise.load(stem)
        self.assertGreater(len(ts), 1000)
        c = fc_noise.analyse_channel(g[:, 0] / meta["g_lsb_per_dps"] * fc_noise.D2R, meta["odr_hz"],
                                     fc_noise.D2R / meta["g_lsb_per_dps"])
        self.assertAlmostEqual(c["N_meas"] / fc_sim.SimDevice.SIM_GYRO_N, 1.0, delta=0.15)

    def test_flash_and_reconnect(self):
        import fc_flash
        elf = os.path.join(os.path.dirname(__file__), "..", "Debug", "Flight Controller V4.elf")
        if not os.path.isfile(elf):
            self.skipTest("no Debug ELF")
        img = fc_flash.elf_to_image(elf)
        build = img[fc_flash.BUILD_ID_OFFSET:fc_flash.BUILD_ID_OFFSET + 32].split(b"\0")[0].decode()
        crc = zlib.crc32(img) & 0xFFFFFFFF
        self.assertTrue(self.ask(f"fw {len(img)} {crc:08x}", ("FWREADY", "FWABORT")).startswith("FWREADY"))
        self.b.proto_begin()
        for i in range(0, len(img), 4096):
            self.assertTrue(self.b.write_raw(img[i:i + 4096]))
        for want in ("FWRECV", "FWCRC ok", "FWVEC ok", "FWPROG"):
            self.assertIsNotNone(self.b.expect((want,), 3.0), want)
        self.assertTrue(self.b.wait_disconnect(3))
        self.assertTrue(self.b.wait_connect(6))
        self.assertIn(f"build {build}", self.ask("ver", (link.VER_PREFIX,)))
        # A corrupt image is refused before programming.
        self.assertTrue(self.ask(f"fw {len(img)} 00000000", ("FWREADY",)).startswith("FWREADY"))
        self.b.proto_begin()
        self.b.write_raw(img)
        self.assertIn("bad", self.b.expect(("FWCRC",), 3.0))
        self.assertEqual(self.dev.fw_writes, 1)



class Hitl(unittest.TestCase):
    """HITL-01 host side: wire formats match hitl.h, plant matches OpenRocket."""

    def test_frames_round_trip_through_demux(self):
        import fc_hitl as hl
        self.assertEqual(hl.SAMPLE.size, 32)
        self.assertEqual(hl.REPLY.size, 60)
        payload = hl.REPLY.pack(7, 3, 2, 0x80, 321, 1, 0, *([0.1] * 4), 1.0, 0, 0, 0, 12.5, -1.0, 30.0, 550.0)
        frm = hl.frame(hl.MSG_REPLY, payload)
        d = link.StreamDemux()
        ev = d.feed(b"text\r\n" + frm[:5]) + d.feed(frm[5:])
        self.assertEqual(ev[0], ("line", "text"))
        self.assertEqual(ev[1][0:2], ("frame", hl.MSG_REPLY))
        r = dict(zip(hl.REPLY_FIELDS, hl.REPLY.unpack(ev[1][2])))
        self.assertEqual((r["idx"], r["phase"], r["loop_us"]), (7, 3, 321))
        self.assertAlmostEqual(r["qbar"], 550.0)

    def test_plant_reproduces_openrocket(self):
        import numpy as np
        import hitl_model as hm
        P, _src = hm.load_params()
        m = hm.load_openrocket(os.path.join(hm.SIM_DIR, "or_sim1_G79W.csv"))
        v = hm.Vehicle(m, P, hm.SimCfg(site_elev=46.0, use_fins=False, dt=0.002))
        v.ignite()
        while v.t_apogee is None and v.t < 30:
            v.frame(np.zeros(4))
        self.assertLess(abs(v.apogee - m.ref["apogee"]) / m.ref["apogee"], 0.02)
        self.assertLess(abs(v.t_apogee - m.ref["t_apogee"]), 0.3)

if __name__ == "__main__":
    unittest.main(verbosity=2)
