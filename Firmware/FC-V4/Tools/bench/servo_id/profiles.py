"""BENCH-02 servo identification: the command profiles, in one place.

The flight computer plays these from a table in flash ('bench run <name> <fin>', firmware built with
CFG_BENCH_PROFILE). This file defines them, writes that table, and replays them bit for bit for the
analysis, so the command behind every video frame is known exactly.

    python profiles.py              list the profiles
    python profiles.py --emit      rewrite Core/hyades/svc/bench_profiles.c (then rebuild and flash)

Units: a profile is written in degrees at the canard and seconds; the table holds PWM ticks about the
fin's zero (9600 ticks over 135 deg = 71.11 ticks/deg, 0.0141 deg/tick) and 200 Hz frames.

Timing of one run (frames):
    PRE  300   fins at 0, both board LEDs off
    RUN  N     frame i commands the profile; LEDs on while (i // 97) is even. N = 97 x an odd number that
               is different for every profile, so the LED starts on, ends on, and the count of half
               periods identifies the profile in a video.
    POST 300   fins at 0, LEDs off
The LED and the servo pulse for frame i change at the same instant (PWM_Commit).

Segment kinds (bench.h): MOVE to a target over b frames then hold; SINE about the current value;
TABLE of per-frame steps (the flight replay). All integer, so bench.c and play() agree exactly; the
CRC-32 of the played ticks is in the table, the firmware prints its own at the end of a run, and
test_bench.c checks one against the other.
"""
import math, os, sys, zlib

HERE = os.path.dirname(os.path.abspath(__file__))
FRAME_HZ = 200
TICKS_PER_DEG = 9600.0 / 135.0      # board.h: (SERVO_MAX_US - SERVO_MIN_US) x PWM_TICKS_PER_US / SERVO_RANGE_DEG
PRE, POST, LED_HALF = 300, 300, 97
DITHER_TICKS, DITHER_HALF = 12, 2   # flight dither: +/-1.5 us, 50 Hz square (CTRL_DITHER_*)
TICKS_MAX = 2133                    # 30 deg
MOVE, SINE, TABLE = 0, 1, 2
F_DITHER = 1
SIN_Q = [int(round(32767 * math.sin(0.5 * math.pi * k / 256))) for k in range(257)]


def T(deg):
    return int(round(deg * TICKS_PER_DEG))


def FR(sec):
    return int(round(sec * FRAME_HZ))


def hold(deg, sec, dither=False):
    """Step to deg and stay there for sec."""
    return (MOVE, F_DITHER if dither else 0, FR(sec), T(deg), 0)


def ramp(deg, sec, dither=False):
    """Straight line to deg, arriving after sec."""
    return (MOVE, F_DITHER if dither else 0, FR(sec), T(deg), FR(sec))


def sine(amp_deg, period, cycles, dither=False):
    """Sine about the current value: period in frames (frequency 200 / period Hz), whole cycles."""
    return (SINE, F_DITHER if dither else 0, period * cycles, T(amp_deg), period)


def flight_table():
    with open(os.path.join(HERE, "flight_cmd.csv")) as f:
        k = [int(l) for l in f if l.strip() and not l.startswith("#")]
    d = [b - a for a, b in zip([0] + k[:-1], k)]
    assert all(-128 <= v <= 127 for v in d)
    return d


TABLES = [flight_table()]

# Every profile but 'sync' starts with this: three known levels, used to tie the picture to angle.
HDR = [hold(0, 0.5), hold(10, 0.4), hold(-10, 0.4), hold(0, 0.5)]
SINE_PERIODS = [200, 100, 66, 40, 28, 20, 14, 10, 8, 6, 5]      # 1, 2, 3, 5, 7.1, 10, 14.3, 20, 25, 33.3, 40 Hz


def p_sync():
    return [hold(0, 4.0)]


def p_stair():
    fine = [1.5 * k for k in range(1, 9)] + [12 - 1.5 * k for k in range(1, 17)] + [-12 + 1.5 * k for k in range(1, 9)]
    coarse = [5 * k for k in range(1, 7)] + [30 - 5 * k for k in range(1, 13)] + [-30 + 5 * k for k in range(1, 7)]
    return HDR + [hold(a, 0.4) for a in fine + fine + coarse]


def p_creep(d):
    return HDR + [hold(0, 2.0, d), ramp(2, 5.0, d), ramp(-2, 10.0, d), ramp(0, 5.0, d), hold(0, 1.0, d)]


def p_steps():
    s = list(HDR)
    for size in (0.1, 0.2, 0.3, 0.5, 1, 2, 5, 10, 20, 30):
        h = 0.3 if size <= 2 else 0.4
        for _ in range(4 if size in (1, 5) else 2):
            s += [hold(size, h), hold(0, h), hold(-size, h), hold(0, h)]
    return s


def p_sine(amp, d):
    s = HDR + [hold(0, 0.3, d)]
    for n in SINE_PERIODS:
        s += [sine(amp, n, max(6, -(-240 // n)), d), hold(0, 0.2, d)]
    return s


def p_flight(d):
    return HDR + [hold(0, 0.5, d), (TABLE, F_DITHER if d else 0, len(TABLES[0]), 0, 0), hold(0, 0.5, d)]


# name, segments, what it is for. The order is the profile id; append, never reorder.
RAW = [
    ("sync",    p_sync(),          "no motion: check the LEDs are seen"),
    ("stair",   p_stair(),         "levels to +/-12 deg in 1.5 deg steps (twice) and +/-30 in 5: static map, lash, picture scale"),
    ("creep",   p_creep(False),    "0.4 deg/s triangle to +/-2 deg: the threshold the servo ignores"),
    ("creepd",  p_creep(True),     "the same with the flight dither"),
    ("steps",   p_steps(),         "steps of 0.1 to 30 deg both ways: dead time, rise, rate limit"),
    ("sine03",  p_sine(0.3, False), "0.3 deg sines, 1 to 40 Hz"),
    ("sine1",   p_sine(1.0, False), "1 deg sines, 1 to 40 Hz"),
    ("sine1d",  p_sine(1.0, True),  "1 deg sines with the flight dither"),
    ("sine3",   p_sine(3.0, False), "3 deg sines, 1 to 40 Hz"),
    ("flight",  p_flight(True),    "6 s of a simulated flight's roll command, with dither (as flown)"),
    ("flightn", p_flight(False),   "the same command without dither"),
]


def sin_q15(idx):
    q, r = idx >> 8, idx & 255
    return (SIN_Q[r], SIN_Q[256 - r], -SIN_Q[r], -SIN_Q[256 - r])[q]


def play(segs):
    """Replay as bench.c does. Returns [(ticks, dither)] per RUN frame; the fin gets ticks + sign x dither."""
    out, val = [], 0
    for kind, flags, frames, a, b in segs:
        base = val
        for j in range(frames):
            if kind == MOVE:
                if b <= 1 or j + 1 >= b:
                    val = a
                else:
                    num = (a - base) * (j + 1)
                    val = base + ((num + b // 2) // b if num >= 0 else -((-num + b // 2) // b))
            elif kind == SINE:
                num = a * sin_q15(((j * (0xFFFFFFFF // max(b, 2))) & 0xFFFFFFFF) >> 22)
                val = base + ((num + 16384) >> 15 if num >= 0 else -((-num + 16384) >> 15))
            else:
                val = val + TABLES[a][j]
            val = max(-TICKS_MAX, min(TICKS_MAX, val))
            i = len(out)
            dth = 0 if not (flags & F_DITHER) else (DITHER_TICKS if ((i // DITHER_HALF) & 1) == 0 else -DITHER_TICKS)
            out.append((val, dth))
    return out


def crc_of(seq):
    return zlib.crc32(b"".join(((v + d) & 0xFFFF).to_bytes(2, "little") for v, d in seq)) & 0xFFFFFFFF


def build():
    """Pad each profile with a final hold at 0 so its length is 97 x a distinct odd number."""
    profs, used = [], set()
    for pid, (name, segs, what) in enumerate(RAW):
        n0 = sum(s[2] for s in segs)
        n = n0 + 40
        n += (LED_HALF - n) % (2 * LED_HALF)
        while n in used:
            n += 2 * LED_HALF
        used.add(n)
        dither = bool(segs[-1][1] & F_DITHER)
        segs = segs + [(MOVE, F_DITHER if dither else 0, n - n0, 0, 0)]
        assert all(1 <= s[2] <= 65535 and -32768 <= s[3] <= 32767 and 0 <= s[4] <= 65535 for s in segs)
        seq = play(segs)
        assert len(seq) == n and (n // LED_HALF) % 2 == 1 and n % LED_HALF == 0
        profs.append(dict(id=pid, name=name, what=what, segs=segs, frames=n, halves=n // LED_HALF, crc=crc_of(seq)))
    return profs


PROFILES = build()
BY_NAME = {p["name"]: p for p in PROFILES}
BY_HALVES = {p["halves"]: p for p in PROFILES}


def command_deg(name, fin=0):
    """Commanded angle (deg, true) for every RUN frame of a profile, as sent to the given fin."""
    sign = -1 if (fin & 1) else 1
    return [(v + sign * d) / TICKS_PER_DEG for v, d in play(BY_NAME[name]["segs"])]


def emit(path):
    L = ["/* GENERATED by Tools/bench/servo_id/profiles.py --emit. Do not edit: change profiles.py and run it again.",
         " * BENCH-02 servo identification profiles (bench.h). */",
         '#include "bench.h"', "", "#if CFG_BENCH_PROFILE", "",
         f'_Static_assert((BENCH_PRE_FRAMES == {PRE}u) && (BENCH_POST_FRAMES == {POST}u) && (BENCH_LED_HALF == {LED_HALF}u), "regenerate: profiles.py timing");',
         f'_Static_assert((BENCH_DITHER_TICKS == {DITHER_TICKS}) && (BENCH_DITHER_HALF == {DITHER_HALF}u) && (BENCH_TICKS_MAX == {TICKS_MAX}), "regenerate: profiles.py constants");',
         "", "/* Quarter sine, 32767 x sin(pi/2 x k/256). */", "const int16_t k_bench_sin_q[257] = {"]
    for i in range(0, 257, 12):
        L.append("    " + ", ".join(str(v) for v in SIN_Q[i:i + 12]) + ",")
    L += ["};", ""]
    for ti, tab in enumerate(TABLES):
        L.append(f"/* Table {ti}: per-frame steps of the flight roll command (flight_cmd.csv). */")
        L.append(f"static const int8_t tab{ti}[{len(tab)}] = {{")
        for i in range(0, len(tab), 24):
            L.append("    " + ", ".join(str(v) for v in tab[i:i + 24]) + ",")
        L += ["};", ""]
    L.append("const int8_t *const k_bench_tables[] = { " + ", ".join(f"tab{i}" for i in range(len(TABLES))) + " };")
    L.append(f"const uint16_t k_bench_table_len[] = {{ " + ", ".join(f"{len(t)}u" for t in TABLES) + " };")
    L.append(f"const uint32_t k_bench_ntables = {len(TABLES)}u;")
    L.append("")
    for p in PROFILES:
        L.append(f"/* {p['id']} {p['name']}: {p['what']}. {p['frames']} frames, {p['frames'] / FRAME_HZ:.1f} s. */")
        L.append(f"static const BenchSeg seg_{p['name']}[] = {{")
        row = []
        for s in p["segs"]:
            row.append("{ %uu, %uu, %uu, %d, %uu }" % s)
            if len(row) == 4:
                L.append("    " + ", ".join(row) + ","); row = []
        if row:
            L.append("    " + ", ".join(row) + ",")
        L += ["};", ""]
    L.append("const BenchProfile k_bench_profiles[] = {")
    for p in PROFILES:
        L.append(f'    {{ "{p["name"]}", seg_{p["name"]}, {len(p["segs"])}u, {p["frames"]}u, 0x{p["crc"]:08X}u }},')
    L += ["};", f"const uint32_t k_bench_nprofiles = {len(PROFILES)}u;", "", "#endif /* CFG_BENCH_PROFILE */", ""]
    with open(path, "w", newline="\n") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    if "--emit" in sys.argv:
        out = os.path.normpath(os.path.join(HERE, "..", "..", "..", "Core", "hyades", "svc", "bench_profiles.c"))
        emit(out)
        print("wrote", out)
    tot = 0.0
    for p in PROFILES:
        sec = (PRE + p["frames"] + POST) / FRAME_HZ
        tot += sec
        print(f"{p['id']:2d} {p['name']:8s} {sec:5.1f} s  {p['halves']:3d} LED half periods  crc {p['crc']:08X}  {p['what']}")
    print(f"all profiles: {tot / 60:.1f} min; table {sum(len(p['segs']) for p in PROFILES) * 8 + sum(len(t) for t in TABLES) + 514} bytes")
