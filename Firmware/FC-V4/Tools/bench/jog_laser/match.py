"""Parse the ground-station console log, pair mouse-click releases with 'cal jog' commands."""
import re, json, sys, numpy as np
LOG = sys.argv[1]
WINDOWS = {"83": (15*60+19, 16*60+10), "84": (18*60+16, 19*60+15)}   # log clock (s after midnight) covered by each video
cmds = []
for l in open(LOG, encoding="utf-8", errors="replace").read().splitlines():
    m = re.match(r"(\d\d):(\d\d):(\d\d) › cal (jog|set) (\d) ([+-]?[\d.]+)", l)
    if m:
        h, mi, s = map(int, m.groups()[:3])
        cmds.append(((h if h < 12 else h - 24)*3600 + mi*60 + s, m.group(4), int(m.group(5)), float(m.group(6))))
C = {k: [c for c in cmds if a <= c[0] <= b] for k, (a, b) in WINDOWS.items()}
json.dump(C, open("cmds.json", "w"))
cl = json.load(open("clicks.json"))
for n in C:
    t = np.array(cl["79"+n]); rel = []; i = 0
    while i < len(t):                         # press + release pairs, keep the release
        if i+1 < len(t) and t[i+1]-t[i] < 0.25: rel.append(t[i+1]); i += 2
        else: rel.append(t[i]); i += 1
    rel = np.array(rel); w = C[n]; best = None
    for T0 in np.arange(w[0][0]-6, w[0][0]+1, 0.01):   # video-to-log clock offset
        j = 0; m = []
        for c in w:
            while j < len(rel) and rel[j]+T0 < c[0]-0.05: j += 1
            if j < len(rel) and rel[j]+T0 < c[0]+1.05: m.append(j); j += 1
            else: m.append(None)
        k = sum(x is not None for x in m)
        if best is None or k > best[0]: best = (k, T0, m)
    k, T0, m = best
    print(n, "matched", k, "/", len(w))
    json.dump({"T0": T0, "t": [(float(rel[x]) if x is not None else None) for x in m]}, open(f"match{n}.json", "w"))
