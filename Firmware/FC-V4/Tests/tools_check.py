"""Cross-check: records packed by the firmware (C) decode correctly in Tools/fc_pull.py."""
import csv, os, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Tools"))
import fc_pull

raw = open(sys.argv[1], "rb").read()
out = tempfile.mkdtemp()
fc_pull.decode(raw, out)
rows = list(csv.DictReader(open(os.path.join(out, "session1_state.csv"))))
ev = list(csv.DictReader(open(os.path.join(out, "session1_events.csv"))))
assert len(rows) == 3, rows
assert rows[2]["phase"] == "READY" and rows[2]["mode"] == "FULL"
assert abs(float(rows[2]["wz_dps"]) - 25.0) < 0.06 and abs(float(rows[0]["az_ms2"]) - 9.81) < 0.006
assert abs(float(rows[0]["fin0"]) + 0.5) < 0.005 and rows[0]["faults"] == "0080"
assert len(ev) == 1 and ev[0]["text"] == "SETTLE->READY", ev
print("tools_check: OK (decoder matches firmware packing; torn record rejected)")
