"""Re-run one Monte Carlo seed from a runs.csv with a trace: python rerun.py runs/x/runs.csv SEED trace.csv [k=v ...]"""
import csv, subprocess, sys, os
rows = {int(float(r["seed"])): r for r in csv.DictReader(open(sys.argv[1]))}
r = rows[int(sys.argv[2])]
args = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "sitl"), f"seed={sys.argv[2]}", f"trace={sys.argv[3]}"]
args += [f"{k[3:]}={v}" for k, v in r.items() if k.startswith("in_") and v != ""] + sys.argv[4:]
print(subprocess.run(args, capture_output=True, text=True, cwd=os.path.dirname(os.path.abspath(__file__))).stdout)
