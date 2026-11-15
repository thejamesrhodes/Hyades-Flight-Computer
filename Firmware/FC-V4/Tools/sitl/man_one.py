import sys, subprocess, numpy as np, argparse
sys.path.insert(0, "."); import mc
def params(motor, seed, extra):
    a = argparse.Namespace(servo="bench02", cant_sd=0.1, fin1_sd=0.1, bear_max=0.006, wind_max=8.0, n_gear=1.0, fault="", shadow=False, set=[])
    rng = np.random.default_rng(seed); d = mc.sample(rng, motor, a); d.update(extra); return d
if __name__ == "__main__":
    motor, seed = sys.argv[1], int(sys.argv[2]); extra = dict(kv.split("=", 1) for kv in sys.argv[3:])
    d = params(motor, seed, extra)
    r = subprocess.run(["./sitl_man", f"seed={seed}"] + [f"{k}={v}" for k, v in d.items()], capture_output=True, text=True)
    print(r.stdout[:2000], r.stderr[:500])
