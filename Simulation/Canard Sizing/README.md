# Canard sizing tool

Open-loop **plant characterisation** for the 62 mm canard CV. Scope is set by
`ADR-001-canard-sizing.md`: this produces the plant, not the controller. Control
laws, gains and closed-loop response stay in the MATLAB/Simulink toolchain; this
exports the parameter set that toolchain consumes.

```
pip install streamlit numpy matplotlib pandas
streamlit run app.py        # GUI
python3 verify.py           # model verification, all checks must pass
```

`canard_model.py` is importable and headless — use it directly from an optimiser
or a batch sweep:

```python
import canard_model as cm
c   = cm.Canard(span=0.016, root_chord=0.055, tip_chord=0.035, hinge_frac=0.20)
res = cm.evaluate(c, cm.Vehicle(), cm.Servo(), cm.Linkage(4.0),
                  cm.Bearing(c_visc=0.02), cm.DEFAULT_ENVELOPE, cm.DEFAULT_REQ)
print(res["score"], res["margins"])
cm.export(c, cm.Vehicle(), cm.Servo(), cm.Linkage(4.0), cm.Bearing(0.02),
          res, "canard_plant.json")
```

## What the screening metric is

`score` = **the minimum normalised margin** across authority, limit-cycle
granularity, hinge torque, torsional divergence, span cap and static-margin
budget. Each margin is ≥ 1 when satisfied. Maximising the worst margin gives an
interior optimum rather than pushing to a constraint boundary. It ranks designs;
it does not decide.

## What it deliberately does not do

- **No transonic aerodynamics.** M 0.8–1.2 returns a nominal with ±30 % and
  `transonic=True`. Treat it as a design-exclusion band.
- **No control-effectiveness sign reversal.** Not predictable semi-empirically
  (project lit review). Freeze geometry here, verify by CFD or tunnel, and keep
  the online effectiveness-sign monitor regardless.
- **No control law or gains.**

## Inputs that must be measured, not assumed

| input | default | why it matters |
|---|---|---|
| mechanical backlash | 0.15° | dominant sensitivity — 0 → 0.30° swings the worst margin 1.21 → 0.63 |
| servo deadband at the output shaft | 0.18° | inferred from 2 µs; measure it |
| servo rotational stiffness `K_δ` | 30 N·m/rad | sets divergence; not binding at this scale but cheap to take |
| reflected gearbox inertia | 0 | unknown for a hobby servo; can dominate at high linkage ratio |
| forebody / fin-can roll inertias | 2.0 / 0.4 g·m² | scale everything |

## Files

| file | role |
|---|---|
| `canard_model.py` | physics — geometry, `C_Nα`, roll derivatives, hinge moment, servo derating, two-body roll, constraints |
| `app.py` | Streamlit GUI — design point, sizing sweep, envelope, JSON export |
| `verify.py` | verification against limits with known answers |
| `ADR-001-canard-sizing.md` | the decision record this implements |
