# ADR-001: Canard Sizing Method for Roll Control

**Status:** Proposed
**Date:** 2026-08-23
**Deciders:** James
**Supersedes:** nothing
**Related:** `Canard Controlled Rocket Lit Review`, `h743_flight_architecture.md` §4.3 (allocation)

---

## 1. Context

The 62 mm canard CV needs a canard planform. Roll stabilisation is the initial
mission; full attitude control is the stated follow-on. Four all-moving canards
drive `τ = q̄·S·l·B·δ`, `B ∈ ℝ³ˣ⁴` (h743 architecture §4.3).

**What already exists.** A MATLAB/Simulink toolchain does 6DOF, stability,
sensitivity and controllability analysis, with Monte Carlo gain optimisation.
Control laws, closed-loop response and gain tuning live there and stay there.

**What is missing.** Nothing produces the *plant*: the map from canard geometry
and flight condition to roll authority, hinge moment, actuator-limited deflection
rate and the aeroelastic limits. Sizing is currently an assumption fed into a loop
that cannot question it.

**Actuator (fixed).** Blue Bird BMS-127WV+: 5.3 kg·cm (0.52 N·m) stall and
0.05 s/60° (1200 °/s) at 8.4 V; 4.0 kg·cm / 0.07 s at 6.0 V; 12-bit digital
controller, 2 µs control deadband, 250 Hz frame, 11 g, all-metal gears, coreless.

**Prior findings that constrain the answer** (from the lit review):

- Canard control-effectiveness **sign reversal** from canard–tail vortex
  interference is the dominant failure mode, is not predictable semi-empirically,
  and is mitigated geometrically by **reducing canard span** and keeping the tail
  **> 4 MAC** aft.
- **Hobby-servo rate limit, backlash and delay are the practical limiting factors**,
  and must be characterised *under aerodynamic load*.
- Static margin > 2 calibres must hold through transonic; canards forward of the CG
  erode it.

---

## 2. The finding that should drive the decision

A scale check on a representative canard (exposed span 25 mm, root 55 mm, tip
35 mm, 4 off, `S_f` = 1125 mm², `AR_e` = 1.11, `I_xx` ≈ 2×10⁻³ kg·m²) says the
intuition behind the question needs inverting:

| condition | δ needed for 500 °/s² roll accel | hinge moment at 10°, hinge at 0.20 c̄ |
|---|---|---|
| M 0.15, q = 0.54 kPa (rail exit) | **5.9°** | ~0 |
| M 0.50, q = 15 kPa | **0.21°** | 2 % of stall |
| M 1.3, q = 86 kPa | **0.03°** | **107 % of stall** |

Against a servo deadband of ~0.18° referred to the canard (2 µs on a ~1000 µs/90°
band; 12-bit resolution is 0.03° and is *not* the limit):

**Above roughly M 0.4 the entire roll loop operates inside the actuator deadband.**
Roll authority is over-satisfied by two orders of magnitude at mid-q. The binding
constraints are, from below, the rail-exit condition, and from above, hinge moment,
torsional divergence, static-margin erosion and deadband-driven limit cycling — all
of which get *worse* with area. So:

> For roll control alone, the canards are sized by the **low-q end** and capped by
> **actuator quantisation and hinge moment at high q**. Nothing in the roll mission
> pushes area up except rail exit.

Two consequences that change what the tool must contain:

**(a) The linkage ratio is a design variable of equal standing to area, and it is
nearly free.** With `n = δ_servo / δ_canard`, the canard-referenced quantities scale
as deadband `∝ 1/n`, max rate `∝ 1/n`, available torque `∝ n`, and rotational
stiffness — hence divergence dynamic pressure — `∝ n²`. Rate is currently in gross
surplus (1200 °/s available; order 100 °/s demanded). At `n ≈ 3`: deadband 0.06°,
rate 400 °/s, torque 1.56 N·m, `q_div` ×9. This buys more than any plausible change
in planform, and it is a linkage decision, not an aero one.

**(b) Roll-only sizing under-determines the geometry, so pitch/yaw must be sized
now.** Otherwise the surfaces get re-cut when full attitude control arrives.
Recommend the tool carry a pitch/yaw authority requirement as an explicit,
optional constraint from day one.

### 2.1 What the built model then said

Running the tool (§6) over the planform grid, with the maximin screening metric
and the default requirement set, confirms and sharpens the above:

- **The optimum runs to low aspect ratio** — best on grid is roughly 14–18 mm
  exposed semi-span on a 40–70 mm root, `AR_e` ≈ 0.5–1.0. This is the same
  direction the sign-reversal mitigation pushes (reduce span), so the aerodynamic
  and control constraints agree for once. Worth exploiting.
- **The limit-cycle constraint binds almost everywhere**, and static margin binds
  at large area. Authority binds only in a thin band at small area, driven
  entirely by rail exit.
- **Linkage ratio saturates.** Worst margin rises roughly linearly with `n` up to
  `n ≈ 5`, then flattens as authority takes over from granularity. `n ≈ 4–5` is
  the knee; beyond that you are giving up rate for nothing.
- **Divergence is not binding** — 13× margin even at `n = 1` with an assumed
  `K_servo` = 30 N·m/rad. I over-weighted it before running the numbers. It stays
  in the tool because it is nearly free to evaluate and because it *would* bind if
  the hinge moves aft of ~0.35 c̄ or the linkage is compliant, but it should not
  drive the planform.
- **Backlash is the dominant sensitivity.** Sweeping assumed backlash from 0 to
  0.30° moves the worst margin from 1.21 to 0.63 — enough to flip a design from
  feasible to infeasible on its own. This is the single highest-value measurement
  in the whole programme, and it is an afternoon on the bench.

---

## 3. Decision

Build a standalone **open-loop plant-characterisation tool** in Python, with a
Streamlit front end, that maps

> canard geometry × linkage × flight condition × bearing coupling
> → { roll authority, hinge moment, actuator-limited rate, plant poles, constraint flags }

and exports a parameter set for the MATLAB stability / Monte-Carlo loop.

It computes **no control law, no gains and no closed-loop response** — those stay in
MATLAB. It produces a **feasible-region map with a scalar screening metric**, not an
automatic optimiser, because the aero model's transonic uncertainty exceeds the
differences between neighbouring candidate designs.

---

## 4. Options considered

### Option A — Analytic / semi-empirical plant model in Python (chosen)

| Dimension | Assessment |
|---|---|
| Fidelity | ±10–15 % subsonic and supersonic; unusable M 0.8–1.2 (flagged, not interpolated silently) |
| Speed | microseconds per point; full 4-D sweep interactive |
| Integration | plain importable module; MATLAB reads the exported JSON |
| Maintenance | Low — one file, no binaries |

**Pros:** fast enough for live sliders and dense sweeps; every equation visible and
citable; runs headless in an optimiser; honest about where it fails.
**Cons:** no vortex interference, so no sign-reversal prediction; low-AR supersonic
needs a blended asymptote rather than a first-principles result.

### Option B — Missile DATCOM in the loop

| Dimension | Assessment |
|---|---|
| Fidelity | ±15 % for ~75 % of validated configs; > 20–30 % error subsonic and transonic |
| Speed | ~seconds per case — kills interactivity, tolerable for an overnight sweep |
| Integration | external binary, fixed-format input decks, licence/export questions |
| Maintenance | High |

**Pros:** the standard preliminary tool; handles body–fin interference properly.
**Cons:** by your own lit review it *cannot* be trusted to predict the sign-reversal
boundary or transonic CP, which is precisely where extra fidelity would be worth
having. Its error band is comparable to the spread between candidate designs near
the optimum — it buys precision, not accuracy.

### Option C — CFD-in-the-loop or a CFD-trained surrogate

| Dimension | Assessment |
|---|---|
| Fidelity | The only credible route to the sign-reversal boundary |
| Speed | Hours to days per configuration |
| Integration | Meshing, solver setup, validation burden |
| Maintenance | Very high |

**Pros:** the only tool that answers the question that actually matters most.
**Cons:** wrong instrument for *sizing*. Correct instrument for *verifying* the
chosen geometry once frozen.

### Option D — Extend the existing MATLAB toolchain instead

| Dimension | Assessment |
|---|---|
| Fidelity | Same as A |
| Speed | Same as A |
| Integration | Zero handoff — one environment |
| Maintenance | Couples the sizing model to the controller model |

**Pros:** no new language, no export format, one source of truth.
**Cons:** entangles a *plant-definition* tool with a *controller-tuning* tool, so a
sizing sweep drags the whole 6DOF harness with it; and the GUI you asked for is
weaker in MATLAB than in Streamlit. Reasonable fallback if the handoff proves
annoying.

---

## 5. Trade-off analysis

The decisive trade is **fidelity versus honesty**. DATCOM and CFD both raise
fidelity, but the quantity they would improve — transonic `C_Nα` and the vortex
interference term — is the one where DATCOM is documented to fail and CFD is too
slow to sweep. The right response to an unpredictable region is not a better solver
inside the sizing loop; it is a **design-exclusion flag** and an explicit uncertainty
band, with CFD or wind tunnel reserved for verification of the frozen geometry.

Second trade: **automatic optimiser versus feasible-region map.** An optimiser
returns a point; a map returns the shape of the constraint set. Given that the
objective landscape is flat over a wide area range (authority is over-satisfied) and
the constraints are what bind, the map is more informative and less misleading. The
scalar screening metric exists to rank, not to decide.

---

## 6. Model specification

Reference area `S_ref = πd²/4`, reference length `d`, all angles in radians.

**Normal-force slope, per exposed panel, referenced to panel area `S_f`.**
Blend three limits and take the governing one:

- Subsonic, low AR (DATCOM/Diederich):
  `C_Nα = 2π·AR_e / (2 + √(4 + AR_e²β²(1 + tan²Λ_c/2 / β²)))`, `β² = 1 − M²`
- Slender-wing limit, Mach-independent: `C_Nα → πAR_e/2`
- Supersonic 2-D with tip loss: `C_Nα = (4/β)(1 − 1/(2β·AR_e))` for `β·AR_e ≥ 1`,
  reverting to the slender limit below

with `AR_e = 2s²/S_f` (image-plane effective AR for a panel on a body) and the
Barrowman fin–body interference factor `K_fb = 1 + r/(s+r)`. The subsonic and
slender limits agree to ~7 % at `AR_e ≈ 1.1`, which is the cross-check.
**M 0.8–1.2 returns a flagged band, not a number.**

**Roll derivatives** (derived from strip loading proportional to chord; the
published RocketPy forms are the cross-check):

- `C_lδ = N_f · C_Nα · S_f · ȳ / (S_ref · d)`, `ȳ = ∫c(y)y dy / ∫c(y) dy`
- `C_lp = −2 N_f · C_Nα / (S_ref d²) · ∫ c(y) y² dy`,
  and for a trapezoid `∫c y² dy = (s/12)[(c_r+3c_t)s² + 4(c_r+2c_t)s r + 6(c_r+c_t)r²]`

**Two-body roll model with swept bearing coupling** (your Q3 answer):

```
I_f ṗ_f = q S_ref d [C_lδ^canard δ + C_lp^fore p̂_f] − T_b + L_dist,fore
I_t ṗ_t = q S_ref d [C_l0^cant + C_lp^tail p̂_t] + T_b
T_b     = c_b (p_f − p_t) + T_coulomb · tanh((p_f − p_t)/ε)
```

Sweep `c_b` from 0 (free) to ∞ (seized). The decision-relevant output is the
**forebody roll pole**: seized gives a well-damped first-order plant; free leaves
the controlled body a near-integrator, which is exactly the condition under which
deadband and backlash produce a wandering limit cycle. Note that with a free tail
the dominant residual roll disturbance becomes **canard rigging and servo trim
error**, which scales with canard area — another term pushing area down.

**Hinge moment and actuator.**

- `H = q S_f c̄ C_Nα δ (x̄_ac − x̄_h)`, with `x̄_ac ≈ 0.25` subsonic and `≈ 0.50`
  supersonic; the migration is what makes the hinge-line choice load-bearing.
- Servo torque–speed line: `δ̇(T) = δ̇₀ (1 − T/T_stall)`, `T = |H|/n + J_eff |δ̈|`,
  with `J_eff` = canard inertia about the hinge plus reflected gearbox inertia
  (unknown for a hobby servo — exposed as an input, default 0, flagged).
- Linkage ratio `n` applied as in §2(a). Deadband and backlash referred to the
  canard. Transport delay and the 250 Hz frame as separate inputs.

**Aeroelastic constraint — and a correction worth stating plainly.** The NACA
TN 4197 flutter formula that rocketry calculators use is **for fixed fins and does
not apply to an all-moving surface**; the source says so. The right constraint for
an all-moving canard is **torsional divergence about the hinge**, set by the finite
rotational stiffness of servo gearbox plus linkage. It turns out (§2.1) not to bind
for this vehicle, but it is nearly free to evaluate and it is the correct
formulation to have in hand:

```
q_div = K_δ / (S_f · c̄ · C_Nα · (x̄_ac − x̄_h))        K_δ ∝ n² K_servo
```

`K_δ` is measurable on the bench in an afternoon: apply a known torque at the
canard, measure deflection. This should be done before the geometry is frozen.

**Constraints checked:** `q_div` margin; hinge moment ≤ `n·T_stall` with margin;
static margin ≥ 2 cal with the canard's forward `C_Nα` included; `δ` within the
linear/unstalled range; exposed span ≤ the sign-reversal cap; canard–tail
separation ≥ 4 MAC; Mach-exclusion flag through 0.8–1.2.

---

## 7. Airfoil / section decision

Treat the section as a **small discrete set** (smoothed hexagon — your current
choice, sharp double wedge, biconvex, NACA 000X) with **t/c and hinge fraction as
the continuous variables**. Rationale: for thin sections at |δ| < 10° the section
changes `C_Nα` by under ~10 %, which is inside the model's own error band. What it
genuinely changes is `x̄_ac` (→ hinge moment → divergence), wave drag `∝ (t/c)²`,
and stall onset. Optimising section *shape* against this aero model would be false
precision; optimising t/c and hinge fraction would not.

Your smoothed hexagon is a sound choice and I would not change it: the rounded
leading edge buys subsonic `C_N,max` and delays leading-edge separation at
deflection, while the flat mid-section keeps supersonic wave drag near the
double-wedge value.

**Open decision — hinge line.** Placing the hinge forward of the most-forward
`x̄_ac` over the envelope (≈ 0.20–0.22 c̄) keeps the hinge moment always restoring,
at the cost of peak torque. Placing it between 0.25 and 0.50 c̄ reduces peak torque
but lets the hinge moment **reverse sign through transonic**, which unloads the gear
train and opens the backlash precisely where the loop needs precision. Given the
finding in §2 — that torque is not the binding constraint once `n ≈ 3` — I recommend
the always-restoring option, but this is yours to decide.

---

## 8. Consequences

**Easier**
- Canard sizing becomes a screening exercise over a visible constraint set rather
  than an assumption defended after the fact.
- The MATLAB loop receives a consistent `C_lδ(M, q)`, `C_lp(M, q)`, actuator
  envelope and plant poles instead of hand-entered constants.
- Servo selection and canard sizing converge together via the linkage ratio.

**Harder**
- A second language in the loop, and an export format to keep in step.
- The transonic exclusion band is honest but inconvenient: the tool will refuse to
  give a number exactly where a number would be most reassuring.

**To revisit**
- When pitch/yaw control is added, the authority requirement changes by roughly the
  ratio of `I_yy` to `I_xx` and the sizing driver moves from rail exit to pitch
  response. Build the constraint in now; switch it on later.
- If bench measurement of `K_δ` shows divergence binding earlier than expected, the
  linkage and hinge fraction — not the planform — are the first things to change.
- Sign reversal remains unpredicted. Freeze the geometry with this tool, then verify
  by CFD or wind tunnel, and keep the online effectiveness-sign monitor regardless.

---

## 9. Action items

1. [ ] Build `canard_model.py` — geometry, compressible `C_Nα`, roll derivatives,
       hinge moment, servo derating, two-body roll with bearing sweep, constraints.
2. [ ] Build `app.py` — Streamlit front end: sliders, envelope plots, feasible-region
       map, JSON export for MATLAB.
3. [x] Verify (`verify.py`, all passing): `C_Nα` → πAR/2 as AR→0, → 2π at high AR,
       Prandtl–Glauert scaling, → 4/β supersonically, blend continuous at βAR = 1,
       transonic flag raised; `C_lp` and `C_lδ` against closed-form rectangular-fin
       results; the trapezoid span integral against the published RocketPy closed
       form; linkage scaling laws; the seized two-body model collapsing exactly to
       the rigid single-body time constant; ISA at 0 and 11 km.
   [ ] Still outstanding: spot-check `C_lδ` against RASAero II or a wind-tunnel point.
4. [ ] **Bench-measure backlash and `K_δ`** on the BMS-127WV+ under aerodynamic-scale
       load. Backlash is the highest-value measurement in the programme — it alone
       swings the worst margin from 1.21 to 0.63 across a plausible range (§2.1).
       `K_δ` matters less than expected but costs nothing to take at the same time.
       Also measure the deadband referred to the output shaft rather than trusting
       the 2 µs figure.
5. [ ] Decide the hinge fraction (§7) and the linkage ratio `n` (§2a).
6. [ ] Add the pitch/yaw authority constraint before freezing geometry (§2b).
7. [ ] Feed the exported parameter set into the MATLAB stability / Monte-Carlo loop
       and iterate; convergence criterion — the feasible region stops moving by more
       than the model's stated uncertainty band.

---

## 10. Sources

- Blue Bird BMS-127WV+ manufacturer specification — https://www.blue-bird-model.com/products_detail/413.htm
- RocketPy technical documentation, roll forcing and damping equations — https://docs.rocketpy.org/en/latest/technical/aerodynamics/roll_equations.html
- Martin, NACA TN 4197 fin flutter formula, as summarised in McDonald, *Fin Flutter: A Preliminary Report* — https://www.vibrationdata.com/tutorials/finflutr.pdf (states applicability to fixed fins)
- Project `Canard Controlled Rocket Lit Review` — sign reversal, DATCOM accuracy limits, servo characterisation requirement, dual-spin/bearing evidence
