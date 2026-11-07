# ADR-002 — Preliminary canard sizing for first flight (rate damping only)

Status: **proposed, pending simulation verification**, 2026-08-24.
Supersedes the geometric-scaling analysis in `canard-sizing-the-real-trade.md`.

## Decision

| | as built | **proposed** |
|---|---|---|
| exposed semi-span s | 12.0 mm | **18.0 mm** |
| root chord C_r | 30.0 mm | **24.0 mm** |
| tip chord C_t | 15.0 mm | **12.0 mm** |
| hinge line | 0.90 c | **0.20 c** |
| aspect ratio s²/S_c | 0.53 | **1.00** |
| total planform area | 10.80 cm² | 12.96 cm² (+20%) |
| Cl_δ | 0.2975 | **0.6569** (2.21×) |
| Cl_p | −0.4291 | −0.9179 |
| `P.Controller.c` | 210 | **95.2** |
| Cl_0 holdable, half travel | 0.031 (1.0σ) | **0.069 (2.3σ)** |

**2.21× the roll authority for +20% area.**

## Reasoning

**The current panel is stubby and that is where the loss is.** At AR 0.53 the
low-AR lift slope is 0.83 /rad; at AR 1.00 it is 1.65 /rad. Roll moment goes as
area × spanwise arm, so trading chord for span buys authority twice over — a
better lift slope and a longer arm — at almost constant area. Geometric scaling,
which was the earlier proposal, spends area on chord and buys neither.

Efficiency (Cl_δ per cm² of planform) rises monotonically as chord is traded for
span at every span tested. The chord was not reduced further than 24 mm because
the spar and the section thickness need the room.

**Span stops at 18 mm** because the resolution penalty scales with Cl_δ: the
roll-rate error one servo dead zone produces is ∝ Cl_δ, so authority and
resolution trade directly. At s = 18 mm the residual rate error is ~0.7 rad/s if
the dead zone is 0.1° and ~2.8 rad/s if it is 0.4°. At s = 22 mm the pessimistic
case reaches 4.5 rad/s, which is a poor damper at high q. Since the dead zone is
**unmeasured** — the manufacturer had no data — the design is deliberately
placed where the pessimistic case is still acceptable.

**Hinge at 0.20 c.** It must sit ahead of the most-forward aerodynamic centre at
every Mach or the panel is overbalanced somewhere in the flight and can diverge.
Subsonic x_ac ≈ 0.25 c and it migrates aft transonically (taken as 0.35 c at
M = 0.82). A hinge at 0.20 c is therefore always underbalanced — the aero moment
always restores toward zero — and it keeps a small consistent hinge moment
(0.0064 N·m subsonic) that takes up the 0.115° of backlash in one direction
instead of letting the panel rattle inside it. The previous 0.90 c was
overbalanced by 0.65 MAC and capped the canard on servo torque.

## Derived numbers

- MAC 18.67 mm, taper 0.50, y_Cp 0.03900 m
- C_Nα (all four panels) 1.079 /rad
- Hinge moment at q = 40 kPa, 12°: 0.0064 N·m subsonic, 0.0191 N·m transonic
  worst case → **torque margin 27** against 0.5198 N·m stall
- Torsional divergence q: 13.2 MPa at K_servo = 30 N·m/rad, 2.2 MPa at a
  pessimistic 5 — against 40 kPa max flight q. Not a constraint.
- Root bending 0.0546 N·m, lift 6.82 N per panel at q = 40 kPa, 12°
- Minimum spar diameter: **1.41 mm in steel, 3.82 mm in FDM ASA**
- Panel inertia about the hinge 6.74e-8 kg·m² at t/c 0.15

## Consequences

- **A metal spar is required.** An all-ASA shaft needs 3.82 mm inside a section
  that is only about 3.6 mm deep at 0.20 c on a 24 mm chord. Use 2.5–3 mm
  steel, which carries the root bending with a large margin.
- `P.Controller.c` moves 210 → 95.2. Crossover is unchanged at 2.66 Hz: it is
  actuator-limited (f_N, τ_d, Ms ≤ 2), not canard-limited, and no canard change
  moves it.
- The trim dead band `BandK·dz·q/c` scales as 1/c, so it roughly doubles. The
  trim gains must be re-swept at the new c.

## Open items before this becomes a specification

1. **Static margin.** The canards add C_Nα = 1.079 /rad well forward of the CP.
   Check in OpenRocket with them fitted, target ≥ 1.5 calibres. This is the only
   flight-safety-critical item the roll analysis cannot close. If it fails,
   reduce span before anything else.
2. **Simulation verification** at the new geometry and re-optimised gains,
   including a dead-zone robustness sweep.
3. **Section thickness** is not optimised here — the roll model is
   thickness-blind. t/c 0.15 is assumed for inertia. A 20% section at M = 0.82
   will have significant transonic drag rise; thinner is better aerodynamically
   and worse for spar room.
