# SAF-01: canard manoeuvre authority, a worst-case bound for the safety case

**Date:** 9 November 2026.
**Question (James):** if the canards were driven to the worst possible deflection for the whole flight, how far could the vehicle be turned? Can canards of this size make it manoeuvrable, or can they do nothing more than hold it pointing up?
**Tool:** the 6DOF SITL (MC-01, LD-01 corrected plant) with a forced-deflection override (`force_mode`, `force_deg`, `force_t`), and `Tools/sitl/manoeuvre_sweep.py`. Raw results are in `runs/manoeuvre_sweep.json`.

Markers used in this document:
- **[fact]**: from a cited source;
- **[inference]**: my reasoning;
- **[assumption]**: unverified.

## 1. Result

**The canards cannot overpower the airframe's static stability.** In the worst corner the vehicle still flies at least 46° above the horizon under thrust. The canards' own contribution is at most 15° of flight-path angle. Everything beyond that is wind weathercocking, which happens with the canards neutral too.

| Case (144 conditions each) | Min flight-path angle under thrust | Max angle of attack at q > 500 Pa, in flight | Max extra horizontal distance at apogee vs neutral | Min apogee |
|---|---|---|---|---|
| Canards neutral | 58.5° (70.5° with wind ≤ 6 m/s) | rail exit only (see §3) | — | 281 m |
| All four at 12° (servo travel limit), whole flight | 47.8° (59.7° with wind ≤ 6 m/s) | ≈ 2.6° from the canards | +236 m | 203 m |
| All four at 15° (canard stall), whole flight | **46.1°** (58.0°) | ≈ 3.0° | +251 m | 192 m |
| Adversarial resonant pumping, 15° (bang-bang against the true body rate) | 55.6° (66.0°) | ≈ 3.0° | +251 m | 247 m |

The flight-path change against the same conditions with neutral canards is at most −12.9° (12°), −14.7° (15°) and −14.7° (pumping).

**Conditions swept:**
- all six shortlist motors;
- wind 0, 6 and 12 m/s from four directions, turbulence off;
- rail tilt 0 and 5° aligned with the wind;
- **worst aerodynamic corner throughout:** canard effectiveness Cl_δ × 1.3, normal-force slope CN_α × 0.9, CG 20 mm aft and CP 20 mm forward, i.e. about 0.65 calibre less static margin than nominal.

The worst case in every set is the lowest-thrust motor (G40W) in 12 m/s wind with the rail tilted downwind. That is outside normal launch limits **[inference]**.

## 2. Why: an authority ratio

At flight dynamic pressure, the canards' total lateral force at full deflection is a few newtons. The body's normal force is about 213 N per radian of AoA at q = 6 kPa, acting about 0.15–0.2 m behind the CG.

So the trim AoA the canards can command is:

α_trim ≈ (canard moment at full deflection) / (restoring moment per radian) ≈ 1–1.3° nominal, ≈ 2.6–3° in the worst aerodynamic corner.

Both the SITL and a hand estimate give this (≈ 1 N per canard at δ = 12°, q = 6 kPa) **[inference, matches the SITL]**.

**Stall caps it independently of software.** Canard lift is capped at the 15° stall (model assumption, A-R in CDS-01), so **no command, servo travel or control law can produce more than this trim**. That is the property a safety case wants: a physical bound, not a software one.

The resulting turn rate is about the lateral acceleration over the speed. The worst case is 1.7–2.5 g lateral against 80–100 m/s under thrust, giving a few °/s. Over a 1–3 s burn that is the ≤ 15° flight-path change seen above.

**Resonant excitation was tested explicitly.** Mode 4 drives both canard pairs bang-bang against the true pitch and yaw rates: maximum energy input, with a perfectly informed adversary. It did **not** exceed the static trim AoA (≈ 2–3°). Aerodynamic pitch damping plus the static margin absorb the input at the airframe's 0.7–4 Hz pitch frequency.

## 3. Interpretation and limits

- **The large AoA values (up to 19°) occur at rail exit in 12 m/s crosswind**, identical with and without canard deflection. They are the wind, not the canards.
- **"Tilt > 90°" near apogee is the normal arc-over** at near-zero speed and is not authority. All rockets do this, and recovery deploys there.
- **Landing distance** (up to about 1.1 km in 12 m/s wind) is dominated by drift under the parachute. The canards add at most about 250 m of displacement at apogee.
- **Model limits:**
  - linear aerodynamics up to the 15° canard stall;
  - rigid body;
  - canard–body interference taken as covered by Cl_δ × 1.3 **[assumption]**;
  - no roll: the deflection is held in one plane, as asked. With roll the turn plane rotates and the net displacement is smaller **[inference]**.
- **What would change this:** a much larger canard area, or a marginally stable airframe (a static margin well under 1 calibre). Any future canard resize or airframe change must re-run `manoeuvre_sweep.py`.

## 4. Safety-case statement (proposed wording)

> With the canards at any deflection up to their aerodynamic stall, for the whole flight and under any control law (including a resonant, rate-informed adversary), the Hyades vehicle's trimmed angle of attack in powered flight is bounded at about 3°. Its flight path under thrust remains at least 46° above the horizontal in the worst analysed corner, which is 12 m/s wind, 5° rail tilt, the lowest-thrust motor and 0.65 calibre reduced stability. This is no more than 15° below the same flight with neutral canards. The canards therefore cannot steer the vehicle off a near-vertical trajectory; their authority is limited to roll and small attitude trims. This rests on the 6DOF analysis in SAF-01 and is to be confirmed by the measured canard Cl_δ (CDS-01) and the measured static margin before flight.

## 5. How to run

```
cd Firmware/FC-V4/Tools/sitl
make
./sitl motor=motors/G79W.csv force_mode=2 force_deg=12     # fixed diagonal deflection, whole flight
./sitl motor=motors/G79W.csv force_mode=4 force_deg=15 sv_travel=0.2618   # adversarial pumping at stall
python manoeuvre_sweep.py                                    # 576 runs, about 2 min
```
