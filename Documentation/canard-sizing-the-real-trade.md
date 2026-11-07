# Canard sizing: area buys authority, not bandwidth

Analytic result, 2026-08-24. Monte Carlo confirmation pending.
Supersedes the framing in ADR-001 §2 that treated bandwidth as size-dependent.

## The finding

**Canard area does not buy closed-loop bandwidth.** With the gain schedule
re-tuned at each size to hold the crossover (\(c \propto 1/Cl_\delta\)), the loop
product \(Cl_\delta c\) is invariant — 62.5 for this vehicle at every size
tested. The crossover cap comes from the **actuator** (f_N = 80 rad/s,
τ_d = 14 ms, Ms ≤ 2) and sits at **2.66 Hz** whether the panel is 5 cm² or
43 cm². \(Cl_p\) does change with area but does not move the cap.

Consequence: every metric that depends only on loop gain — the pre-trim droop,
settling, stability margins — is identical at every size *by construction*. An
earlier size study reported a failure rate flat to within noise across a 6×
area range; that looked like a bug and was actually a theorem.

## What area does change

Two opposing effects, both analytic:

**Authority improves.** \(\delta_{req} = Cl_0/Cl_\delta\), so it falls as
1/area. Undersize the canard and the trim clamp — or the mechanical stop — is
reached before the bias is cancelled, leaving a standing roll rate for the
whole flight.

**Resolution worsens.** The servo dead zone is a fixed angle, so as
\(\delta_{req}\) shrinks the dead zone becomes a larger fraction of the working
deflection. The rate error one dead zone produces is

$$\Delta p_{dz} \approx \frac{q\,Cl_\delta\,dz}{Cl_\delta c + q d |Cl_p|/(2V)}$$

and because \(Cl_\delta c\) is invariant, \(\Delta p_{dz}\) grows **linearly
with area**.

## The numbers

Servo travel ±12°, dead zone 0.2005° at the canard (n = 1), 40% of travel
budgeted for holding the bias, \(\Delta p_{dz}\) evaluated at q = 40 kPa.

| scale | area cm² | Cl_δ | c_opt | δ_req @Cl_0=0.03 | Cl_0,max | σ margin | Δp_dz rad/s |
|---|---|---|---|---|---|---|---|
| 0.70 | 5.29 | 0.145 | 432 | 11.88° | 0.0121 | 0.40 | 0.309 |
| 0.85 | 7.80 | 0.214 | 292 | 8.03° | 0.0179 | 0.60 | 0.450 |
| **1.00** | **10.80** | **0.297** | **210** | **5.78°** | **0.0249** | **0.83** | **0.614** |
| 1.20 | 15.55 | 0.431 | 145 | 3.99° | 0.0361 | 1.20 | 0.862 |
| 1.40 | 21.17 | 0.591 | 106 | 2.91° | 0.0495 | 1.65 | 1.139 |
| **1.70** | **31.21** | **0.883** | **70.8** | **1.95°** | **0.0740** | **2.47** | **1.587** |
| 2.00 | 43.20 | 1.240 | 50.4 | 1.39° | 0.1038 | 3.46 | 2.049 |

`Cl_0,max = 0.4 × Travel × Cl_δ`; `σ margin = Cl_0,max / σ(Cl_0)` at
σ(Cl_0) = 0.03.

**The canard as built reaches only 0.83σ of the assumed build tolerance on
Cl_0** — roughly 40% of builds would exceed its bias authority. The smallest
size reaching 2σ is **scale 1.70: s = 20.4 mm, 31.2 cm² total**, about 2.9×
the current area.

## Area and linkage ratio are not interchangeable

Dead zone at the canard is `Deadband/n`, so \(\Delta p_{dz} \propto 1/n\) while
authority is untouched. **Only area fixes authority; only n fixes resolution.**

| configuration | n = 1 | n = 2 | n = 4 |
|---|---|---|---|
| as built, scale 1.00 | 0.614 | 0.307 | 0.154 |
| 2σ size, scale 1.70 | 1.587 | 0.793 | 0.397 |

Scale 1.70 with n = 4 beats the current design on **both** axes: 2.47σ of
authority against 0.83σ, and 0.397 rad/s of dead-zone error against 0.614.
That pairing is the candidate design point. Any n > 1 must be checked against
the slew-rate limit in the sizing bench before it is committed — the bench puts
the linkage knee at n ≈ 4–5, with slew rate the binding constraint.

## The load-bearing assumption

Everything above scales with **σ(Cl_0) = 0.03**, which is assumed, not
measured. It is the single most important number in the sizing case: the
required area is proportional to it. Before committing to a 2.9× area increase,
establish what canard rigging and fin-cant tolerance the build actually
achieves. Tightening σ(Cl_0) is very likely cheaper than tripling the canard.
