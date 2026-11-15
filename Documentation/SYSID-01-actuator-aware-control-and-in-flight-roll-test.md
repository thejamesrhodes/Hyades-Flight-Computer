# SYSID-01: actuator-aware roll control, and the in-flight roll test

**15 November 2026.** SITL study, nothing flown. **Labels:** [F] simulated or measured result, [I] inference, [S] speculation.
**Questions (James, 15 Nov):**
1. Would an MPC-like controller that knows the actuator (delay, threshold, lash, rate limit) gain much? It must keep the current property of having no aerodynamic model.
2. Simulate an in-flight step test, 0 → 2 rad/s → 0.
3. What in-flight roll schedule gets the most useful data about the system?

**Basis.**
- Plant: SITL 6DOF with the flight code compiled in, BENCH-02 servo at the corrected scale (`--servo bench02`).
- Dispersions: build tolerance (cant 0.1°, fin 0.1°, bearing ≤ 6 mN m), 600 flights per campaign (6 motors × 100, paired seeds), unless stated.
- Every number in this note is from that model: a simulated result, not a flight result.

## 1. Answers

1. **An actuator-aware controller of the MPC kind gains almost nothing here.**
   - A perfect actuator (no delay, threshold, lash or lag) improves coast roll RMS p95 by 1 %, 19.1 → 18.9 °/s [F].
   - The loop crosses over near 8 rad/s, and the servo is 15 times faster.
   - The canard limits never bind in normal flight, so MPC's constraint handling would not be used.
   - Without a canard angle measurement, the servo's landing error cannot be corrected by any controller.
2. **What does matter is the disturbance torque.**
   - Its sources are mainly:
     - bearing friction from the free-spinning fin can;
     - fin cant and body Cl₀ torques that change with q;
     - the burnout transient.
   - The current trim integrator follows it too slowly. Allowing a higher gain helps only a little: c = 22 gives 17.2 °/s.
   - **A q-scheduled extended state observer (ESO) cut coast roll RMS p95 from 19.1 to 4.2 °/s, and peak roll p95 from 106 to 41 °/s [F].** It needs only one number: canard effectiveness per unit dynamic pressure, β = S d Cl_δ / Ixx. The c/q gain already assumes that number, and no aerodynamic model is added.
   - **Actuator awareness helps inside the observer, not in an MPC.** If the observer assumes a faster servo than the real one, 1.5 % of flights go unstable at ω_o = 30 rad/s. With ω_o = 20 rad/s and a conservative (slower) actuator model, every flight passes, even with a servo twice as slow (§4).
3. **Step test 0 → 2 rad/s → 0** (§5). The current law reaches 90 % in a median 0.19 s and overshoots a median 10 % (up to 48 %).
   - Afterwards the trim is left wound up: the body keeps rolling at about 10 °/s (median, up to 27) for seconds.
   - The body drags the fin can up to 90–265 °/s through the bearing.
   - Under the ESO the same step settles to 1.7 °/s within a second, with about 1 % overshoot.
4. **Recommended schedule** (§6):
   - **Excitation:** a guarded, torque-scaled canard 3-2-1-1 followed by a ±2 rad/s rate doublet. It starts 0.25 s after detected burnout and ends by q̂ = 1.5 kPa.
   - **What one flight gives:** the closed-loop step response, the friction asymmetry, and β to about ±20 % (one sd).
   - **What limits β:** the data, not the choice of excitation.
     - The fin-can spin rate is not measured, so friction cannot be separated from control torque.
     - The log stores the canard command to only 0.24°.
     - The equation-error estimator is simple.
   - **Why ±20 % is enough:** the ESO tolerates β wrong by 2× high or about 3× low (§4).

## 2. Why an actuator-aware or MPC law has little to gain

| Item | Value | Consequence [I] |
|---|---|---|
| Roll loop crossover | ω_c = c·β ≈ 15 × 0.56 = 8.4 rad/s (1.3 Hz) at nominal β | slow next to the servo |
| Servo (BENCH-02) | ω_n 130 rad/s, ζ 0.76, dead time 9.3 ms from pulse start, plus 2.5 ms frame hold | about 11° of phase at crossover, against a phase margin of about 70° |
| Canard in normal flight | p95 of the maximum 2.9°, never at the ±12° limit (R8: 0 %) | MPC's constraint handling would not be used |
| Threshold 1.6 µs (0.15°), lash 0.12°, landing scatter 0.09–0.16° | not observable without a canard angle sensor | a predictor would only act on a model of them. The dither already linearises the threshold, and the landing error acts like a slowly changing torque bias, which a disturbance observer absorbs |

**Bounds, from 600 flights each** (`sysid_bounds.png`, [F]):

| Case | Coast roll RMS p50 / p95 / max (°/s) | Peak roll p95 / max (°/s) | All requirements |
|---|---|---|---|
| Current law, BENCH-02 servo | 9.4 / 19.1 / 34.0 | 106 / 220 | 100 % |
| … perfect actuator | 9.1 / 18.9 / 32.8 | 104 / 222 | 100 % |
| … no bearing friction | 8.8 / 17.7 / 28.8 | 95 / 165 | 98.7 % (R6: can lock-in) |
| … c = 22 | 7.9 / 17.2 / 31.8 | 90 / 158 | 100 % |
| … c = 22, perfect actuator | 7.8 / 17.1 / 30.8 | 91 / 170 | 100 % |
| … perfect actuator, no friction | 8.6 / 17.6 / 28.2 | 92 / 158 | 98.5 % (R6) |
| ESO law (§4) | 2.1 / 4.2 / 8.1 | 41 / 64 | 100 % |

- **[I]** No controller acting on the actuator alone can gain more than the perfect-actuator row: about 1 %. That remains true at c = 22.
- **[I]** Removing friction gains about 7 %, without the can coupling it brings (without friction the can spins more freely and lock-in fails rise). The large error is in how fast the slowly changing torque is cancelled.
- **[F]** An MPC is computationally feasible on the F411 (a 2-state model, a 10-step horizon). It would add tuning, a horizon and a cost function for no benefit here.

## 3. What the error is

From 12 traced flights [F]:
- the coast RMS is spread over high, middle and low q, not concentrated near apogee;
- the fin can spins at 20–170 °/s RMS, from its cant, and drags the body through the bearing.

The trim integrator in the current law is deliberately slow: Ki 0.0234, rate limited to 3°/s, with a dead band. So it lags this torque. **[I]** The fix is a fast estimate of the total disturbance, not a better model of the actuator.

## 4. The observer law (CTRL_ROLL_LAW 2, study only)

**Model and law.**
- The plant is taken as ṗ = β q̂ δ + f, with f the lumped disturbance acceleration.
- A 2-state ESO (Han 2009; Gao 2003) estimates p̂ and f̂ with bandwidth ω_o: l₁ = 2ω_o, l₂ = ω_o².
- The law is δ = (c/q̂)(p_ref − p) − f̂ / (β̂ q̂).
  - The first term is the current law.
  - The second term replaces the trim integrator.
- **Actuator model in the observer:** the observer's input is the commanded canard, delayed `CTRL_ESO_DN` + 1 frames and passed through a first-order lag `CTRL_ESO_TA`.
- **What it is not:** no Cl_p, cant, Cl₀, friction or can model is used. **[I]** It is INDI by another route: law 0 failed in HITL because its effectiveness was fixed rather than scheduled with q (ADR-004).

| Campaign (600 flights unless stated) | Coast RMS p50 / p95 / max | Peak p95 / max | Pass |
|---|---|---|---|
| ESO, ω_o 30, β̂ nominal, actuator model 15 ms + 12 ms | 2.1 / 4.2 / 8.1 | 41 / 64 | 100 % |
| … β̂ 2× too high | 2.3 / 4.7 / 8.4 | 46 / 82 | 100 % |
| … β̂ 2× too low | 2.1 / 5.5 / 249 | 36 / 296 | 99.3 %: 4 unstable, true β 3.1–3.6× β̂ |
| … slow servo (21 ms, ω_n 60, ζ 0.6) | 3.0 / 10.3 / 671 | 48 / 2117 | 98.5 %: unstable flights |
| ESO ω_o 20, BENCH-02 servo | 2.9 / 6.2 / 12.8 | 53 / 87 | 100 % |
| ESO ω_o 20, slow servo | 3.7 / 8.4 / 66 | 56 / 141 | 99.5 % |
| **ESO ω_o 20, actuator model 20 ms + 20 ms, slow servo** | 3.6 / 7.7 / 40.7 | 58 / 91 | **100 %** |
| **ESO ω_o 20, actuator model 20 ms + 20 ms, BENCH-02 servo** | 3.0 / 6.4 / 13.0 | 55 / 94 | **100 %** |
| Wide tolerance: current law / ESO ω_o 30 | 13.6 / 30.8 / 83.4 ; 3.3 / 6.7 / 31.1 | 197 / 312 ; 79 / 117 | 95.8 % ; 97.3 % (ESO fails only R6, can lock-in) |
| Hard-over, 240 flights: current law / ESO ω_o 30 | 62 / 120 / 138 ; 20 / 52 / 61 | 457 / 598 ; 375 / 533 | 27.5 % ; 66.2 % |

- **Robustness [F]:**
  - The ESO tolerates β̂ too high easily (2× costs 0.5 °/s).
  - It goes unstable when the true β is more than about 3× β̂. Start with β̂ at 1.3× nominal until flight data narrows it.
  - It is sensitive to an actuator slower than its model. A conservative (slow) model at ω_o = 20 costs almost nothing (6.2 → 6.4 °/s) and covers a servo twice as slow. **This is where the actuator model belongs.**
- **Not tested [I/S]:**
  - unmodelled structural or aeroelastic modes, and IMU mount resonance near ω_o;
  - the servo under air load;
  - gyro vibration different from the synthetic model.
  - An observer at 20 rad/s is low enough that these are unlikely to matter, but HITL and a bench run come first.

## 5. Step test 0 → 2 rad/s (1 s) → 0

**Set-up.** The step starts 0.25 s after detected burnout. Median q̂ is 4.7 kPa at the start and 3.2 kPa at the end. 120 flights; `sysid_step.png` shows one of them.

| | Current law | ESO law |
|---|---|---|
| Time to 90 %: median (5–95 %) | 0.19 s (0.10–0.81) | 0.37 s (0.23–0.66) |
| Overshoot | 10 % (−18 to +48 %) | 1 % (−7 to +9 %) |
| Error over the last 40 % of the hold | 4 % (−21 to +20 %) | 3 % (−3 to +10 %) |
| \|p\| 1 s after the reference returns to 0 | 10 °/s (1.5–27) | 1.7 °/s (0.2–5.0) |
| Peak roll p95 / max | 170 / 220 °/s | 125 / 130 °/s |
| Canard, largest in the manoeuvre | 0.86° (0.27–1.85) | 0.87° (0.27–1.79) |
| Fin-can spin, largest | 88 °/s median, 265 p95 | 91 / 265 |

**What the trace shows [F].**
- The body drags the can along through the bearing. When the reference returns to 0, the can drags the body back.
- **Current law:** the trim integrator wound up during the step, so the body holds about −12 °/s for several seconds.
- **ESO:** it settles cleanly.

**For a flight on the current law [I]:**
- use a doublet (±), not a single step, so the trim unwinds;
- or freeze the trim integrator while `p_ref` ≠ 0 and for 0.3 s after.

The rise time is a poor measure of β: it correlates only 0.38 with the truth, because friction and the trim shape it.

## 6. The schedule that tells you most

**What one flight can identify** (equation error: ṗ = β q̂ δ + k_p q̂ p/V + φ tanh(slip/0.5) + g q̂ + b):

| Term | Meaning |
|---|---|
| β | sets c and the ESO |
| φ | bearing friction |
| g, b | cant and Cl₀ torque, offset |

**Design rules [I].**
- **Information on β grows with (q̂·δ)²**, where δ must vary independently of p. In closed loop δ follows p, so an independent excitation is needed: a rate-reference change, or a canard input added after the law.
- **Size canard excitation in torque units** (δ ∝ 1/q̂). At q = 5 kPa a fixed 0.5° 3-2-1-1 drives the body to 337 °/s, above R5a. Scaled to 0.5° at 3 kPa, peak p95 is 164 °/s.
- **A ± rate doublet separates friction from the cant/Cl₀ torque.** Friction is odd in slip; the cant/Cl₀ torque is not. A single step mixes them.
- **Time window:** detected burnout + 0.25 s, while q̂ ≥ 1.5 kPa, which is about 3 s on G79W.
- **Guard:** return to the zero-rate law at once if q̂ < 1.5 kPa, |p| > 200 °/s, or any fault.

**Results** (current law, 120 flights each; `sysid_schedules.png`). β̂/β is median (5–95 %); "as logged" is today's log, "+ fine cmd" logs the command at full resolution, "+ can sensor" adds a fin-can spin sensor.

| Schedule | β̂/β as logged | + fine cmd | + can sensor | Peak roll p95 / max |
|---|---|---|---|---|
| None (natural excitation) | not identifiable | 0.33 (0.06–1.22) | 0.38 | 98 / 220 |
| Rate step +2 rad/s, 1 s | 0.74 (0.44–1.05) | 0.87 (0.59–1.11) | 0.89 (0.52–1.10) | 170 / 220 |
| Rate doublet ±2 rad/s, 0.8 s each | 0.75 (0.53–0.96) | 0.80 (0.59–1.01) | 0.91 (0.63–1.08) | 180 / 220 |
| Canard 3-2-1-1 0.5° (unscaled) | 0.85 (0.66–0.99) | 0.88 (0.68–1.02) | — | 235 / **337** |
| Canard multisine 0.4°, 2 s | 0.81 (0.58–0.99) | 0.84 (0.63–1.01) | — | 137 / 220 |
| **Combined: 3-2-1-1 torque-scaled, then doublet, guarded** | 0.79 (0.61–0.99) | 0.82 (0.64–1.03) | 0.91 (0.52–1.05) | 164 / 220 |

- **[F]**
  - **The choice of input moves β̂ by only a few per cent.**
  - **What the data records matters more:**
    - Logging the command at full resolution raises the median from 0.74–0.79 to 0.80–0.87.
    - A can-spin sensor raises it to about 0.90. With true slip, a noise-free fit is unbiased (median 0.97), so the remaining bias is friction confounded with control torque.
- **[I]**
  - The schedule still pays for itself: it gives the step response, the friction sign and size, and β to within the ESO's tolerance. That turns a design assumption into a measurement.
  - A better estimator (output error with a fin-can state, or a Kalman smoother) is the next lever after sensing.

**Recommended in-flight schedule** (coast, current law or ESO):
1. **Wait:** detected burnout + 0.25 s; start only if q̂ ≥ 3 kPa.
2. **3-2-1-1 canard input**, added after the law, unit 0.1 s (0.7 s), amplitude 0.5° × 3000 Pa / q̂ (at most 1.5°).
3. **Gap:** 0.2 s.
4. **Rate doublet:** +2 rad/s for 0.8 s, then −2 rad/s for 0.8 s, then 0. On the current law, freeze the trim while `p_ref` ≠ 0 (proposed; the simulations below did not freeze it).
5. **Abort** to zero-rate at q̂ < 1.5 kPa, |p| > 200 °/s, or any fault.

In simulation this costs 1.7–2.5 s of coast. Peak roll p95 is 164 °/s, the body/can lock-in ratio stays ≤ 0.30/0.38 (R6 0.6), and zero-rate control resumes with |p| about 11 °/s (median) 1 s later.

## 7. What to change, and decisions needed

| # | Item | Why | Status |
|---|---|---|---|
| 1 | Log the roll command at full resolution in ascent (PLAN-01 S14), and log at 200 Hz during the manoeuvre | Identification bias (§6) | Proposed |
| 2 | **Fin-can spin sensor** (Hall sensor and magnets on the bearing) | The largest single gain for identification, and an input for friction feed-forward. MC-01 FM4 dropped it for V6; revisit | Proposed |
| 3 | ESO roll law to HITL: ω_o 20, β̂ 1.3× nominal, actuator model 20 ms + 20 ms | 4–5× lower coast roll, robust in SITL | **Decision: James** |
| 4 | Test manoeuvre in the flight code (`p_ref`, `u_exc` exist; a scheduler and guard are needed) | §5–6 | **Decision: James** (which flight) |
| 5 | Output-error estimator with a fin-can state | Next step for β, φ | Later |

## 8. Files

- `Core/hyades/gnc/control.c/h`: `p_ref` (roll-rate reference) and `u_exc` (additive canard excitation), both 0 in normal flight. Law 2 (ESO) is compiled only with `CTRL_ROLL_LAW 2`. The flight build is unchanged: law 1, host tests 123/123.
- `Core/Inc/board.h`: `CTRL_ROLL_LAW` and the `CTRL_ESO_*` constants are overridable for SITL.
- `Tools/sitl/sitl.c`: `man_*` test manoeuvres and new trace columns (p_ref, u_exc, measured gyro, V, Ixx).
- `Tools/sitl/man.py`, `man_one.py`: manoeuvre Monte Carlo and identification. `sysid_campaigns.sh`: every campaign above. `sysid_figs.py`: figures.
- `Tools/sitl/runs/{O*,E*,K*}/summary.json` (and `man.csv`); `Documentation/Images/sysid01/`.

**References.**
- Han, J. (2009). From PID to active disturbance rejection control. *IEEE Trans. Industrial Electronics* 56(3), 900–906. https://doi.org/10.1109/TIE.2008.2011621
- Gao, Z. (2003). Scaling and bandwidth-parameterization based controller tuning. *Proc. American Control Conference*, 4989–4996. https://doi.org/10.1109/ACC.2003.1242516
- Smeur, E. J. J., Chu, Q. P., de Croon, G. C. H. E. (2016). Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles. *J. Guidance, Control, and Dynamics* 39(3), 450–461. https://doi.org/10.2514/1.G001490
- Morelli, E. A., Klein, V. (2016). *Aircraft System Identification: Theory and Practice*, 2nd ed. Sunflyte Enterprises. ISBN 978-0-9974306-1-5. (Input design, 3-2-1-1 and multisine inputs, equation-error identification.)
