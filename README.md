<p align="center">
<img src="Images/OpenRocket_1.png" width="884">
</p>

---

# Hyades Flight Computer

Custom STM32H743 flight computer and GNC software for a roll-rate-stabilised high-power rocket.

![Status](https://img.shields.io/badge/status-active%20development-orange)
![MCU](https://img.shields.io/badge/MCU-STM32H743-blue)
![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-red)
![KiCad](https://img.shields.io/badge/KiCad-10-blue)

*This is an engineering project, not a product. The safety case, regulatory compliance, and airworthiness of any vehicle built from this material are the builder's sole responsibility.*

The project is designed to:
- Provide a modular active roll-rate damping system suitable for a wide range of high-power rockets.
- Improve flight data quality by reducing excessive vehicle roll.
- Improve parachute deployment reliability by limiting roll-induced line twist.
- Provide a reusable avionics platform for future guidance, navigation and control research.
- Maintain compliance with applicable rocketry regulations through both hardware and software design decisions.
- Release all hardware, firmware and simulation tools as open source.

**Control scope.** The flight controller closes a loop on roll *rate*, not roll *angle*. It damps rate transients and cancels the standing roll moment produced by build asymmetry. It holds no commanded heading and has no outer attitude loop. Roll-angle hold and full three-axis attitude control are planned work and are not implemented.

This repository contains the hardware design, embedded software, simulation models, and analysis tools used during development.

---

## Specification

| Parameter | Specification |
|-----------|---------------|
| Purpose | Active roll stabilisation for mid- and high-power rockets |
| Flight Computer | STM32H743 ARM Cortex-M7 |
| Control Surfaces | Four independently actuated canards |
| Control System | Proportional roll-rate damper, dynamic-pressure gain scheduled, with a slow bounded trim path for bias rejection |
| Control Scope | Roll rate damping and bias rejection. No attitude loop |
| Interfaces | I²C, UART and 6× PWM outputs |
| Data | On-board logging with optional telemetry |
| Simulation | MATLAB, Simulink, 6DOF modelling and Monte Carlo analysis |
| Architecture | Modular flight computer, sensor board and power distribution board |
| Design Goals | Low mass, modular, reliable and adaptable |
| License | Apache 2.0 |

---

## Project Progress

| Feature                         | Status |
|---------------------------------|:------:|
| Flight Computer Design          | ✅ Complete |
| Flight Computer Testing         | 🚧 In Progress |
| Flight Software                 | 🚧 In Progress |
| Flight Hardware Design          | 🚧 In Progress |
| Flight Hardware Testing         | 🚧 In Progress |
| Simulink Roll Model             | ✅ Complete |
| Simulink 6DOF Model             | ✅ Complete |
| MATLAB Monte Carlo Analysis     | 🚧 In Progress |
| MATLAB Gain Optimisation        | 🚧 In Progress |
| System Integration              | ⏳ Not Started |
| HITL Testing                    | ⏳ Not Started |
| Flight Testing                  | ⏳ Not Started |

---

## Repository Structure

```text
Hyades-Flight-Computer
│
├── Firmware/
├── Hardware/
│   ├── PDB/
│   └── MCU Board/
├── Simulation/
│   ├── Buck Regulator/
│   ├── HSE Clock/
│   └── Roll Tuning/
├── Images/
└── Documentation/
```

---

## Flight Computer Design

The main PCB, containing all the computation and data gathering.

<p align="center">
<img src="Images/PCB_MCU.png" width="500">
</p>

The PDB board, provides power and distributes PWM signals to the servos, as well as providing a clean 3V3 to the sensor board.

<p align="center">
<img src="Images/PCB_PDB.png" width="500">
</p>

---

## Flight Computer Testing

0 Ohm Series Termination vs 47 Ohm Series Termination for the 24Mhz HSE Oscillator, unnecessary? Yes, but a good opportunity to learn SPICE Simulations, plus it looks cool.
Below the schematic that the simulations where derived from.

<p align="center">
<img src="Simulation/HSE Clock SPICE Simulation/Figures/0 Ohm Series Termination Figure.png" width="300">
<img src="Simulation/HSE Clock SPICE Simulation/Figures/47 Ohm Series Termination Figure.png" width="300">
<img src="Simulation/HSE Clock SPICE Simulation/Figures/Schematic Figure.png" width="600">
</p>



---

## Flight Hardware Design

Full assembly, front section and nose cone are missing in action.

<p align="center">
<img src="Images/Full_Assembly.png" width="500">
</p>

Spinning Rear Fin Can designed to avoid the roll reversal phenomenon where upstream control surfaces tip vortices interact with the downstream fixed fins, causing a difficult to predict moment in the opposite direction.
Front avionics assembly, WIP
<p align="center">
<img src="Images/Rear_Fin_Can_Assembly.png" width="300">
<img src="Images/Front_Avionics_Assembly.png" width="300">
</p>

---

## Flight Hardware Testing

%words

---

## Flight Software

%words

---

# Simulation

---

## Roll Control Architecture

The loop closes on roll rate, in two deliberately separated paths.

**Damping.** A proportional rate damper, gain scheduled on dynamic pressure, `Kp = c / q_e`. The schedule holds the open-loop *crossover* constant, `wc = c*k_b` with `k_b = A*d*Cl_delta / Ixx`. Crossover is the right invariant to target rather than closed-loop bandwidth, because the open-loop roll pole `a = k_a*q/V` varies across the envelope by orders of magnitude wherever passive roll damping is strong. Active control earns its place at the low-q, high-V corner, where the airframe is close to undamped; at high q and low V it damps itself and the loop is near idle.

**Trim.** Proportional damping cannot reject a constant disturbance. The disturbance here is a standing roll moment `Cl_0` from fin cant, canard rigging error and body asymmetry. Adding an integrator to the damper is exactly what generates a limit cycle against the servo dead zone, so bias rejection is separated into a slow bounded trim path instead.

That separation is correct rather than a workaround, because the deflection which cancels a standing bias,

`delta_ff = -Cl_0 / Cl_delta`

is independent of dynamic pressure and airspeed: disturbance and control authority both scale with q, so one fixed deflection cancels the bias at every flight condition. It is a constant to be learned once, not a signal to be tracked. The trim path can therefore run an order of magnitude slower than the damper, and being that slow is what keeps it out of the loop dynamics.

The trim integrator is bounded by four gates: a bandwidth set well below the stick-slip frequency; a dead zone *subtracted from* the integrator input rather than switching the integrator, so the integration rate falls smoothly to zero; a saturation freeze; and a clamp with a rate limit. Subtracting the band matters - a dead zone wrapped *around* an integrator is the same stick-slip generator at lower frequency. Low-pass filtering the error ahead of it adds phase lag and creates new hunting points.

The path accepts an external `Cl_0` estimate, so a future MEKF can supply it directly while keeping the saturation and clamp gates. Without a canard position encoder an estimator cannot separate a genuine aerodynamic bias from actuator hang-off, so it needs the same bounding.

**Dead zone and dither.** The servo dead zone, the command quantiser and gear lash together form a lattice, and whether the loop hunts depends on where the equilibrium deflection happens to land on it. This is the classic limit cycle of an integrating element driving an actuator with a dead band - the same pathology as valve stiction in process control, where the established cures are to remove the integrator, to add dither, or to fix the actuator.

The dead zone sits inside the servo's *own* position loop, not on the command path, which is why command-side dead-zone compensation does not touch it. Dither does: injected on the command, it sweeps the servo's internal position error across the dead zone continuously and smears the threshold, and the second-order actuator attenuates it far too heavily at that frequency for the surface to follow. The flown setting is 1.5x the modelled dead zone at 125 Hz.

**Known open defect.** The trim path's saturation gate tests the pre-dither command while the travel clamp acts on the post-dither command, so within one dither amplitude of the stop the gate can miss a saturation event. The fix is to include the dither amplitude in the gate threshold. It is not yet applied.

---

## Simulink Roll Model

%words

---

## Simulink 6DOF Model

%words

---

## MATLAB Monte Carlo Analysis

Each design point is evaluated over 400 dispersed runs. Aerodynamic coefficients, vehicle properties, actuator parameters and the disturbance bias are sampled per run. The controller's dynamic-pressure estimate is drawn about the *sampled* truth rather than the nominal - a 20% 1-sigma relative error on `q_e` multiplying the true `q` - so scheduling error compounds with aerodynamic dispersion the way it would in flight rather than cancelling against it.

Scoring is in the rate domain, with transient and steady behaviour reported separately. The loop has a large, benign and physically unavoidable transient while the trim converges, during which the surface saturates, so a peak-over-the-whole-run metric reports "saturated" for every candidate design and tells you nothing about any of them.

| Metric | Definition |
|---|---|
| `rms_err` | RMS of (p - ref) over the steady tail |
| `bias` | mean(p) - ref over the tail; signed hang-off alone |
| `ripple_pp` | max(p) - min(p) over the tail; limit-cycle amplitude alone |
| `activity` | RMS surface rate; catches stick-slip, which no amplitude metric does |
| `growth` | RMS of the last half of the tail over the first half; the stability test |
| `travel_peak` / `travel_steady` | Fraction of mechanical travel used over the whole run / over the tail |

`growth` rather than magnitude is the divergence test, because magnitude is dominated by the transient at high q whatever the gains are.

Dither amplitude was swept over four campaigns at identical dispersion. Limit-cycle amplitude falls sharply between no dither and the flown setting, with no increase in surface activity - the hunting the dither removes was costing more motion than the dither adds. The residual cost appears as a small steady rate bias rather than as oscillation.

An earlier peak-to-peak roll-*angle* metric has been withdrawn. It is the detrended integral of rate over the final 30% of the run and carries a resolution guard that cannot distinguish "has not converged" from "nothing is oscillating", so it is not a sound basis for a published figure.

<!-- FIGURE SLOT A: settling-time and rate-error distributions, regenerated against
     the current canard geometry and the flown gainset. -->

<!-- FIGURE SLOT B: parameter-vs-outcome correlation structure, regenerated. -->

---

## MATLAB Gain Optimisation

The damping gain is parameterised as a dynamic-pressure-normalised constant, `Kp = c / q_e`, so that a single value of `c` holds the open-loop crossover constant across the flight envelope. `c` is set analytically from a crossover target of 4 Hz, placed below the actuator's natural frequency; the numerical job is then to confirm that stability margins around that value survive parameter uncertainty, not to search for an optimum.

Margins are evaluated directly from the open-loop frequency response, with the transport delay treated exactly rather than approximated, over:

- a set of *paired* (q, V) flight conditions taken along the trajectory, spanning q = 2-40 kPa
- independent +/-30% variations on control effectiveness `Cl_delta` and roll damping `Cl_p`
- acceptance at phase margin >= 35 deg, gain margin >= 6 dB and peak sensitivity <= 2.0, taken at the worst corner of that set

Pairing the flight conditions rather than taking the outer product of a `q` list and a `V` list matters more than it sounds. Most of that outer product is not flyable - it implies air densities the vehicle never sees - and the corner that used to size the gain implied roughly 35 km of altitude. Sizing a controller against a condition the vehicle cannot reach costs real gain everywhere it can.

Aerodynamic and actuator constants are derived from the single parameter file rather than restated in the optimiser, after an earlier version carried its own stale copy of `Cl_delta` and returned a schedule constant 2.25x too low as a result.

A separate grid search over `c` and the trim parameters scores candidates on settling time, steady-state hang-off, limit-cycle amplitude and actuator demand, using the Monte Carlo campaign above at each grid point.

`Ti` is retained only for the legacy PI mode kept in the model for A/B comparison. It has no effect in the flown configuration, where bias rejection comes from the bounded trim path rather than from integral action in the damper.

<!-- FIGURE SLOT C: stability-margin and failure-probability maps over c, regenerated
     against the current canard. The previously published heatmaps swept c against Ti
     for the legacy PI controller and the earlier canard geometry, and are not
     representative of the flown loop. -->

---

## System Integration

%words

---

## HITL Testing

%words

---

## Flight Testing

%words

---
