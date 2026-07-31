<p align="center">
<img src="Images/OpenRocket_1.png" width="884">
</p>

---

# Hyades Flight Computer

Custom STM32H743 flight computer and GNC software for an actively roll-controlled high-power rocket.

![Status](https://img.shields.io/badge/status-active%20development-orange)
![MCU](https://img.shields.io/badge/MCU-STM32H743-blue)
![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-red)
![KiCad](https://img.shields.io/badge/KiCad-10-blue)

*This is an engineering project, not a product. The safety case, regulatory compliance, and airworthiness of any vehicle built from this material are the builder's sole responsibility.*

The project is designed to:
- Provide a modular active roll-stabilisation system suitable for a wide range of high-power rockets.
- Improve flight data quality by reducing excessive vehicle roll.
- Improve parachute deployment reliability by limiting roll-induced line twist.
- Provide a reusable avionics platform for future guidance, navigation and control research.
- Maintain compliance with applicable rocketry regulations through both hardware and software design decisions.
- Release all hardware, firmware and simulation tools as open source.

This repository contains the hardware design, embedded software, simulation models, and analysis tools used during development.

---

## Specification

## Specification

| Parameter | Specification |
|-----------|---------------|
| Purpose | Active roll stabilisation for mid- and high-power rockets |
| Flight Computer | STM32H743 ARM Cortex-M7 |
| Control Surfaces | Four independently actuated canards |
| Control System | Configurable closed-loop roll controller with dynamic-pressure gain scheduling |
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
| MATLAB Monte Carlo Analysis     | ✅ Complete |
| MATLAB Gain Optimisation        | ✅ Complete |
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

## Simulink Roll Model

%words

---

## Simulink 6DOF Model

%words

---

## MATLAB Monte Carlo Analysis

Montecarlo (n = 4000) Demonstrating a <0.5s roll rate settling time for a step response with the current optimal gainset.

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_1.png" width="300">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_2.png" width="300">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_3.png" width="300">
</p>

Visualises the correlation structures between convergent roll damping and the different parameters.

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_4.png" width="500">
</p>

---

## MATLAB Gain Optimisation

Controller gains are parameterised using dynamic-pressure-normalised coefficients to maintain approximately constant closed-loop behaviour throughout ascent. A grid search evaluates each gain combination using 400 Monte Carlo simulations and scores them according to settling time, overshoot, actuator demand and stability margin. (note gains are q normalised as follows: Kp = c/q  and  Ki = Kp*Ti, where q is dynamic pressure (Pa), c and Ti are the user configurable parameters.).

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_5.png" width="300">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_6.png" width="300">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_7.png" width="300">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_8.png" width="300">
</p>

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
