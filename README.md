<p align="center">
<img src="Images/OpenRocket_1.png" width="884">
</p>

---

**Hyades Flight Computer**

Custom STM32H743 flight computer and GNC software for an actively roll-controlled high-power rocket.
*This is an engineering project, not a product. The safety case, regulatory compliance, and airworthiness of any vehicle built from this material are the builder's sole responsibility.*

The system is designed to provide:
- High sample rate inertial measurement and state estimation
- Real-time flight control
- 6 PWM servo connectors, currently with 4 being used for canard-based active roll stabilisation
- In flight data logging and optional in-flight telemetry

This repository contains the hardware design, embedded software, simulation models, and analysis tools used during development.

---

**Project Progress**

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

The main PCB, containing all the computation and data gathering.

**Flight Computer Design**
<p align="center">
<img src="Images/PCB_MCU.png" width="884">
</p>

---

The PDB board, provides power and distributes PWM signals to the servos, as well as providing a clean 3V3 to the sensor board.

<p align="center">
<img src="Images/PCB_PDB.png" width="900">
</p>

**Flight Computer Testing**

0 Ohm Series Termination vs 47 Ohm Series Termination for the 24Mhz HSE Oscillator, unnecessary? Yes, but a good opportunity to learn SPICE Simulations, plus it looks cool.

<p align="center">
<img src="Simulation/HSE Clock SPICE Simulation/Figures/0 Ohm Series Termination Figure.png" width="900">
</p>

---

<p align="center">
<img src="Simulation/HSE Clock SPICE Simulation/Figures/47 Ohm Series Termination Figure.png" width="900">
</p>

---

The schematic that the simulations where derived from.

<p align="center">
<img src="Simulation/HSE Clock SPICE Simulation/Figures/Schematic Figure.png" width="900">
</p>

**Flight Hardware Design**

Full assembly, front section and nose cone are missing in action.

<p align="center">
<img src="Images/Full_Assembly.png" width="900">
</p>

---

Spinning Rear Fin Can designed to avoid the roll reversal phenomenon where upstream control surfaces tip vortices interact with the downstream fixed fins, causing a difficult to predict moment in the opposite direction.

<p align="center">
<img src="Images/Rear_Fin_Can_Assembly.png" width="900">
</p>

---

<p align="center">
<img src="Images/Front_Avionics_Assembly.png" width="900">
</p>

**Flight Hardware Testing**



**Flight Software**



**Simulink Roll Model**



**Simulink 6DOF Model**



**MATLAB Monte Carlo Analysis**

Demonstrating a <0.5s roll rate settling time for a step response.

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_1.png" width="900">
</p>

---

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_2.png" width="900">
</p>

---

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_3.png" width="900">
</p>

---

Visualises the correlation structures between convergent roll damping and the different parameters.

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Step_Response_Figure_4.png" width="900">
</p>

**MATLAB Gain Optimisation**

Heatmaps where each square is a n=400 montecarlo simulation at the corresponding proportional and integral gains plotted against some measure relevant to system performance (note gains are q normalised as follows: Kp = c/q  and  Ki = Kp*Ti, where q is dynamic pressure (Pa), c and Ti are the user configurable parameters.).


<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_5.png" width="900">
</p>

---

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_6.png" width="900">
</p>

---

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_7.png" width="900">
</p>

---

<p align="center">
<img src="Simulation/Matlab Roll Tuning/Figures/Heatmap_Figure_8.png" width="900">
</p>

**System Integration**



**HITL Testing**



**Flight Testing**


