# QMI8658A noise characterisation (FC-V4)

## 20261103-201033 — 2.46 h static, bench

- **Setup:** board flat, untouched, room about 19 °C, USB direct. The die temperature went from 26.2 to 27.5 °C.
- **Sensor configuration (flight):**
  - ±16 g, ±2048 dps.
  - ODR 224.2 Hz, sensor LPF 13.37 % of ODR (about 30 Hz).
  - CTRL1–CTRL7 = `40 35 75 00 77 00 03`.
- **Firmware:** build `46390eb2+49f9dd`.
- **Data:** 1 982 674 samples, 0 lost.
- **Raw data:** `Tools/logs/imucap/20261103-201033.imu` (35.7 MB, not in git).
- **Analysis:** `Tools/fc_noise.py`. The full result is in `20261103-201033_noise.json`; the figures are `*_adev.png`.

| | x | y | z | Unit |
|---|---|---|---|---|
| Gyro ARW (τ 0.1–1 s) | 7.71 | 8.51 | 7.37 | mdps/√Hz (datasheet 13) |
| Quantisation share of ARW variance | 2.4 | 2.0 | 2.7 | % |
| ρ = σ/LSB (per sample) | 1.12 | 1.20 | 0.93 | — |
| Gyro bias instability (min ADEV / 0.664) | 13.5 @ 31 s | 8.3 @ 154 s | 16.6 @ 62 s | deg/h |
| Long-τ envelope K (τ ≥ 100 s) | 160 | 58 | 106 | deg/h/√h |
| Gyro bias tempco (1-min means) | +0.023 | +0.008 | 0.000 | dps/°C |
| Accel VRW | 66.2 | 67.1 | 84.0 | µg/√Hz |
| Accel per-sample σ | 0.56 | 0.60 | 0.65 | mg |
| Accel bias instability | 53 | 83 | 105 | µg |

**Reading the curves**

- Below about 0.1 s the ADEV rises to a hump near 20 ms instead of following τ^-1/2. This is the sensor's own LPF correlating successive samples, so that region is excluded from the read-out and the fit.
- The white region from 0.1 to about 10 s has a clean slope of -1/2.
- Rate-register quantisation is white in the rate output, so it adds to this slope-(-1/2) level. It is not the slope of -1 seen for angle quantisation.
- With ρ ≈ 1 the sensor sits at the knee of η(ρ): η ≈ 1, and Δ²/12 is a valid estimate of the quantisation share. On z (ρ = 0.93) the Sheppard correction is borderline.
- The long-τ rise on x tracks the 1.3 °C temperature rise (tempco +0.023 dps/°C × 1.3 °C ≈ 0.03 dps ≈ 110 deg/h). So the fitted RRW on x is mostly thermal.
- z (roll, board flat) shows a flicker floor of about 11 deg/h from 20 to 1000 s and essentially no tempco.
- The WN+GM+RW model fits x and y well. It cannot represent the flat flicker floor on z: the fitted RW over-predicts there, so the envelope K is used for the filter rather than the fit.

**IEKF values set from this run (`board.h`)**

- `IEKF_Q_ANGLE` = 1.49e-4 rad/s/√Hz. This is the worst-axis measured ARW, including quantisation.
- `IEKF_Q_BIAS` = 1.29e-5 rad/s/√s. This is the worst-axis long-τ envelope, and it includes thermal drift on purpose.
- `IEKF_R_ACCEL` = 1e-3. This is the sensor floor (4.3e-7) plus 0.03 g of unmodelled pad acceleration. Larger transients are rejected by the |a| and NIS gates. The bench and HITL H1 should confirm it.

**Caveats**

- 2.46 h is enough for ARW and bias instability, and gives only a few independent samples at τ ≥ 1000 s. RRW and thermal drift at that end are indicative only.
- This is one unit at one temperature. A run in a controlled cold environment would separate the thermal drift from RRW.
