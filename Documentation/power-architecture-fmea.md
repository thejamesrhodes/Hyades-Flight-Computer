# Avionics power architecture and FMEA (PDB rev 3 + MCU board)

Values were read from schematic screenshots (2026-10-18).

**Status (rev 4, 2026-10-18):**
- **Fixed:** P1 (MAX40200 added on the PDB), P2 (MCU-board USB LDO changed to the AP2115R5-3.3), and C27 reduced from 47 µF to 10 µF, which brings +3V3 to about 43 µF, under the MAX40200's 100 µF limit.
- **Reported fixed:** P3 (buzzer moved to VDD_Buck). Confirm this, because the screenshot still shows VBAT_Safe.
- **New:**
  - N1: LED5 has an orphaned label. R20 connects to `3V3_AP2112K`, but the LDO output is now named `3V3_LDO`.
  - N2: C11 is across the buzzer. Move its lower end to GND.
  - N3: the PDB MAX40200 needs at least 0.33 µF locally on its OUT pin.
- **Retracted:** the IMU reserved-pin item in §5.

**Verdict:** the architecture is sound. The pieces that are right:
- the battery feed with a fused split into servo and logic branches
- logic isolated from servo sag (D2, then the buck, then the LDO)
- USB → LDO → ideal diode onto the shared 3.3 V rail

There is one structural gap: the 3.3 V OR-ing only has an ideal diode on the USB side (P1). The other material issues are the USB LDO's temperature (P2), a reverse-battery path around D2 (P3), and no hold-up time if the battery connection drops (P5).

## 1. Combined power map

```
XT30 2S (6.0–8.4 V)
 ├─ F1 (4 A) ─ VBAT_Servo ─ RC damper ─ J1: 4× BMS-127WV+ (5.2 A all stalled)
 └─ F2 (1–2 A) ─ VBAT_Safe ─┬─ D1 SMAJ12CA
                            ├─ R5/R11 ─ VBAT_Sense ─ BtB ─ PA6 ADC (3.0 V at 8.4 V)
                            ├─ J4 buzzer + D3/R1 ─ Q1          ← move after D2 (P3)
                            └─ D2 SS34 ─ VDD_Buck ─ TPS62135 ─ +4V0 ─ AP2115 ─┬─ GPS J2, LED2
                                                                  no reverse block │ (P1)
                                                                        BtB 4× +3V3 / 6× GND
USB-C VBUS 5 V ─ USBLC6 ─ AP2112K ─ MAX40200 ──────────────────────── +3V3 (shared) ─ H743, IMU, baro,
                                                                         flash, CN3 I²C, CN4 UART/LoRa
```

## 2. Operating modes

| Mode | What supplies +3V3 | Status |
|---|---|---|
| Battery only | AP2115 | ✓ The MAX40200 stops +3V3 feeding back into the AP2112K |
| USB only | AP2112K → MAX40200 | ✗ USB feeds backwards into the PDB through the AP2115 (P1). The servos are unpowered but still receive PWM (P7) |
| Both connected | Whichever LDO's output is higher | ✓ No conflict: neither LDO can sink current, and the MAX40200 blocks reverse current. USB may end up carrying the whole load (P2) |
| Plugging or unplugging USB or battery while the other is connected | — | ✓ Glitch-free once both supply paths have an ideal diode |

## 3. Findings

**P1. OR-ing is one-sided (medium).** The AP2115 datasheet doesn't specify reverse-current blocking. A CMOS LDO's internal body diode conducts when its output is higher than its input (that's my inference, not a datasheet statement). So with USB only, +3V3 back-feeds +4V0 to about 2.7 V, and from there the TPS62135's output while its input is unpowered. The current is small, but neither part is characterised for that state.

Fix: add a second MAX40200 between AP2115 OUT and the PDB's +3V3 net.
- The existing C5 (10 µF) meets the MAX40200's requirement of at least 0.33 µF on its input.
- The GPS stays powered on USB, so the MCU's UART TX can't back-power it.
- Cost: the SOT-23-5 drops 32–65 mV at 0.1 A and 46–90 mV at 0.2 A, so the rail sits at about 3.2 V. That's within range for every load; the flash (2.7 V minimum) is the tightest.

Constraint: the MAX40200's output capacitance must be between 0.3 µF and 100 µF. +3V3 already carries about 78 µF nominal, 47 µF of which is C27. Reduce C27 to 10–22 µF; the LoRa module has its own decoupling.

**P2. USB LDO runs hot (medium).**
$$\Delta T_J = \theta_{JA}\,(V_\text{BUS}-3.3)\,I$$
where \(\Delta T_J\) is the junction temperature rise (K), \(\theta_{JA}\) = 184 K/W for the AP2112K in SOT-23-5, \(V_\text{BUS}\) ≈ 5.0 V, and \(I\) is the +3V3 load current (A).
- At 0.2 A the rise is about 63 K; at 0.3 A, about 94 K.
- LoRa transmit bursts (around 0.12 A for an SX1276-class module at +20 dBm; check your module) push the junction towards the 160 °C thermal shutdown.
- This applies whenever USB's LDO is the higher of the two outputs, not only in USB-only mode.

Fix: swap U5 to the AP2115R5-3.3 (SOT-89-5, 1 A, same 6.5 V absolute maximum). It's already on the PDB's BOM, so this also consolidates parts. Measure the +3V3 current with the H743 at 480 MHz to confirm; the MCU dominates, and I couldn't verify the DS12110 current figures.

**P3. Reverse battery leaks around D2 (medium).** The XT30 is keyed, so this only matters with a miswired battery lead. J4 and D3 sit on VBAT_Safe, upstream of D2. With the pack reversed, current flows GND → Q1's body diode → D3 (forward-biased) → VBAT_Safe → F2. Q1's body diode is rated only about 0.1–0.3 A, so it fails before the time-lag F2 opens.

Fix: supply J4 from VDD_Buck instead (the buzzer then sees about 7–8 V). The servo branch still has no reverse protection, which hurts both the servos and any polarised damper capacitor. Optionally, a single P-channel MOSFET for reverse protection at the input would cover both branches for a few milliohms of loss.

**P4. USB detection is broken by LED4 (low).** LED4 + R15 hang on the R13/R14 divider node. The divider's equivalent source is 3.76 V through 24.8 kΩ, so the LED clamps the node to about 1.6–1.8 V. PA9 then sits at an indeterminate logic level, and the LED gets about 80 µA (invisible).

Fix:
- Delete LED4 and R15. LED5 already shows USB power, once N1 is fixed. If you want a separate VBUS LED, connect it from VBUS with 1.5 kΩ, not from the divider midpoint.
- PA9 then sees 3.76 V, which a 5 V-tolerant pin handles. Read it as a GPIO, with the hardware VBUS sensing disabled.
- VBUS and +5V are two names for the same net. Pick one, and keep it off the board-to-board connector, since the PDB no longer has a +5 V rail.

**P5. No hold-up if the battery connection drops (medium).**
$$t_\text{hold}=\frac{C\,(V_1^2-V_2^2)}{2P}$$
where \(C\) is the bulk capacitance on VDD_Buck (F), \(V_1\) ≈ 7.4 V and \(V_2\) ≈ 3.6 V are the starting voltage and the buck's minimum workable input (V), and \(P\) ≈ 1.4 W is the logic input power.
- With C3 (10 µF) this gives about 0.15 ms; 470 µF would give about 7 ms.
- So any brief XT30 interruption (vibration, or ejection and recovery shock) resets the avionics.
- The primary mitigation is mechanical: strain relief on the battery lead and retention on the connector.

Firmware implications:
- The H743's VBAT pin is tied to +3V3, so backup SRAM and the RTC survive watchdog resets but not a loss of power. The software guideline's "read the backup SRAM fault log on reset" only covers watchdog resets.
- After a power-loss reset, boot must not assume the rocket is on the pad. Work out the flight phase from the sensors. FDS-01 currently only defines LOCKOUT for resets during the pad sequence.

**P6. Foldback LDOs and external connectors (medium).** Both the AP2112 and AP2115 fold back to 50 mA when their output is shorted to 0 V. A short on any harnessed +3V3 line (GPS J2, CN3 or CN4) therefore collapses the shared rail and resets all the avionics. The LDOs themselves survive.
- Priority: medium-low. The minimum response is harness discipline: keyed connectors, strain relief, unplug unused breakouts for flight, and a short check on each harness.
- The hardware option is one current-limited switch per board, feeding that board's external connector supplies. An example is the TI TPS2553D: 2.7–6.5 V input, current limit set by one resistor anywhere from 75 mA to 1.7 A, 85 mΩ on-resistance, SOT-23-6. It holds a fault at the limit and blocks reverse current. Set the limit to about 250–300 mA.
- Also check that +3V3 starts up with every module attached. Foldback LDOs can stall into a heavy resistive load.

**P7. Servos receive PWM while unpowered (low).** In USB-only mode, or with F1 open, the PWM outputs back-drive the servo inputs through the 220 Ω resistors, at about 12 mA per line.
- Firmware: hold the PWM pins low unless the battery voltage is healthy.
- Hardware (cheap): add a divider from VBAT_Servo to a spare ADC pin. That gives you an F1 and servo-bus health signal for READY gating.
- The firmware rule is only hygiene; it doesn't change flight outcomes. The servo-rail sense does. VBAT_Sense taps the supply after F2, so a blown F1 or a disconnected servo supply is otherwise invisible. Hobby servos have no position feedback, and the IMU can't see a canard deflection on the pad, so FDS-01's arm-time polarity self-test can't detect a dead servo bus.

**P8. Endurance on the pad.** The logic draws about 1.4 W from the pack, which is about 0.2 A. That gives roughly 2 h from 400 mAh usable (80 % of the 500 mAh pack). If the servos hold position at about 20 mA each (an assumption), that falls to about 1.4 h. Set the FDS-01 "battery below flight-completion margin" threshold from the measured draw.

Easy reductions: hardware changes save only a few mA. The largest is raising R8 (LED2 on the PDB +3V3, about 6 mA) to 1–2.2 kΩ, which saves about 3 mA at the battery. The firmware is where the current goes:
- Put the core to sleep (WFI) between control-loop ticks.
- Run a lower clock on the pad until arming.
- Stop servo PWM on the pad until arming, after checking whether your servos keep holding position or go limp without pulses.
- Measure the pad current first.

**P9. USB hot-plug (low).** There's 9.4 µF of ceramic capacitance on VBUS. On a long cable, AN88-style ringing could approach the LDO's 6.5 V absolute maximum; the USBLC6's VBUS clamp reduces this. Check one plug-in on a scope.

**P10. Brownout hygiene (low).**
- Set the H743 brownout reset (BOR) to its highest level, 2.7 V (the configurable range is 2.1–2.7 V).
- Add a 10 kΩ pull-up on the flash CS line. The W25Q128JV datasheet (rev M, §4.1) says "/CS input must track the VCC supply level at power-up and power-down… If needed a pull-up resister on the /CS pin can be used." The H743's GPIOs are high-impedance from reset until firmware configures them, so here it is needed.
- Make the log format survive a power cut in the middle of a write.

## 4. FMEA (single faults)

| Fault | Effect | Detection | Mitigation |
|---|---|---|---|
| Battery lead or XT30 interruption | Reset (if over ~0.15 ms) or total loss | Boot logic detects a reset in flight | Mechanical retention; P5 |
| F2, D2 or the buck/AP2115 failing open | Avionics dead | — | Single power string, accepted |
| F1 open | Canards dead, avionics still running | VBAT_Servo sense (P7) | Size F1 from the stall calculation |
| D2 short | Latent: reverse-polarity protection and servo-sag isolation lost | None | Inspection |
| D1 or C3 short | F2 clears, avionics dead | — | Intended behaviour |
| Buck high-side FET short | 8.4 V on +4V0 exceeds the AP2115's 6.5 V maximum; risk of the fault cascading onto +3V3 | — | Optional TVS crowbar on +4V0 (standoff ≥ 5 V) so that F2 opens |
| Short on an external +3V3 harness | Rail collapses, avionics reset | Reset log | P6 |
| One board-to-board +3V3 or GND pin fretting | No effect (4 +3V3 pins and 6 GND pins in parallel) | — | — |
| All four servos stalled | Pack sags 0.26–0.62 V; logic unaffected (still above the buck's ~3.6 V minimum input) | — | ✓ |
| Battery over-discharged on the pad | Brownout | VBAT_Sense | FDS LOCKOUT threshold |

## 5. Non-power items on the MCU sheet

- **IMU reserved pins: retracted.** The DS-000347 rev 1.6 pin table lists pins 2, 3, 10 and 11 as "No Connect or Connect to GND", and pin 7 as "Connect to GND". Tying them all to GND, as drawn, is correct. The earlier H3 finding was wrong.
- **PPS LED.** A blue LED's forward voltage (about 2.9–3.1 V typical) leaves only 0.1–0.3 V across R10 on a 3.2 V rail. It will be nearly dark and vary unit to unit. Use a green, red or orange LED.
- **H11/H12.** These have no nets assigned yet. Mirror the PDB pinout exactly, and run the connector pin-matching check.

## 6. Bench checks

1. Measure the +3V3 current at 480 MHz with all peripherals running. This sizes P2 and P8.
2. Check that +3V3 starts up with all modules attached, on both the USB and battery paths.
3. Scope the XT30 and USB hot-plug transients.
4. Pull the battery with USB attached, and pull USB with the battery attached. Neither should cause a reset.

## Sources

- AP2115 datasheet: https://static.chipdip.ru/lib/362/DOC011362831.pdf
- AP2112 datasheet: https://cdn.sparkfun.com/assets/2/e/d/f/2/AP2112_datasheet.pdf
- MAX40200 datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX40200.pdf
- STM32H743VI datasheet (BOR levels): https://shop-archive.boxtec.ch/pub/sparkfun/STM32H743VI_Datasheet.pdf
- Analog Devices AN88 (hot-plug ringing): https://www-cce.cldnet.analog.com/media/en/technical-documentation/application-notes/an88f.pdf
- W25Q128JV datasheet, rev M: https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/7161/256_W25Q128JV.pdf
- ICM-42688-P datasheet, DS-000347 rev 1.6: https://pcbsync.com/wp-content/uploads/2026/07/ICM-42688-P-Datasheet-TDK.pdf
- TI TPS2553D: https://www.ti.com/product/TPS2553D
- Project documents: FDS-01; Software design guidelines; CDS-01; flight_computer_pcb_review.md
