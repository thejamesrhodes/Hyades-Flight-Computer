# Hyades Avionics Rev 0.3 — Schematic Review

**Scope:** PDB Board 0.3 (sch 22 Oct 11:55, pcb 11:59) and MCU Board 0.3 (sch 11:53, pcb 11:52), now under `Hyades-Flight-Computer/Hardware/PCB Designs/`. Netlists were parsed from the live `.kicad_sch` files, and PCB pad nets were cross-checked: 544 labelled pads, 0 mismatches.

**Reviewed:** 2026-10-22. This review lists open items only.

**Companion reviews:** `claude/cubemx-config-review.md`. A PCB review will follow once both boards are placed and routed.

**Confidence tags:**
- [verified]: checked against a datasheet or the files.
- [schematic]: read from the netlist.
- [inference]: engineering judgement.
- [verify]: a cheap check for the owner to make.

## Verdict

Both schematics are complete and the MCU footprints are repaired:
- The MCU board has its female sockets (U5/U9, C2938387) and the ICM-42688-P footprint back.
- PWM_1 is on PB4.
- Both PCBs match their schematics.

What's left is layout and BOM work:
- The MCU sockets and mounting holes aren't yet placed to mate with the PDB.
- C13 is in the PDB schematic but not on its PCB.
- 26 BOM parts have no LCSC number.

Placement is still in progress. On the MCU board, 16 footprints are off the board (U3, U5–U9, X1, several capacitors and net-ties). On the PDB, the VSERVO divider (C12, R13, R14, TP11) is unplaced.

## Decisions recorded (22 Oct)

- **Servo-branch reverse polarity:** accepted risk. The XT30 is keyed, and the harness gets a polarity check.
- **Hold-up capacitor:** left off; it doesn't fit on the PDB. A bodge pad will allow a 470 µF can (≈6 ms) later if PVD logging shows brown-outs.
- **AP2115 pad 1:** stays NC. If an R5A part turns up, bodge pad 1 to VIN.
- **Buzzer:** to be chosen when it arrives.
- **BtB headers:**
  - MCU side: female sockets (U5/U9).
  - PDB side: male headers (H9/H10) on B.Cu, DNP, hand-soldered.
  - The MCU sockets are numbered in reverse, so PDB pin n mates with MCU pin 17 − n.

## Open findings

| ID | Severity | Where | Finding | Fix | Basis |
|---|---|---|---|---|---|
| S1 | Medium | MCU U5/U9, H1–H4 | The sockets are off the board, and the 3 mm holes sit in a column at x = 87.55. Neither matches the PDB. The outlines are identical, with the MCU offset (+2.828, +5.753) mm from the PDB. Because of the reverse numbering, only one rotation works. | Place U5 at (84.648, 69.096) and U9 at (124.648, 69.096), both at 90°; U5 pad 16 (PWM_1) then sits at (84.648, 59.571). Place H1–H4 at (89.648, 54.096), (119.648, 54.096), (89.648, 84.096) and (119.648, 84.096). A 180° error doesn't short power (the power pins are symmetric) but scrambles every signal. U5/U9 aren't DNP, so JLC will quote them as THT assembly. | [verified] files |
| S2 | Medium | PDB C13 | PA35V100M5X9 (100 µF, 5 × 9 mm) is in the schematic and BOM but not on the PCB. There's no VDD_Buck/GND pin pair for a later bodge: J4.2 is the switched buzzer return, and TP9/TP10 are VBAT_Raw, ahead of F1/F2. | Delete C13, or set it DNP and "exclude from board". Add a THT test pad on VDD_Buck beside TP2 or TP3 (GND). | [verified] files |
| S3 | Medium | BOM | 26 parts have no LCSC number. MCU: J7, LED1/2/5/6, Q1, U1, U2, U3, U4, U7, U8. PDB: BT1, D1–D3, J1, J2, J4, LED1–4, Q1, U3, U4. A value-only match is where an AP2115R5A substitution would come from. | Enter the exact-MPN C-numbers before ordering, including AP2115R5-3.3TRG1 on both U3s. | [verified] files |
| S4 | Low | PDB J4 buzzer | The type is undecided; no PCB change is needed to choose it later. J4 sees VDD_Buck (≈5.6–8.1 V), switched by Q1 (2N7002, 60 V, 0.34 A). | Pick a part rated ≥ 8.5 V, or a piezo; avoid low-resistance magnetic transducers. R1 can stay (67 mW of its 125 mW rating when on). The firmware drive follows the choice (config review C1). | [verified] MCC, files |
| V1 | Verify | CN4 LoRa | The E22 tables disagree on minimum supply (2.3 V vs 3.3 V); +3V3 is ≈3.2 V (≈3.12 V worst case). | Bench-test transmit at the configured power. | [verify] |
| V2 | Verify | Stack height | Female body 4.3 mm; male header 7.2 mm overall (from the 3D models). Board gap ≈ 4.3 mm + male spacer, roughly 5.3–5.8 mm. USB-C ≈3.2 mm on the MCU top; PDB THT tails point into the same gap. | Check in 3D, trim the PDB tails flush, and check USB-C overmold clearance. | [inference] |

## Hold-up arithmetic

$$t_\text{hold}=\frac{C\,(V_1^2-V_2^2)}{2P}$$

where:
- \(C\) is the bulk capacitance [F].
- \(V_1\) and \(V_2\) bound the usable window [V].
- \(P\) is the power drawn through the window [W].

| Rail | Window | \(P\) | Capacitance | Hold-up |
|---|---|---|---|---|
| VDD_Buck | 7.0 → 3.6 V | ≈1.4 W | C3 10 µF | ≈0.13 ms (less with DC bias) |
| VDD_Buck | 7.0 → 3.6 V | ≈1.4 W | 470 µF bodge | ≈6 ms |
| +3V3 | 3.23 → 2.7 V | ≈1.0 W | 100 µF | ≈0.16 ms, and breaks the MAX40200 100 µF C_OUT limit |

There are two reasons the capacitor can't go on the MCU board:
- The MCU board has no VDD_Buck rail.
- The 9 mm can wouldn't fit in the stack gap.

## Board-to-board map

| PDB pin | MCU pin | H9 ↔ U5 | H743 | H10 ↔ U9 | H743 |
|---|---|---|---|---|---|
| 1 | 16 | PWM1 | PB4 (TIM3_CH1) | NC | — |
| 2 | 15 | PWM2 | PA7 (TIM3_CH2) | NC | — |
| 3 | 14 | PWM3 | PB0 (TIM3_CH3) | NC | — |
| 4 | 13 | PWM4 | PB1 (TIM3_CH4) | GND | — |
| 5 | 12 | PWM5 | PE9 (TIM1_CH1) | RX1 | PD9 (USART3_RX) |
| 6 | 11 | PWM6 | PE11 (TIM1_CH2) | TX1 | PD8 (USART3_TX) |
| 7 | 10 | GND | — | GND | — |
| 8 | 9 | +3V3 | — | +3V3 | — |
| 9 | 8 | +3V3 | — | +3V3 | — |
| 10 | 7 | GND | — | GND | — |
| 11 | 6 | SWO | PB3 | PPS | PA0 (TIM2_CH1) |
| 12 | 5 | SWCLK | PA14 | Buzzer | PD14 (TIM4_CH3) |
| 13 | 4 | SWDIO | PA13 | GND | — |
| 14 | 3 | NRST | NRST | VBAT_Sense | PA6 (INP3) |
| 15 | 2 | GPIO1 | PD0 | GND | — |
| 16 | 1 | GPIO2 | PD1 | VSERVO_Sense | PC4 (INP4) |

## Rails (unchanged)

- **VBAT:** 6.0–8.4 V.
- **VBAT_Servo:** up to 5.2 A with four servos stalled; sensed on PC4.
- **VDD_Buck:** 5.6–8.0 V; D1 clamps at 17 V.
- **+4V0:** 3.99 V, from

  $$V_\text{out}=V_\text{FB}\left(1+\frac{R_2}{R_7}\right)=0.7\left(1+\frac{47}{10}\right)=3.99\ \text{V}$$

  where:
  - \(V_\text{FB} = 0.7\) V is the TPS62135 reference.
  - \(R_2 = 47\) kΩ and \(R_7 = 10\) kΩ form the feedback divider.
- **+3V3:** ≈3.23 V, set by the OR-ed MAX40200s.
- **Sense dividers:** ÷2.8, so 8.4 V reads as 3.00 V; with the 100 nF filter capacitor, τ = 6.4 ms.

All ICs are within their datasheet limits.

## For the PCB review

- S1 placement, then the V2 stack check.
- Trim the THT tails on the PDB.
- Check USB-C plug clearance.
- Mechanical retention: stake the fuses, retain the servo plugs, use latching GPS and buzzer connectors, and stake any bodged capacitor.
- Run the J1 ground return straight to the XT30.
- Place the VDD_Buck bodge pad beside a GND pad.

## Priority

1. S1: place the parts.
2. V2: check the stack in 3D.
3. S2: remove C13 and add the bodge pad.
4. S3: fill in the LCSC numbers.
5. S4: choose the buzzer on arrival.
6. V1: bench-test the LoRa module.

## Sources

- MCC, *2N7002*: https://www.mccsemi.com/products/mosfets/small-signal-mosfets/2N7002
- LCSC, KH-1.27PH180 (C2938407) and KH-1.27FH (C2938387): https://www.lcsc.com/product-detail/C2938407.html
- Datasheet set as in the HTML review (TPS62135, MAX40200, AP2115, MS5611, ASDM1, ICM-42688-P, DS12110).
