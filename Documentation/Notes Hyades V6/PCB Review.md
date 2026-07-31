# Flight‑computer PCB review — MCU board + Power Distribution Board (PDB)

KiCad 10.0 projects, format `20260306` (sch) / `20260206` (pcb). Both boards parsed in full.

---

## 0. Method, scope, and confidence

**What I did.** I parsed every schematic and PCB file directly (KiCad 10 stores each net's _name_ on every pad/track/zone; there is no integer net table). I reconstructed the complete pad→net→pin‑function map for every IC from the PCB net assignments cross‑referenced with each symbol's pin table, then checked power, supervision, clock, bus, and protection topology. Load‑bearing device facts were verified against current datasheets (sources cited inline), not from memory.

**What I did _not_ do.** I did not run a geometric DRC. Trace‑to‑trace clearances, controlled‑impedance geometry, thermal‑relief/via‑stitch quality, copper‑pour stitching, and exact current‑carrying capacity of individual polygons were **not** verified — those need the board open in KiCad or a fab DRC. PCB‑layer comments below (§3) are from stackup, track‑width, via‑size, zone, and outline data only.

**Confidence tags:** `[verified]` = confirmed against a datasheet/primary source; `[fact]` = read directly from your files; `[inference]` = engineering judgement from the netlist; `[verify]` = you should confirm against a spec I couldn't fully pin down.

---

## 1. Board inventory `[fact]`

||MCU board|PDB|
|---|---|---|
|Size / outline|50 × 50 mm, rounded‑rect|50 × 50 mm|
|Stackup|4‑layer: F.Cu / In1.Cu=**GND plane** / In2.Cu=**power** (+3V3/+5V split) / B.Cu|same topology|
|Thickness|1.60 mm|1.66 mm|
|Key ICs|U1 STM32H743VIT6, U3 W25Q128JV (SPI flash), U4 ICM‑42688‑P (IMU), U5 MS5611 (baro), U2 USBLC6‑2P6|U1 TPS62135 (buck), U5 AP2112K‑3.3 (LDO), U3 LM358 (sense), U2 IHLP2525 1 µH, D2 SS34, D1 SMAJ15A|
|Connectors|J7 USB‑C, H8/H9 (2× 16‑pin 1.27 mm board‑to‑board), CN1/CN2 JST‑SH|CN1 XT30 (batt), U6/U7 (mate H8/H9), H9 servo header, JP3 rail‑select|

The MCU board carries **no regulation** — it receives **+3V3 and +5V from the PDB** over H8/H9. All logic (MCU, IMU, baro, flash) runs on the PDB's AP2112K 3.3 V LDO.

---

## PART 1 — SCHEMATIC REVIEW

## 1.1 MCU board — STM32H743 core

### Power / supervision — correct `[verified]`

- `VDD` ×5 (11,27,50,75,100) → +3V3; all `VSS` → GND; `VBAT` (6) → +3V3 (fine with no coin cell). `[fact]`
- `VCAP1`/`VCAP2` (48,73) each to a 2.2 µF (C13/C12) — matches the H743 requirement of 2×2.2 µF. `[verified]`
- `NRST` (14): 10 kΩ pull‑up (R12) + 100 nF (C11) + button (SW1). Standard. `[fact]`
- `BOOT0` (94): 5.1 kΩ pull‑down (R13) + button to +3V3 (SW2). Standard. `[fact]`
- Decoupling: ~11× 100 nF + bulk across the VDD pins and VDDA — well provisioned. `[fact]`

### Clock — correct topology `[fact]`

X1 (ASDM1‑24 MHz **active oscillator**) `Output`→PH0 (OSC_IN); **PH1 (OSC_OUT) left floating** — correct for an external‑clock/HSE‑bypass source. Ensure firmware selects **HSE bypass** mode.

- **L4 [verify]:** X1 pin 1 (`Standby`/enable) is left floating. Most Abracon MEMS oscillators have an internal pull‑up so floating = enabled, but confirm on the ASDM1 datasheet; if not, tie it to +3V3.

### Sensors

**IMU ICM‑42688‑P (U4)** — SPI wiring correct: SCLK→PC10, SDI(MOSI)→PB2, SDO(MISO)→PC11, CS→PB12, INT1→PC7, INT2→PC6, VDD/VDDIO→+3V3. `[fact]`

> **H3 — reserved‑pin error `[verified]`.** Datasheet (InvenSense DS‑000347): **pin 10 (RESV) "should NOT be connected to GND or Logic Low"** — tie to VDDIO or leave floating (internal pull‑up default; VDDIO preferred). **Pin 11 (RESV/NC) "is by default an output pin, should be float (no connection)."** On your board **both pin 10 and pin 11 are tied to GND**. Pin 10 to GND fights the internal pull‑up and is explicitly prohibited; pin 11 is a driven output shorted to GND (contention). Pins 2, 3 → GND and pin 7 → GND are all acceptable/required, so those are fine. **Fix:** cut the GND connection to pins 10 and 11; route pin 10 to VDDIO (+3V3) (or leave open), leave pin 11 open.

**Baro MS5611 (U5)** — correct `[verified]`: `PS`→+3V3 (I²C mode), `CSB`→+3V3, `SDA`→SDA1, `SCL`→SCL1, `SDO` floating (fine in I²C), 4.7 kΩ pull‑ups (R8/R9) present.

### Flash W25Q128JV (U3) — correct `[fact]`

Single‑SPI: CS→PC13, DO→MISO, DI→MOSI, CLK→SCK. `IO2` (/WP) and `IO3` (/HOLD) each pulled to +3V3 via 10 kΩ (R4/R5) — correct so the part isn't held/write‑protected in single‑SPI. (IO2/IO3 don't go to the MCU, so no quad mode — fine for logging.)

### USB

- Data path correct `[fact]`: J7 D±(A6/B6, A7/B7) → USBLC6 → PA12/PA11. USB‑C CC1/CC2 each have a 5.1 kΩ Rd to GND (device/UFP). Good.
- **M2 — VBUS↔buck 5 V contention `[fact/inference]`.** The `+5V` net contains **both** the USB‑C VBUS pins (J7 A4/A9/B4/B9) **and** the PDB feed (H8.3/4), with only a TVS (D1) between them. So the USB host's VBUS is hard‑tied to the PDB buck output (§2.2, ≈5.04 V). With battery **and** USB connected, the two 5 V sources are paralleled with no isolation → the buck can back‑feed the host port. On the MCU board `+5V` powers _only_ the USBLC6 VBUS pin and the VBUS divider — nothing else. **Fix:** give the USB side its own VBUS domain (ideal‑diode/load‑switch OR‑ing, or simply don't route PDB +5 V to the MCU board and don't tie USB VBUS to it), or guarantee only one source is ever present.
- **L1 — VBUS sense + LED `[fact/inference]`.** `USB_VBUS`→PA9 comes from an R1/R2 = 100 k/10 k divider: $V=5\text{ V}\cdot\frac{10\text{k}}{110\text{k}}\approx 0.45\text{ V}$ — a poor "VBUS present" level (and identical logic‑low whether or not USB is present, if read as GPIO). LED1 is tied to this 0.45 V node, so it can't turn on and it loads the divider. Minor; workable in firmware. If you want a USB‑power LED, drive it from +5 V through a series resistor to GND.

### **H1 — analog sense inputs on non‑ADC pins `[verified]`**

`ISHUNT_DELTA` → **PB5 (pin 91)** and `VBAT_SENSE` → **PB6 (pin 92)**, arriving from the PDB via H8.6/H8.7.

On the STM32H743, the ADC inputs are on ports A/C and PB0/PB1 only; **PB5 and PB6 have no ADC channel** (confirmed via ST's own community answer referencing DS12110 Table 2, and the ADC channel list PA0/PA1/PC0‑3 direct+fast, PA6/PC4/PB1 fast). PB5/PB6 are timer/GPIO‑capable but cannot be sampled by the internal ADC. **As drawn, battery voltage and current cannot be measured.**

**Fix (easy):** the sense lines are just connector pins — reassign H8.6/H8.7 on the MCU side to two free ADC‑capable pins. You have plenty unused: PA0, PA1, PA2, PA3, PA4, PA6, PC0–PC5. e.g. `VBAT_SENSE`→PA0, `ISHUNT_DELTA`→PA1. (Then also address L2 below.)

---

## 1.2 PDB — power tree

**Path `[fact]`:** XT30 → `+BATT` → **SS34 series Schottky** (reverse‑polarity protection, ~0.55 V drop @3 A) → `VDD` → **TPS62135 buck** → `+5V` → **AP2112K LDO** → `+3V3`. Servo rail (`JP3‑C`) is selectable via JP3 between `+BATT` and `+5V`, with 1000 µF + 100 µF bulk — a nice feature.

### Buck output — correct `[verified]`

$$V_\text{out}=V_\text{FB}\left(1+\frac{R_{15}}{R_{16}}\right)=0.7\text{ V}\left(1+\frac{62\text{k}}{10\text{k}}\right)=5.04\text{ V}$$ where $V_\text{FB}=0.7\text{ V}$ is the TPS62135 feedback reference (TI datasheet / E2E). Input range 3–17 V covers 2–4S. Correct 5 V rail. `[verified]`

- **L3 [verify]:** output cap is C6 10 µF + C3 1 µF ≈ 11 µF. DCS‑Control parts typically want ~22 µF effective; confirm against the datasheet's recommended $C_\text{out}$ for transient/stability margin (VIN cap C18 = 10 µF and SS/TR = 2.2 nF are fine).

### **H2 — current sense is non‑functional (three independent faults) `[fact + verified]`**

Intended: 10 mΩ shunt (R3) + LM358‑A (U3) → `ISHUNT_DELTA`. As drawn:

1. **Shunt not in series with the load `[fact]`.** `R3.1`→`VDD` and the buck `VIN` (U1.1) is on the **same** `VDD` node. `R3.2`→`ISHUNT-` connects _only_ to the op‑amp inverting input (high‑Z) and a test point. So the load current flows BATT→D2→VDD→buck and **bypasses the shunt** — R3 carries only nA and drops ~0 V. It measures nothing regardless of the amplifier.
2. **Common‑mode / abs‑max violation `[verified]`.** LM358‑A inputs sit at `VDD` (battery potential, ~7–16 V) while the op‑amp is powered from +3V3. LM358 common‑mode range is $0$ to $V^+-1.5\text{ V}\approx1.8\text{ V}$; the onsemi LM358 datasheet also states that for $V_S<32$ V the **absolute‑max input equals the supply** (3.3 V here). Battery‑potential inputs are far outside both → invalid output and possible device damage.
3. **Open loop `[fact]`.** `1OUT`→`ISHUNT_DELTA` has no feedback network back to `1IN−`; the stage runs open‑loop (comparator), no defined gain.

**Fix:** put the shunt **in series** between D2's cathode and the buck VIN (split `VDD` into "pre‑shunt" and "post‑shunt" nodes), and use a **dedicated high‑side current‑sense amplifier rated for the battery common‑mode voltage** (INA180/INA181/INA226‑class), not a 3.3 V LM358. Then route its output to a real ADC pin (H1).

### M1 — voltage sense clips at high battery `[verified]`

LM358‑B buffers the battery divider: $V_\text{REF}=V_\text{DD}\frac{R_2}{R_4+R_2}=V_\text{DD}\frac{33\text{k}}{213\text{k}}=0.155,V_\text{DD}$. The buffer's input/output can only reach $V^+-1.5\text{ V}\approx1.8\text{ V}$ on a 3.3 V supply, so it **clips once $0.155,V_\text{DD}>1.8\text{ V}$, i.e. $V_\text{DD}\gtrsim11.6$ V (~12.2 V battery)** — the top of a 3S/4S range is unreadable. Fine for 2S. (And it lands on PB6 → H1.) **Fix:** power the sense op‑amp from a rail‑to‑rail part and/or lower the divider ratio so the full range stays < ~1.6 V, or drop the buffer and feed a scaled divider straight to an ADC pin with VDDA filtering.

### M4 — input TVS vs cell count `[verify]`

D1 = SMAJ15A: 15 V standoff, ~16.7 V breakdown (min). A **4S** LiPo reaches 16.8 V → the TVS conducts near full charge and overheats. Fine for **≤3S**. Confirm your battery; for 4S use ~SMAJ18A/20A (with margin above pack max and below the buck's 17 V VIN limit — note 4S at 16.8 V is already close to the TPS62135 17 V ceiling).

### M3 — LDO thermals `[inference]`

$P_\text{diss}=(V_\text{in}-V_\text{out})I=(5.04-3.3)I\approx1.74,I$. At 0.25 A → 0.43 W, at 0.35 A → 0.61 W in a SOT‑23‑5 ($\theta_{JA}$ ~150–250 °C/W board‑dependent) → ~65–120 °C rise. An H743 at high clock plus peripherals can sit in that current band. **Verify your 3.3 V budget;** if it's >~0.25 A steady, consider a small **buck** to 3.3 V (you already have the 5 V rail) or a larger‑pad LDO. `[verify]`

### Correct on the PDB `[fact]`

Reverse protection (SS34 series), VIN decoupling, soft‑start cap, EN straps, MODE/VSEL straps, JP3 servo‑rail select with bulk, battery‑present LED (LED1 via 470 Ω). All fine.

 

## PART 2 — PCB / LAYOUT OBSERVATIONS

From stackup/width/via/zone/outline data only (no geometric DRC — see §0).

- **Stackup is appropriate `[fact/inference]`:** In1 solid GND plane, In2 power, outer layers signal + GND pour. Good return paths for the SPI/USB/PWM signals.
- **Track widths `[fact]`:** MCU signals 0.15–0.20 mm (fine at these currents). PDB power uses 1.0 mm (×7) and one 3.0 mm segment; signals 0.25 mm.
    - **[verify] Power/servo current.** By IPC‑2221, ~1.0 mm of 1 oz outer copper ≈ 2 A at 10 °C rise; the single 3.0 mm run (main battery path) more. Check the **sum of worst‑case servo stall currents** on the `JP3‑C` rail and the buck output path against trace width **and via count** (0.6/0.3 mm vias ≈ ~1 A each — parallel enough of them on high‑current nets). This is the one PCB item most worth a hard check.
- **Vias `[fact]`:** MCU mostly 0.5/0.3 mm (0.1 mm annular ring — OK for JLC); PDB 0.6/0.3 mm.
- **[verify] Buck thermal pad:** ensure a full array of GND vias under the TPS62135 QFN pad to the plane (couldn't confirm count from the parse).
- **[verify] Placement:** confirm each MCU VDD 100 nF sits at its pin with a short GND via; keep the USB D± pair matched (USB 2.0 FS is tolerant, so this is minor); keep the X1→PH0 trace short.
- **Mechanical [verify]:** both boards are 50 × 50 mm (~71 mm diagonal). If the airframe internal diameter is ~62 mm (per earlier notes), a 50 mm **square** board won't fit a 62 mm tube (needs ≤ ~43 mm side, or a circular board). Confirm the airframe section these mount in.

---

## Appendix — reference maps (from your files)

**STM32H743 net map (used pins):** PE2 SCK1·PE5 MISO1·PE6 MOSI1·PC13 CS1(flash) | PC10 SCK2·PB2 MOSI2·PC11 MISO2·PB12 CS2·PC7 INT1·PC6 INT2(IMU) | PA8 SCL1·PC9 SDA1(baro) | PA11 USB_D-·PA12 USB_D+·PA9 USB_VBUS | PA13 SWDIO·PA14 SWCLK·PB3 SWO | PA5/PA7/PB0/PB1/PE9/PE11 PWM1‑6 | PB5 ISHUNT_DELTA·PB6 VBAT_SENSE (**H1**) | PD8/PD9 TX1/RX1·PB14/PB15 TX2/RX2·PE7 PPS·PD0‑3 GPIO1‑4.

**H8 (power+PWM to PDB):** 1,2=+3V3 · 3,4=+5V · 5,8,15,16=GND · 6=ISHUNT_DELTA · 7=VBAT_SENSE · 9‑14=PWM1‑6. **H9 (debug+comms):** 1,2,7‑10=GND · 3=PPS · 4=SWCLK · 5=SWDIO · 6=SWO · 11=RX1 · 12=TX1 · 13,14=GPIO1,2 · 15,16=+3V3.

**PDB power tree:** XT30→SS34→VDD→TPS62135→+5V(5.04 V)→AP2112K→+3V3. Feedback R15 62k/R16 10k. Batt divider R4 180k/R2 33k. Shunt R3 10 mΩ (**H2**).