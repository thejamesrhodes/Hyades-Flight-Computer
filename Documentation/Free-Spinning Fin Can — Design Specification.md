
**Doc:** HYA-FC-001 · **Rev:** A · **Date:** 2026-07-05 · **Status:** For CAD modelling **System:** Hyades — roll-decoupled rear fin can, 29 mm motor, reusable (printed parts expendable)

---

## 1. Scope and design intent

A rear fin can rotates freely about the roll axis on two ball bearings, decoupling rear-fin roll torque from the airframe (prevents roll reversal against the forward canards). The bearings ride on an aluminium journal tube that is **also** the motor mount and thrust column: motor thrust passes journal → printed inner mount → M3 bolted friction joint → CF airframe, **bypassing the bearings and fin can entirely**.

**Design rules (binding):**

- R1 — No thrust-path element may bear on the rotating fin can. Motor retention reacts into the journal/mount only.
- R2 — Fins uncanted (0°). A seized bearing must revert the vehicle to a conventional fixed-fin configuration (verified benign).
- R3 — Printed parts (fin can, inner mount) are consumables; journal cartridge (journal + bearings + spacer + retainers) and fasteners are reused across prints.
- R4 — Mass is minimised wherever it does not erode a stated margin.

**Conventions:** SI units, mm in CAD. Station axis `STA` along the roll axis, **origin at the fin root leading-edge plane, positive aft**. "Radial" ⟂ roll axis. All loads below are **ultimate** (α = 10° gust at V = 300 m/s) unless noted.

---

## 2. Master parameter table (SolidWorks global variables)

Copy into Equations → Global Variables. Names are CAD-safe. `[A]` = assumption, revisit; `[F]` = fixed by prior analysis/fact; `[D]` = derived, update if a parent changes.

|Variable|Value|Unit|Basis|
|---|---|---|---|
|`D_AIRFRAME_OD`|61.0|mm|[F] airframe|
|`D_AIRFRAME_ID`|59.0|mm|[F] 1 mm CF wall|
|`D_MOTOR`|29.0|mm|[F] motor calibre|
|`D_JOURNAL_ID`|29.3|mm|[D] motor +0.3 clearance|
|`D_JOURNAL_OD`|35.0|mm|[F] = bearing bore|
|`BRG_BORE`|35.0|mm|[F] 6807|
|`BRG_OD`|47.0|mm|[F] 6807|
|`BRG_W`|7.0|mm|[F] 6807|
|`STA_BRG_FWD`|−5.0|mm|[A] see §5 (straddle CP)|
|`STA_BRG_AFT`|+75.0|mm|[A] see §5|
|`L_BRG_SPACING`|80.0|mm|[D]|
|`STA_FIN_CP`|+39.0|mm|[D] Barrowman, subsonic|
|`C_ROOT`|100.0|mm|[F]|
|`C_TIP`|50.0|mm|[F]|
|`FIN_SPAN`|60.0|mm|[F] exposed semispan|
|`FIN_SWEEP_LE`|43.0|mm|[F]|
|`T_FIN`|4.0|mm|[F] (owner may vary; see §7.3)|
|`R_FILLET`|10.0|mm|[F] curvature-continuous|
|`N_FINS`|4|—|[F]|
|`T_SHELL_BASE`|2.0|mm|[D] §7.1|
|`T_SHELL_PAD`|3.0|mm|[D] §7.1 (or ribs, §7.2)|
|`L_CAN`|130.0|mm|[A] STA −15 to +115|
|`GAP_CAN_AIRFRAME`|0.8|mm|[A] axial running gap, §8|
|`N_BOLTS`|16|—|[F] 8 × 2 planes|
|`D_BOLT`|3.0|mm|[F] M3 × 0.5, class 8.8|
|`T_BOLT_TORQUE`|0.6|N·m|[A] §9, bench-verify|

Load constants (analysis, not geometry): `F_THRUST_ULT = 465 N` (AeroTech I200W cert peak — bounding verified 29 mm I-class); `Q_MAX = 54.0 kPa` (V = 300 m/s SL `[A]`); `CNA_FINS = 9.0 rad⁻¹` (Barrowman incl. body interference); `N_ULT = 248 N` (α = 10°); `A_DRAG = 79 N` (C_D = 0.5 `[A]`).

---

## 3. Assembly architecture and part list

```
AIRFRAME (CF tube, stationary)
 └─ INNER MOUNT (FDM, consumable) ── 16 × M3 friction joint ──> CF tube
     └─ JOURNAL TUBE (Al, reused)  ← thrust column; motor inside
         ├─ BRG_FWD 6807 (locating: inner clamped, outer clamped)
         ├─ INNER SPACER (Al tube, sets L_BRG_SPACING)
         ├─ BRG_AFT 6807 (floating: inner g6 slide + wave washer, outer clamped)
         └─ retainers / clamp ring
     FIN CAN (FDM, consumable, rotating)
         ├─ 2 × bonded Al SEAT SLEEVES (H-tolerance bores the print can't hold)
         ├─ 2 × bolted CAP RINGS (outer-race clamp; removable for reuse)
         └─ 4 × fins (integral)
```

|#|Part|Material / spec|Qty|Reuse class|
|---|---|---|---|---|
|P1|Fin can (shell + fins + seat pockets)|FDM PC or PA-CF (min ASA)|1|consumable|
|P2|Inner mount / thrust bulkhead|FDM PC or PA-CF|1|consumable|
|P3|Journal tube Ø35 × Ø29.3|Al 6061/6082|1|reused|
|P4|Bearing 6807-2RS, 440C stainless|35 × 47 × 7, ISO 15 dims|2|reused|
|P5|Inner spacer tube|Al, Ø35 slip ID|1|reused|
|P6|Seat sleeve, Ø47 K7 bore|Al, machined|2|consumable (bonded)|
|P7|Cap ring (outer-race clamp)|FDM or Al, 3 × M2.5/M3|2|reused|
|P8|Wave washer, Ø35 shaft series|steel|1|reused|
|P9|Bolts M3 × L, class 8.8 + captive nuts + conical spring washers + Ø9 flat washers|ISO 898-1|16 sets|reused|

---

## 4. Interfaces, fits, and tolerances

Fit logic `[inference from standard bearing-fit practice — ring rotating relative to load direction takes the tight fit]`: the radial aero load is fixed in the airframe frame; the **outer rings rotate with the can → circumferential outer-ring load → outer rings must be tight/clamped** (else seat creep). Inner rings see a stationary (point) load → may be loose; **axial float lives at the aft inner ring**. _(Supersedes Rev-0 discussion which floated the outer race.)_

|Interface|Fit / tol|Notes|
|---|---|---|
|Outer race Ø47 in seat sleeve|sleeve bore **K7**, plus mechanical clamp (cap ring); optional retaining compound (Loctite 603-class) as anti-creep|tight side of transition; clamp is the primary retention `[verify against chosen bearing brand's fit table]`|
|Seat sleeve OD in printed pocket|pocket +0.2…+0.4 mm on dia, epoxy-bonded|bond fills FDM tolerance (±0.1–0.3 mm typical)|
|Fwd inner race on journal|Ø35 **js6** zone, clamped shoulder + retainer|locating bearing; takes A_DRAG = 79 N|
|Aft inner race on journal|Ø35 **g6** zone, axial slide + wave washer (P8)|floating; absorbs thermal growth, print tolerance|
|Journal ID / motor|Ø29.3 +0.2/−0|motor retention per R1, into journal only|
|Mount OD / CF tube ID|Ø59 −0.05…−0.15 (slight clearance to light touch)|do **not** design a heavy press fit: FDM CTE ≫ CF hoop CTE, heating grows interference and hoop-stresses the 1 mm wall; keep design interference ≤ 0.1 mm radial|
|Can OD / airframe OD|flush at Ø61, axial gap `GAP_CAN_AIRFRAME`|add labyrinth lip, §8|

Datum scheme (use in every part and the assembly): **A** = journal/roll axis · **B** = fwd bearing seat face (STA reference) · **C** = fin-1 index plane through axis.

---

## 5. Bearing stations and load rationale

Fin CP sits at `STA_FIN_CP` ≈ +39 mm (MAC = 77.8 mm, spanwise CP ȳ = 26.7 mm, subsonic; shifts aft transonic `[inference]`). Bearings are placed to **straddle** the CP:

$$R_\text{fwd} = N_\text{ult}\frac{STA_\text{aft}-STA_\text{CP}}{L_b} = 248\times\frac{36}{80} = 112\ \mathrm{N},\qquad R_\text{aft} = 248\times\frac{44}{80} = 136\ \mathrm{N}$$

vs. 620 N for the cantilevered layout — a 4.5× reduction on the worst bearing. Against 6807-class static rating $C_0 \approx 3\ \mathrm{kN}$ `[verify brand table]`: static safety ≈ 22. Bearing stations are `[A]`-tagged: move them if packaging demands, but keep the CP between them; re-run the two-line formula above if you do.

Axial: drag 79 N → fwd (locating) bearing. Gyroscopic couple negligible (<1 N with 0° cant).

---

## 6. Load and margin summary (carried from analysis, Rev A values)

|Path|Ultimate load|Capacity|Margin|Tag|
|---|---|---|---|---|
|Friction joint (service thrust path)|465 N|≈ 2.0 kN (μ = 0.25, 500 N/bolt retained)|~4×|`[verify μ + relaxation, §9]`|
|Bolt shear (backstop)|58 N/bolt|≈ 2.4 kN (8.8, A_s = 5.03 mm²)|~42×|ISO 898-1|
|CF hole bearing (backstop)|19 MPa|~150 MPa conservative|~8×|`[verify tube coupon]`|
|Bearing radial (aft, straddled)|136 N|C₀ ≈ 3 kN|~22×|`[verify]`|
|Bearing axial|79 N|≫|large|—|
|Fin root bending|12.4 MPa|≥ 60 MPa in-plane (printed PC)|≥ 5×|print orientation §10|
|Fin flutter|300 m/s|V_f ≥ 530 m/s (G = 0.3 GPa)|≥ 1.8×|Martin boundary, rough|
|Shell wall (fin root, local)|see §7|—|≥ 2× at t per §7|—|

---

## 7. Shell wall sizing (mass-minimised)

### 7.1 Governing case and thickness

Global beam bending of the shell between bearings is trivial (σ ≈ 4 MPa at t = 1 mm). The governing case is **local circumferential wall bending where each fin root moment enters the shell**.

Per-fin ultimate root moment $M_f = 3.31\ \mathrm{N,m}$ distributed over the root chord gives a running moment

$$m' = \frac{M_f}{c_r} = \frac{3.31}{0.100} = 33.1\ \mathrm{N,m/m}$$

The wall reacts $m'$ in plate bending. Bounds:

$$\sigma = \frac{6m'}{t^2}\ \text{(all one side, conservative)}\qquad \sigma = \frac{3m'}{t^2}\ \text{(shared both sides, continuous shell)}$$

Design allowable for printed PC/PA-CF in the favourable (in-plane) direction: $\sigma_\text{allow} = 30\ \mathrm{MPa}$ (≈ 50 % knockdown on typical 60+ MPa datasheet flexural strength for FDM voids and warm structure `[assumption — coupon-verify with your filament]`). Then:

$$t_\text{req} = \sqrt{\frac{6m'}{\sigma_\text{allow}}} = 2.6\ \mathrm{mm}\ \text{(one-sided)}\qquad t_\text{req} = 1.8\ \mathrm{mm}\ \text{(shared)}$$

**Specification:**

- `T_SHELL_BASE` = **2.0 mm** everywhere (also the practical FDM printability/handling floor for a Ø61 tube; going below 2.0 saves < 10 g and costs robustness — not worth it).
- Local reinforcement under each fin root to the one-sided bound: either `T_SHELL_PAD` = **3.0 mm** over a strip (axial extent = root chord + fillets + 10 mm ≈ full can length here; circumferential extent = `T_FIN` + 2·`R_FILLET` + 2 × 10 mm ≈ 44 mm arc), **or** the rib option below.

### 7.2 Rib option (preferred for mass)

Instead of pads, run a **pair of internal axial ribs** under each fin root, one each side of the root plane at ≈ ± (`T_FIN`/2 + `R_FILLET`), rib section ≈ 2 mm thick × 5 mm deep, full can length, blended into the bearing-seat bosses. Ribs carry the root couple with bending depth rather than wall thickness — roughly half the added mass of pads for the same stress (≈ +12 g total vs ≈ +25 g for pads `[inference]`). Bonus: ribs tie the fin loads directly into the seat bosses, which is where the load leaves the shell anyway.

### 7.3 Fin thickness note

`T_FIN` = 4 mm is comfortable everywhere (root σ margin ≥ 5×, flutter ≥ 1.8× even at G = 0.3 GPa). If mass-shaving: root bending scales as $1/t^2$ and flutter as $t^{3/2}$; 3 mm keeps root margin ≈ 3× and V_f ≥ ~350 m/s at the weak-print modulus — **legal but thin on flutter margin at V = 300 m/s; keep ≥ 3.5 mm unless the flight sim shows V_max well under 300 m/s.**

---

## 8. Dust, gap, and running clearance

- Axial gap can↔airframe: `GAP_CAN_AIRFRAME` = 0.8 mm nominal (FDM tolerance stack + thermal).
- Add a **labyrinth lip**: 1.5 mm axial overlap ring on the can nesting inside a matching rebate on the airframe side, 0.4 mm radial clearance — shields the forward bearing from dust/exhaust with zero contact. Print-in feature, ~2 g.
- Seals: 2RS contact seals both bearings (reuse + dust). Residual friction ~5–10 mN·m total vs canard roll authority of order 10⁻¹–10⁰ N·m at high q → coupling ≤ a few % where roll reversal matters `[inference]`. If freer spin is wanted near apogee, fit non-contact (RZ) on the protected inboard faces only.

---

## 9. Bolted friction joint (thrust path) — assembly spec

- 16 × M3 × (grip + nut) class 8.8, **through-bolt into captive nuts** (printed hex pockets in mount or bonded tapped ring). No threads in plastic on the thrust path.
- Stack per bolt, outside→in: bolt head / **conical (Belleville) spring washer** / Ø9 flat washer / CF tube / mount / captive nut.
- Torque `T_BOLT_TORQUE` = 0.6 N·m `[A]` → preload ≈ 1000 N fresh, ≈ 500 N assumed retained after viscoelastic relaxation. **Bench-verify:** (i) CF/print friction coefficient (design credit currently μ = 0.25 `[unverified]`), (ii) preload retention at 48 h and at 60 °C.
- **Witness marks** across the CF/mount seam at 4 clock positions — slip indicator, inspect post-flight.
- **Re-torque before every flight** until relaxation data exists; then set an interval.
- Bolt planes ≥ 2–3·d (6–9 mm) from the CF tube's cut end (edge distance / shear-out).

---

## 10. Materials and print process

|Item|Spec|
|---|---|
|Filament|PC or PA-CF preferred; ASA/ABS minimum. **PLA and PETG prohibited** on P1/P2 (motor heat; PLA T_g ≈ 55–60 °C)|
|Orientation|**Axis vertical** (layers ⟂ roll axis). Puts fin span-bending stress and shell circumferential bending stress **in-plane**; the weak interlayer axis sees only the ~4 MPa global axial stress|
|Shell/fins|Shell 100 % solid (it _is_ a wall); fins ≥ 4 perimeters, 100 % solid at ≤ 4 mm thickness (infill saves < 15 g and costs balance symmetry)|
|Thermal|Thermal break (phenolic/glass liner or air gap) between motor case and journal; **measure journal/mount temperature on a static fire** before committing filament and grease (grease limit typ. 120–150 °C)|
|Balance|Static-balance the finished can on the journal; correct with small bore-side material removal. Criticality is low (0° cant) but cheap to do|

---

## 11. Mass budget (estimate, PC at 1200 kg/m³ `[inference]`)

|Item|Mass|Notes / levers|
|---|---|---|
|Shell 2.0 mm × Ø61 × 130 + ribs + seat bosses|~90 g|ribs not pads (−13 g)|
|4 × fins, solid, airfoil factor 0.66|~57 g|`T_FIN` 3.5 mm → −7 g (watch flutter)|
|2 × 6807-2RS 440C|~60 g|`[verify catalogue ~30 g ea]`|
|Journal Ø35 × Ø29.3 × ~145|~110 g|**biggest lever:** thin to 1.5 mm wall between bearing lands → ~−35 g `[verify buckling/thrust column trivially OK]`|
|Spacer, retainers, caps, sleeves|~30 g||
|16 × M3 sets|~25 g|12-bolt variant saves 6 g; keep 16 for CF hole-stress spread|
|**Total**|**≈ 370 g**|rotating portion ≈ 150 g|

Rotating polar inertia estimate $I_p \approx 2.6\times10^{-4}\ \mathrm{kg,m^2}$ (shell 6.5×10⁻⁵ + fins 1.9×10⁻⁴) — for the roll-dynamics model.

---

## 12. Verification checklist

1. Coupon: CF tube bearing/shear-out allowable (or accept 8× margin on the conservative 150 MPa).
2. Bench: μ (CF/print) and bolt preload retention (48 h, 60 °C).
3. Static fire: journal and mount temperatures → filament + grease confirmation.
4. Spin test + torque-feel each flight; replace bearings on any roughness (≈ £5 ea).
5. Post-flight: witness marks (slip), bolt torque, seat sleeve bond, fin roots.
6. Flight sim on the chosen I-load: confirm V_max ≤ 300 m/s or rescale §5–§7 loads by (V/300)².

## 13. Open items

- `STA_BRG_*` and `L_CAN` are packaging placeholders — set in CAD, keep CP straddled.
- μ and preload retention unverified (§9) — until then the joint is formally "friction service / bolt-bearing ultimate."
- 6807 C₀ and mass from the chosen brand's catalogue.
- Journal thin-wall option (§11) — confirm before machining.
- Transonic CP shift if the flight sim shows M > 0.8: recheck straddle condition.