# Nose Cone — Geometry and Structure (62 mm CV)

Captured 2026-08-23. Supersedes nothing; first structures doc in this project.

## Requirement

| | |
|---|---|
| Finished length | 100 mm |
| Base OD | 62 mm (R = 31 mm) |
| Tip | spherical, r_n = 15 mm (30 mm dome, nose camera) |
| Fineness (finished) | 1.61 |
| Material | ASA, FDM |

## Profile decision: **elliptical**, not LD-Haack

Ranked by peak suction from an incompressible axisymmetric source-panel solve
(validated against the exact sphere solution: Cp_min = −1.25, 0.4 % error at 800
panels; grid-converged to < 0.4 % on 2× refinement). Lower |Cp_min| ⇒ higher
critical Mach ⇒ later transonic drag rise.

| profile | L_theo (mm) | cut (mm) | Cp_min | M_cr (indic.) | A_wet (mm²) | vol (cm³) |
|---|---|---|---|---|---|---|
| **elliptical** | 101.95 | 1.95 | **−0.173** | 0.84 | 16 119 | **205** |
| LV-Haack | 113.76 | 13.76 | −0.183 | 0.84 | 15 479 | 191 |
| tangent ogive | 120.26 | 20.26 | −0.189 | 0.83 | 15 615 | 194 |
| LD-Haack (Von Kármán) | 121.90 | 21.90 | −0.254 | 0.80 | 14 995 | 179 |
| power n = 0.75 | 139.62 | 39.62 | −1.802 | 0.50 | 14 160 | 160 |

Findings:

1. **The aft shoulder governs, not the tip.** Cp_min occurs at x = 67–100 mm. At
   this fineness the 30 mm dome blends by x = 6.3 mm and the rest is a fairing.
2. **LD-Haack is wrong here.** Near the base, y = R − k(L−x)^{3/2}, so dy/dx → 0
   but d²y/dx² → −∞: the base meridional curvature is formally singular. At
   fineness 5–6 that is a local nuisance; at 1.61 it costs ≈ 0.04 in M_cr, spread
   over the whole aft nose (Cp −0.18 → −0.24 from x = 85 to 99). Its 7 % wetted-area
   advantage is ≈ 0.5 % of vehicle skin friction — nowhere near enough to pay for it.
3. **Nothing meaningfully better exists.** A two-parameter superellipse sweep
   ((y/R)^s + ((L−x)/L)^n = 1) optima at n ≈ 1.7, s = 2 with Cp_min = −0.170 — 2 %
   over the plain ellipse. The G2 option (superellipse s = 2, n = 3.12, natural tip
   radius exactly 15 mm, no blunting, no curvature jump) is *worse*, Cp_min = −0.253:
   a naturally-15 mm tip over 100 mm demands a fuller forebody, repaid in aft expansion.
   Accept the G1 joint.

Limits: incompressible potential flow, α = 0. Ranks drag-rise onset only — says
nothing about supersonic wave drag or base drag. Past M ≈ 1.2 check in RASAero II.

## Geometry (closed form, exact for the ellipse)

    L    = R·sqrt(1 + (L_f − r_n)² / (R² − r_n²))
    u_t  = L²·sqrt(R² − r_n²) / (R·sqrt(L² − R²))          u_t = L − x_t
    y_t  = (R/L)·sqrt(L² − u_t²)
    y'_t = R·u_t / (L·sqrt(L² − u_t²))
    L_f  = r_n + sqrt((R² − r_n²)(L² − R²)) / R

Tangency condition (general, any profile): y(x_t)·sqrt(1 + y'(x_t)²) = r_n;
sphere centre x_0 = x_t + y_t·y'_t; new tip x_apex = x_0 − r_n. In the finished
frame the cap centre always sits at x = r_n.

For R = 31, L_f = 100, r_n = 15:

| | |
|---|---|
| Theoretical L | 101.954547 mm |
| Removed at tip | 1.954547 mm |
| Tangency (finished frame) | (6.341176, 12.248460) mm, dy/dx = 0.706932 |
| Cap centre | (15.000000, 0), R 15, sweep 54.742° |
| Exposed dome diameter | 24.497 mm |

**Open item:** "30 mm dome" is ambiguous. The above assumes a 30 mm *sphere*, which
exposes only 24.50 mm of aperture. If a 30 mm *clear aperture* is required,
r_n = 17.0397 mm (34.08 mm sphere): L = 104.034083, cut 4.034083, tangency
(8.955729, 15.000027), slope 0.538930 — marginally better aerodynamically
(Cp_min −0.169, 208 cm³). **Decide before cutting metal.**

## SOLIDWORKS build

Native conic — no equation-driven curve, no spline. Sketch on Front, origin at the
physical tip, revolve about X:

1. Ellipse, centre (100, 0), major radius 101.954547 (X), minor radius 31 (Y). Trim
   to the forward-upper quadrant.
2. Centrepoint arc: centre (15, 0), R 15, from (0, 0) to (6.341176, 12.248460).
3. Close (100, 31) → (100, 0) → (0, 0). Revolve.

Parametric alternative (origin at the *theoretical* tip; shift aft 1.954547 mm):
`x_t = 101.954546831*(1-cos(t))`, `y_t = 31*sin(t)`, `t1 = 0.406189351`, `t2 = pi/2`.

## Wall thickness — ASA, FDM

**Baseline: 1.6 mm shell (4 × 0.4 mm perimeters), 3 mm shoulder, 5 solid layers,
no infill in the shell.**

No flight load sizes the wall. Margins at 1.6 mm against Prusament ASA
(42 MPa XY / 45 MPa Z printed, E = 1.6 GPa, ρ = 1.07 g/cm³, HDT 86–93 °C):

| case | demand | capacity | margin |
|---|---|---|---|
| stagnation hoop at q = 50 kPa | 0.97 MPa | 42 MPa | ≈ 43× |
| external-pressure buckling (Cp −0.17 at q = 50 kPa) | 8.5 kPa | 276 kPa | ≈ 32× |
| boost axial (≈ 60 g assembly at 20 g) | 12 N | 3.9 kN | ≈ 300× |
| recovery shock through a 3 mm shoulder | 0.5–2 kN | ≈ 8 kN derated | 4–15× |

Shell mass ≈ 27.6 g at 1.6 mm (16 119 mm² × t × ρ).

Do **not** trust 45 MPa in Z without an enclosure at 45–55 °C; Prusa's figure comes
from enclosed prints and layer adhesion is the parameter that collapses first.

## Crush / sacrificial behaviour — reality check

The forward 40 mm of shell is 8.7 g at 1.6 mm. At an FDM SEA of 8–18 J/g that is
**70–160 J**. A ballistic 62 mm vehicle carries 3–15 kJ (1–2.5 kg at 50–140 m/s).
The nose therefore absorbs **1–3 %** of the impact. It is a *fuse*, not a crumple
zone; do not size it as one and do not expect the camera to survive a chute failure.

Consequences for design:

- Keep the wall **thin**, not thick. A thicker shell raises peak force and drives
  more load into the coupler without meaningfully raising absorbed energy.
- Real absorption per unit volume comes from a **10–15 % gyroid core** in the dead
  volume ahead of the camera (44.7 cm³ fwd of x = 40 → 4.8–7.2 g → 40–130 J).
  Roughly doubles the total. Still a token, but it is free volume.
- Print **tip-up**, no supports. Layer planes normal to the impact axis: crush loads
  the bonds in compression and failure is splitting/delamination rather than a rigid
  spear.
- Do not over-strengthen the shoulder. It should fail before the airframe does.

**Thermal:** adiabatic wall temperature (r = 0.89, ISA SL) is 48 °C at M 0.8,
66 °C at M 1.0, 89 °C at M 1.2. ASA HDT is 86–93 °C at 0.45 MPa. Above M ≈ 1.2 the
stagnation region reaches HDT — brief exposure and thermal mass probably carry it,
but it is no longer a non-issue. Not analysed.

**Vent the nose** (a small hole through the shoulder) — pressure differential
otherwise loads the shell and, more importantly, corrupts the barometric channel
that apogee detection depends on (see FDS-01 §7 / the baro integration notes).

## Tooling

`nosecone.py` (geometry + SOLIDWORKS output + .sldcrv export) and `nose_aero.py`
(panel solver + ranking; `--validate` runs the sphere check) were produced with this
analysis and delivered in-session. Not stored here.

## Sources

- Stoney, *Transonic Drag Measurements of Eight Body-Nose Shapes*, NACA RM L53K17 (1954) — https://ntrs.nasa.gov/citations/19930087953
- *Nose cone design* (Haack series, elliptical) — https://en.wikipedia.org/wiki/Nose_cone_design
- Prusament ASA technical data sheet v2022-16 — https://prusament.com/wp-content/uploads/2022/10/ASA_Prusament_TDS_2022_16_EN.pdf
- Albak et al., *Crashworthiness of Thin-Walled Multi-Cell PLA 3D-Printed Tubes*, Designs 7(5):108 (2023), DOI 10.3390/designs7050108 — https://www.mdpi.com/2411-9660/7/5/108
