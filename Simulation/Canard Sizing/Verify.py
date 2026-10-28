"""Verification of canard_model against limits with known answers.
Run: python3 verify.py
"""
import numpy as np
import canard_model as cm

ok = True
def chk(name, got, want, tol, note=""):
    global ok
    p = abs(got-want)/max(abs(want), 1e-12)
    good = p <= tol
    ok &= good
    print(f"[{'PASS' if good else 'FAIL'}] {name:52s} {got:12.5g} vs {want:12.5g} "
          f"({100*p:5.2f}% , tol {100*tol:.0f}%)  {note}")

print("== C_Nalpha limits ==")
# 1. slender-wing limit: DATCOM formula -> pi*AR/2 as AR -> 0
for ar in (0.05, 0.1):
    chk(f"subsonic DATCOM -> pi*AR/2  (AR={ar})",
        cm.cn_alpha(0.2, ar)[0], np.pi*ar/2, 0.02)
# 2. high-AR incompressible -> 2*pi  (thin-airfoil limit)
chk("subsonic DATCOM -> 2*pi at AR=1000", cm.cn_alpha(0.05, 1000.0)[0], 2*np.pi, 0.01)
# 3. Prandtl-Glauert scaling at high AR
chk("PG scaling at AR=1000, M=0.6", cm.cn_alpha(0.6, 1000.0)[0],
    2*np.pi/np.sqrt(1-0.36), 0.02)
# 4. supersonic 2-D limit 4/beta at large beta*AR
b = np.sqrt(2.0**2-1)
chk("supersonic -> 4/beta at AR=500, M=2", cm.cn_alpha(2.0, 500.0)[0], 4/b, 0.01)
# 5. continuity of the supersonic blend at beta*AR = 1
ar = 1.111; Mstar = np.sqrt(1+1/ar**2)
lo = cm.cn_alpha(Mstar*0.999, ar)[0]; hi = cm.cn_alpha(Mstar*1.001, ar)[0]
chk("supersonic blend continuous at beta*AR=1", lo, hi, 0.01)
# 6. transonic window is flagged, not silently interpolated
_, _, _, tf = cm.cn_alpha(1.0, 1.111)
print(f"[{'PASS' if tf else 'FAIL'}] transonic flag raised at M=1.0"); ok &= tf

print("\n== roll damping against an analytic rectangular fin ==")
# rectangular fin, constant chord c, root at r, span s:
#   int c y^2 dy = c/3 [ (r+s)^3 - r^3 ]
c0, s0, r0 = 0.040, 0.025, 0.031
can = cm.Canard(span=s0, root_chord=c0, tip_chord=c0, sweep_le_deg=0.0)
veh = cm.Vehicle(diameter=2*r0)
a = cm.cn_alpha(0.3, can.ar_eff, can.sweep_half_chord())[0]*cm.interference(can, veh)
want = -2*can.n_fins*a*(c0/3*((r0+s0)**3 - r0**3))/(veh.s_ref*veh.diameter**2)
_, clp, _ = cm.roll_derivatives(can, veh, 0.3)
chk("C_lp trapezoid integral vs closed form", clp, want, 1e-4)

print("\n== roll forcing: hand calculation ==")
ybar = r0 + s0/2                                    # rectangular fin centroid
want = can.n_fins*a*can.area*ybar/(veh.s_ref*veh.diameter)
cld, _, _ = cm.roll_derivatives(can, veh, 0.3)
chk("C_ldelta vs hand calculation", cld, want, 1e-6)

print("\n== RocketPy published trapezoid integral ==")
cr, ct, s1, rt = 0.055, 0.035, 0.025, 0.031
xi = np.linspace(0, s1, 20000)
num = np.trapezoid((cr + (ct-cr)*xi/s1)*(rt+xi)**2, xi)
rp = (s1/12)*((cr+3*ct)*s1**2 + 4*(cr+2*ct)*s1*rt + 6*(cr+ct)*rt**2)
chk("numeric integral vs RocketPy closed form", num, rp, 1e-4)

print("\n== servo derating against the datasheet ==")
sv = cm.Servo()
chk("no-load rate at zero load", cm.available_rate(
        cm.Canard(), cm.Vehicle(), 0.3, 0.0, 0.0, sv, cm.Linkage(1.0))[0],
    sv.rate_noload, 1e-9)
# linkage ratio scaling laws
cap1, cap3 = cm.Linkage(1.0).refer(sv), cm.Linkage(3.0).refer(sv)
chk("linkage: torque scales as n",     cap3["torque"],   3*cap1["torque"], 1e-9)
chk("linkage: rate scales as 1/n",     cap3["rate"],     cap1["rate"]/3, 1e-9)
chk("linkage: deadband scales as 1/n", cap3["deadband"], cap1["deadband"]/3, 1e-9)
chk("linkage: stiffness scales as n^2", cap3["stiffness"], 9*cap1["stiffness"], 1e-9)

print("\n== two-body roll model ==")
can, veh = cm.Canard(), cm.Vehicle()
free = cm.roll_plant(can, veh, cm.Bearing(0.0), 0.5, 15e3, 170.0)
seiz = cm.roll_plant(can, veh, cm.Bearing(1e6), 0.5, 15e3, 170.0)
print(f"       free-bearing forebody tau  {free['tau_fore']:.3f} s")
print(f"       seized-bearing forebody tau {seiz['tau_fore']:.3f} s")
good = seiz["tau_fore"] < free["tau_fore"]
ok &= good
print(f"[{'PASS' if good else 'FAIL'}] seizing the bearing speeds the forebody roll pole")
# seized two-body must match a single rigid body with combined inertia and damping
It = veh.Ixx_fore + veh.Ixx_tail
cld, clp_c, _ = cm.roll_derivatives(can, veh, 0.5)
hd = veh.diameter/(2*170.0)
want = It/abs(15e3*veh.s_ref*veh.diameter*(clp_c + veh.body_Clp + veh.fin_Clp)*hd)
chk("seized bearing == rigid single body tau", seiz["tau_fore"], want, 0.02)

print("\n== ISA ==")
T, p, rho, a = cm.isa(0.0)
chk("ISA sea-level pressure", p, 101325.0, 1e-4)
chk("ISA sea-level density", rho, 1.2250, 2e-3)
chk("ISA sea-level speed of sound", a, 340.29, 1e-3)
T, p, rho, a = cm.isa(11000.0)
chk("ISA 11 km pressure", p, 22632.1, 2e-3)

print("\n" + ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED"))
raise SystemExit(0 if ok else 1)
