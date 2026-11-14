"""Roll-loop stability margins: a numpy port of the margin evaluation in optimise_roll_schedule.m.

Worst case over the flight envelope and +/-30 % on Cl_delta and Cl_p, for the ADR-004 digital chain
(tau_extra 6.6 ms, 60 Hz IMU low-pass). Reproduces the ADR-004 section 3.2 table (c = 15, nominal servo:
PM 66.8 deg, GM 10.8 dB, Ms 1.51) and evaluates the servo values measured in BENCH-01.

    python3 roll_margins.py
"""
import numpy as np
def roll_coeffs(V=100.0, d=0.062, s=0.018, Cr=0.024, Ct=0.012, N=4, C_f=0.003, Lb=0.8):
    y_Cp = d/2 + s*(Cr + 2*Ct)/(3*(Cr + Ct))
    S_c = (Cr + Ct)*s*0.5; AR = s*s/S_c; r = d/2; A = np.pi*r*r
    beta = np.sqrt(max(1 - (V/343)**2, 1e-6)); a = 2*np.pi*AR/(2 + np.sqrt(4 + (AR*beta)**2)); K = 1 + r/(s + r)
    Cl_d = K*N*a*(S_c/A)*(y_Cp/d); k = (Ct - Cr)/s; I3 = ((r+s)**3 - r**3)/3; I4 = ((r+s)**4 - r**4)/4
    Cl_p = -2*K*N*a*(Cr*I3 + k*(I4 - r*I3))/(A*d*d) - 2*C_f*(Lb/d)
    return Cl_d, Cl_p, A
ENV = np.array([[2000,60],[5000,100],[7200,100],[12000,150],[20000,200],[32000,250],[40000,280]], float)
def margins_fr(w, L):
    mag = np.abs(L); ph = np.unwrap(np.angle(L))
    PM = 180.0; k = np.flatnonzero((mag[:-1] >= 1) & (mag[1:] < 1))
    if len(k):
        k = k[0]; t = (1 - mag[k])/(mag[k+1] - mag[k]); PM = 180 + np.degrees(ph[k] + t*(ph[k+1] - ph[k]))
    GM = np.inf; j = np.flatnonzero((ph[:-1] >= -np.pi) & (ph[1:] < -np.pi))
    if len(j):
        j = j[0]; t = (-np.pi - ph[j])/(ph[j+1] - ph[j]); GM = 1/(mag[j] + t*(mag[j+1] - mag[j]))
    return GM, PM
def worst(C, wn, zeta, tau_d, Ixx=2.2e-4, tau_extra=0.0066, lpf_hz=60.0, unc=0.30, gain=1.0):
    Cl_d, Cl_p, A = roll_coeffs(); d = 0.062
    k_b = A*d*Cl_d/Ixx; k_a = A*d*d*Cl_p/(2*Ixx)
    w = np.logspace(-2, np.log10(30*max(wn, 80)), 4000); jw = 1j*w
    H = wn*wn/(jw**2 + 2*zeta*wn*jw + wn*wn)*np.exp(-jw*(tau_d + tau_extra))
    if lpf_hz > 0:
        wl = 2*np.pi*lpf_hz; H = H*wl*wl/(jw**2 + np.sqrt(2)*wl*jw + wl*wl)
    pm, gm, ms = np.inf, np.inf, -np.inf
    for sd in (1-unc, 1, 1+unc):
        for sp in (1-unc, 1, 1+unc):
            for q, v in ENV:
                L = gain*(C/q)*H*(sd*k_b*q/(jw - sp*k_a*q/v))
                G, P = margins_fr(w, L); pm = min(pm, P); gm = min(gm, G); ms = max(ms, np.max(np.abs(1/(1+L))))
    return pm, 20*np.log10(gm), ms
if __name__ == "__main__":
    Cl_d, Cl_p, A = roll_coeffs(); print("Cl_delta %.4f Cl_p %.4f" % (Cl_d, Cl_p))
    rows = [("ADR-004 nominal servo (80, 0.7, 14 ms)", dict(wn=80, zeta=0.7, tau_d=0.014)),
            ("ADR-004 worst corner (Ixx -30%, 52, 0.7, 21 ms)", dict(wn=52, zeta=0.7, tau_d=0.021, Ixx=2.2e-4*0.7)),
            ("measured 130, 0.8, tau_d 5 ms", dict(wn=130, zeta=0.8, tau_d=0.005)),
            ("measured 130, 0.8, tau_d 10 ms", dict(wn=130, zeta=0.8, tau_d=0.010)),
            ("measured 130, 0.8, tau_d 14 ms", dict(wn=130, zeta=0.8, tau_d=0.014)),
            ("measured 110, 0.8, tau_d 14 ms, Ixx -30%", dict(wn=110, zeta=0.8, tau_d=0.014, Ixx=2.2e-4*0.7)),
            ("measured 130, 0.8, tau_d 14+25 ms (small-move start delay)", dict(wn=130, zeta=0.8, tau_d=0.039)),
            ("  same, Ixx -30%", dict(wn=130, zeta=0.8, tau_d=0.039, Ixx=2.2e-4*0.7)),
            ("measured 130, 0.8, tau_d 10+25 ms", dict(wn=130, zeta=0.8, tau_d=0.035)),
            ("  same, Ixx -30%", dict(wn=130, zeta=0.8, tau_d=0.035, Ixx=2.2e-4*0.7)),
            ("scale K=0.75: measured 130, 0.8, 10 ms", dict(wn=130, zeta=0.8, tau_d=0.010, gain=0.75))]
    for C in (12, 15, 18, 20):
        print(f"--- c = {C}")
        for name, kw in rows:
            pm, gm, ms = worst(C, **kw); print("  %-58s PM %5.1f deg  GM %5.1f dB  Ms %.2f" % (name, pm, gm, ms))
