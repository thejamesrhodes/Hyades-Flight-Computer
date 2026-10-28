"""
Streamlit front end for canard_model.  Run:  streamlit run app.py
 
Scope reminder (ADR-001): this characterises the PLANT only. Control laws,
gains and closed-loop response stay in the MATLAB/Simulink toolchain; this
exports the parameter set that toolchain consumes.
"""
import io, json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import streamlit as st
 
import canard_model as cm
 
SER = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e3e2de", "#fcfcfb"
MARGIN_KEYS = ["authority", "granularity", "torque", "divergence", "span", "static_margin"]
 
st.set_page_config(page_title="Canard sizing", layout="wide")
 
 
def style(ax):
    ax.set_facecolor(SURF); ax.grid(True, color=GRID, lw=.7); ax.set_axisbelow(True)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    for s in ("left", "bottom"): ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    return ax
 
 
def fig(w=6.4, h=3.6):
    f, ax = plt.subplots(figsize=(w, h), facecolor=SURF)
    return f, style(ax)
 
 
# ----------------------------------------------------------------- sidebar
sb = st.sidebar
sb.header("Canard planform")
span = sb.slider("exposed semi-span s [mm]", 5.0, 40.0, 18.0, 0.5)/1e3
root = sb.slider("root chord [mm]", 15.0, 90.0, 45.0, 1.0)/1e3
taper = sb.slider("taper ratio λ", 0.2, 1.0, 0.65, 0.01)
sweep_le = sb.slider("LE sweep [deg]", 0.0, 60.0, 25.0, 1.0)
toc = sb.slider("thickness ratio t/c", 0.04, 0.14, 0.08, 0.005)
hinge = sb.slider("hinge line x/c̄", 0.10, 0.50, 0.20, 0.01)
section = sb.selectbox("section", list(cm.SECTIONS), index=0)
nfins = sb.selectbox("number of canards", [3, 4, 6], index=1)
 
sb.header("Linkage + servo")
ratio = sb.slider("linkage ratio n = δ_servo/δ_canard", 1.0, 8.0, 4.0, 0.25)
volt = sb.selectbox("servo supply", ["8.4 V", "7.4 V", "6.0 V", "3.7 V"], index=0)
_SV = {"8.4 V": (0.520, 1200.0), "7.4 V": (0.461, 1000.0),
       "6.0 V": (0.392, 857.0), "3.7 V": (0.245, 545.0)}   # BMS-127WV+ datasheet
stall, rate0 = _SV[volt]
deadband = sb.slider("servo deadband [deg]", 0.0, 0.60, 0.18, 0.01)
backlash = sb.slider("mechanical backlash [deg] — MEASURE", 0.0, 0.60, 0.15, 0.01)
kservo = sb.slider("servo rotational stiffness [N·m/rad] — MEASURE", 2.0, 80.0, 30.0, 1.0)
delay = sb.slider("transport delay [ms]", 0.0, 30.0, 6.0, 0.5)/1e3
 
sb.header("Vehicle")
diam = sb.slider("body diameter [mm]", 40.0, 120.0, 62.0, 1.0)/1e3
Ixx_f = sb.slider("forebody I_xx [g·m²]", 0.2, 10.0, 2.0, 0.1)/1e3
Ixx_t = sb.slider("fin-can I_xx [g·m²]", 0.05, 3.0, 0.4, 0.05)/1e3
fin_clp = -sb.slider("tail fin |C_lp|", 2.0, 30.0, 12.0, 0.5)
 
sb.header("Bearing coupling")
log_cb = sb.slider("log₁₀ c_visc [N·m/(rad/s)]  (−6 = free, +1 = seized)",
                   -6.0, 1.0, -1.7, 0.1)
c_visc = 10**log_cb
 
sb.header("Requirements")
dmax = sb.slider("max deflection [deg]", 4.0, 20.0, 12.0, 0.5)
pdot_req = sb.slider("commandable roll accel [deg/s²]", 100.0, 3000.0, 500.0, 50.0)
p_lc = sb.slider("tolerable roll-rate limit cycle [deg/s]", 1.0, 30.0, 5.0, 0.5)
span_max = sb.slider("exposed span cap [mm] (sign-reversal mitigation)", 10.0, 45.0, 30.0, 1.0)/1e3
cna_budget = sb.slider("canard C_Nα budget on S_ref (static margin)", 0.5, 8.0, 2.5, 0.1)
div_margin = sb.slider("required q_div / q", 1.0, 6.0, 2.0, 0.25)
 
canard = cm.Canard(span=span, root_chord=root, tip_chord=taper*root,
                   sweep_le_deg=sweep_le, thickness_ratio=toc, hinge_frac=hinge,
                   n_fins=nfins, section=section)
veh = cm.Vehicle(diameter=diam, Ixx_fore=Ixx_f, Ixx_tail=Ixx_t, fin_Clp=fin_clp)
servo = cm.Servo(stall_torque=stall, rate_noload_deg=rate0, deadband_deg=deadband,
                 backlash_deg=backlash, stiffness=kservo, delay_s=delay)
link = cm.Linkage(ratio)
brg = cm.Bearing(c_visc=c_visc)
req = dict(delta_max_deg=dmax, pdot_req_deg=pdot_req, p_limit_cycle_budget_deg=p_lc,
           div_margin=div_margin, span_max=span_max, cna_fore_budget=cna_budget)
 
st.title("Canard sizing — open-loop plant")
st.caption("Plant characterisation only. No control law, no gains, no closed-loop response — "
           "those stay in the MATLAB/Simulink toolchain. M 0.8–1.2 carries ±30 % and no "
           "sign-reversal model; treat it as a design-exclusion band.")
 
# ------------------------------------------------------------ flight envelope
DEFAULT_ENV_ROWS = [{"point": e["name"], "altitude_m": e["alt"], "speed": e["V"]}
                    for e in cm.DEFAULT_ENVELOPE]
 
with st.expander("Flight envelope — edit values, add or delete rows", expanded=False):
    c_mode, c_reset = st.columns([3, 1])
    mode = c_mode.radio("speed column is", ["V [m/s]", "Mach"], horizontal=True)
    if c_reset.button("reset to default", width="stretch"):
        st.session_state.pop("env_df", None)
        st.rerun()
    if "env_df" not in st.session_state:
        st.session_state["env_df"] = pd.DataFrame(DEFAULT_ENV_ROWS)
 
    edited = st.data_editor(
        st.session_state["env_df"], num_rows="dynamic", width="stretch",
        key="env_editor",
        column_config={
            "point": st.column_config.TextColumn("point", help="label only"),
            "altitude_m": st.column_config.NumberColumn(
                "altitude [m]", min_value=0.0, max_value=30000.0, step=100.0, format="%.0f"),
            "speed": st.column_config.NumberColumn(
                "speed", min_value=0.0, step=5.0, format="%.1f",
                help="m/s or Mach, per the selector above"),
        })
    st.session_state["env_df"] = edited
 
    envelope = []
    for _, r in edited.iterrows():
        try:
            alt, sp = float(r["altitude_m"]), float(r["speed"])
        except (TypeError, ValueError):
            continue
        if not np.isfinite(alt) or not np.isfinite(sp) or sp <= 0:
            continue
        a_s = cm.isa(alt)[3]
        V = sp if mode.startswith("V") else sp*a_s
        envelope.append(dict(name=str(r["point"]) if r["point"] else f"pt {len(envelope)+1}",
                             alt=alt, V=float(V)))
    if not envelope:
        st.warning("No valid rows — falling back to the default envelope.")
        envelope = cm.DEFAULT_ENVELOPE
 
    prev = []
    for e in envelope:
        _, _, rho, a_s = cm.isa(e["alt"])
        prev.append({"point": e["name"], "alt [m]": e["alt"], "V [m/s]": round(e["V"], 1),
                     "Mach": round(e["V"]/a_s, 3), "q [kPa]": round(0.5*rho*e["V"]**2/1e3, 2),
                     "transonic": cm.TRANSONIC[0] <= e["V"]/a_s <= cm.TRANSONIC[1]})
    st.dataframe(pd.DataFrame(prev), width="stretch", hide_index=True)
    st.caption("Every margin is the worst case over these points, so the envelope decides the "
               "answer as much as the geometry does. Keep a low-q point — rail exit is what "
               "sizes authority. Rows flagged transonic carry ±30 % and no sign-reversal model.")
 
res = cm.evaluate(canard, veh, servo, link, brg, envelope, req)
g, cap = res["geometry"], res["capability"]
 
c1, c2, c3, c4, c5 = st.columns(5)
c1.metric("panel area", f"{g['S_f_mm2']:.0f} mm²")
c2.metric("effective AR", f"{g['AR_eff']:.2f}")
c3.metric("MAC", f"{g['MAC_mm']:.1f} mm")
c4.metric("panel mass", f"{canard.mass()*1e3:.1f} g")
c5.metric("worst margin", f"{res['score']:.2f}",
          delta="feasible" if res["score"] >= 1 else "infeasible",
          delta_color="normal" if res["score"] >= 1 else "inverse")
 
t1, t2, t3, t4 = st.tabs(["Design point", "Sizing sweep", "Envelope", "Export"])
 
# ------------------------------------------------------------- design point
with t1:
    a, b = st.columns([1, 1])
    with a:
        f, ax = fig(5.6, 3.4)
        ks = MARGIN_KEYS
        vals = [min(res["margins"][k], 6) for k in ks]
        cols = [SER[2] if res["margins"][k] >= 1 else SER[1] for k in ks]
        ax.barh(range(len(ks)), vals, color=cols, height=.6)
        ax.axvline(1.0, color=INK2, lw=1.4, ls=(0, (4, 2)))
        ax.set_yticks(range(len(ks))); ax.set_yticklabels([k.replace("_", " ") for k in ks])
        ax.invert_yaxis(); ax.set_xlim(0, 6.2)
        ax.set_xlabel("normalised margin  (≥ 1 = satisfied, clipped at 6)", color=INK2)
        for i, k in enumerate(ks):
            ax.text(min(res["margins"][k], 6)+.08, i, f"{res['margins'][k]:.2f}",
                    va="center", fontsize=9, color=INK2)
        st.pyplot(f, clear_figure=True)
        st.caption("The binding constraint is whichever bar is shortest. "
                   "Raise the linkage ratio before changing the planform — it moves "
                   "granularity, torque and divergence together at no aerodynamic cost.")
    with b:
        f, ax = fig(5.6, 3.4)
        r0 = 0.5*veh.diameter
        xle = np.array([0, span*np.tan(np.radians(sweep_le))])
        xs = [0, xle[1], xle[1]+taper*root, root, 0]
        ys = [r0, r0+span, r0+span, r0, r0]
        ax.fill(xs, ys, color=SER[0], alpha=.25)
        ax.plot(xs, ys, color=SER[0], lw=2)
        ax.axhline(r0, color=INK2, lw=2)
        xh = hinge*canard.mac
        ax.plot([xh, xh + span*np.tan(np.radians(sweep_le))], [r0, r0+span],
                color=SER[1], lw=2, ls=(0, (5, 2)))
        ax.text(xh, r0+span*1.06, "hinge", color=SER[1], fontsize=9)
        ax.set_aspect("equal"); ax.set_xlabel("chordwise [m]", color=INK2)
        ax.set_ylabel("from roll axis [m]", color=INK2)
        st.pyplot(f, clear_figure=True)
 
    st.subheader("Flight envelope")
    df = pd.DataFrame(res["points"])[
        ["name", "M", "q", "C_ldelta", "delta_req_deg", "pdot_at_dmax_deg",
         "p_limit_cycle_deg", "hinge_Nm_at_dmax", "torque_frac_at_dmax",
         "rate_avail_deg", "q_div_kPa", "tau_fore", "transonic"]]
    df["q"] = df["q"]/1000
    st.dataframe(df.style.format({
        "M": "{:.2f}", "q": "{:.1f}", "C_ldelta": "{:.2f}", "delta_req_deg": "{:.2f}",
        "pdot_at_dmax_deg": "{:.0f}", "p_limit_cycle_deg": "{:.2f}",
        "hinge_Nm_at_dmax": "{:.3f}", "torque_frac_at_dmax": "{:.2f}",
        "rate_avail_deg": "{:.0f}", "q_div_kPa": "{:.0f}", "tau_fore": "{:.3f}"}),
        width="stretch")
    st.caption("q in kPa. `delta_req_deg` is the deflection needed for the commandable "
               "roll acceleration; where it falls below the canard-referred deadband "
               f"({np.degrees(cap['deadband']):.3f}°) the loop is operating inside the actuator's "
               "dead zone and limit-cycle amplitude, not authority, is what matters.")
 
# -------------------------------------------------------------- sizing sweep
with t2:
    st.write("Maximin score over the planform grid: **max over designs of the "
             "minimum normalised margin**. Flat regions mean the constraint set, "
             "not the objective, is deciding.")
    ns = st.slider("grid resolution", 7, 21, 13, 2)
    sp = np.linspace(0.006, 0.035, ns)
    rt = np.linspace(0.015, 0.090, ns)
    S, Mg, A = cm.sweep(veh, servo, link, brg, envelope, req, sp, rt,
                        taper=taper, sweep_le_deg=sweep_le, thickness_ratio=toc,
                        hinge_frac=hinge, n_fins=nfins, section=section)
    i, j = np.unravel_index(np.argmax(S), S.shape)
 
    a, b = st.columns(2)
    with a:
        f, ax = fig(5.8, 4.4)
        im = ax.pcolormesh(rt*1e3, sp*1e3, np.clip(S, 0, 3), cmap="viridis", shading="auto")
        cs = ax.contour(rt*1e3, sp*1e3, A, levels=8, colors=[INK2], linewidths=.7, alpha=.7)
        ax.clabel(cs, fmt="%.0f mm²", fontsize=7)
        ax.contour(rt*1e3, sp*1e3, S, levels=[1.0], colors=["#ffffff"], linewidths=2)
        ax.plot(rt[j]*1e3, sp[i]*1e3, "o", ms=11, mfc="none", mec="#ffffff", mew=2)
        ax.plot(root*1e3, span*1e3, "x", ms=11, color="#ffffff", mew=2)
        ax.set_xlabel("root chord [mm]", color=INK2); ax.set_ylabel("exposed semi-span [mm]", color=INK2)
        f.colorbar(im, ax=ax, label="worst margin")
        st.pyplot(f, clear_figure=True)
        st.caption("White contour = feasibility boundary. ○ best on grid, × your current point. "
                   "Grey contours are panel area.")
    with b:
        binding = np.zeros_like(S, dtype=int)
        stack = np.stack([Mg[k] for k in MARGIN_KEYS])
        binding = np.argmin(stack, axis=0)
        f, ax = fig(5.8, 4.4)
        im = ax.pcolormesh(rt*1e3, sp*1e3, binding, cmap="tab10", vmin=0, vmax=9, shading="auto")
        ax.set_xlabel("root chord [mm]", color=INK2); ax.set_ylabel("exposed semi-span [mm]", color=INK2)
        cb = f.colorbar(im, ax=ax, ticks=range(len(MARGIN_KEYS)))
        cb.ax.set_yticklabels([k.replace("_", " ") for k in MARGIN_KEYS])
        ax.set_title("which constraint binds", color=INK, fontsize=11, loc="left")
        st.pyplot(f, clear_figure=True)
 
    st.info(f"Best on grid: semi-span **{sp[i]*1e3:.1f} mm**, root chord "
            f"**{rt[j]*1e3:.1f} mm** (area {A[i,j]:.0f} mm², worst margin {S[i,j]:.2f}).")
 
    st.write("**Linkage ratio sweep** — the cheapest lever in the design.")
    ratios = np.arange(1.0, 8.5, 0.5)
    sc = []
    for n in ratios:
        rr = cm.evaluate(canard, veh, servo, cm.Linkage(float(n)), brg,
                         envelope, req)
        sc.append([rr["margins"][k] for k in MARGIN_KEYS] + [rr["score"]])
    sc = np.array(sc)
    f, ax = fig(9.0, 3.4)
    for k, key in enumerate(MARGIN_KEYS):
        if key in ("span", "static_margin"): continue
        ax.plot(ratios, np.clip(sc[:, k], 0, 12), lw=2, color=SER[k % 4], label=key)
    ax.plot(ratios, np.clip(sc[:, -1], 0, 12), lw=2.6, color=INK, label="worst margin")
    ax.axhline(1, color=INK2, ls=(0, (4, 2)), lw=1.2)
    ax.set_xlabel("linkage ratio n", color=INK2); ax.set_ylabel("normalised margin", color=INK2)
    ax.set_yscale("log"); ax.legend(fontsize=8, ncol=5, frameon=False)
    st.pyplot(f, clear_figure=True)
 
# ------------------------------------------------------------------ envelope
with t3:
    Ms = np.linspace(0.05, 2.0, 300)
    cld, cna_, hm, qdv = [], [], [], []
    alt = 1200.0
    _, _, rho, a_s = cm.isa(alt)
    for M in Ms:
        V = M*a_s; q = 0.5*rho*V*V
        c_, _, _ = cm.roll_derivatives(canard, veh, M)
        cld.append(c_); cna_.append(cm.cn_alpha(M, canard.ar_eff, canard.sweep_half_chord())[0])
        hm.append(cm.hinge_moment(canard, veh, M, q, np.radians(dmax)))
        qdv.append(cm.q_divergence(canard, veh, M, cap["stiffness"])/1e3)
 
    a, b = st.columns(2)
    with a:
        f, ax = fig(5.8, 3.4)
        ax.axvspan(*cm.TRANSONIC, color=SER[1], alpha=.12)
        ax.plot(Ms, cna_, lw=2, color=SER[0])
        ax.text(np.mean(cm.TRANSONIC), max(cna_)*.95, "not modelled", ha="center",
                color=SER[1], fontsize=9)
        ax.set_xlabel("Mach", color=INK2); ax.set_ylabel("panel $C_{N\\alpha}$ [/rad]", color=INK2)
        st.pyplot(f, clear_figure=True)
        f, ax = fig(5.8, 3.4)
        ax.axvspan(*cm.TRANSONIC, color=SER[1], alpha=.12)
        ax.plot(Ms, cld, lw=2, color=SER[2])
        ax.set_xlabel("Mach", color=INK2); ax.set_ylabel("$C_{l\\delta}$ [/rad]", color=INK2)
        st.pyplot(f, clear_figure=True)
    with b:
        f, ax = fig(5.8, 3.4)
        ax.axvspan(*cm.TRANSONIC, color=SER[1], alpha=.12)
        ax.plot(Ms, np.array(hm), lw=2, color=SER[3], label=f"hinge moment at {dmax:.0f}°")
        ax.axhline(cap["torque"], color=SER[1], lw=2, ls=(0, (5, 2)),
                   label=f"available {cap['torque']:.2f} N·m")
        ax.axhline(0, color=GRID, lw=1)
        ax.set_xlabel("Mach", color=INK2); ax.set_ylabel("N·m per panel", color=INK2)
        ax.legend(fontsize=8, frameon=False)
        st.pyplot(f, clear_figure=True)
        f, ax = fig(5.8, 3.4)
        cbs = np.logspace(-6, 1, 60)
        taus, lcs = [], []
        for cbv in cbs:
            rr = cm.evaluate(canard, veh, servo, link, cm.Bearing(c_visc=float(cbv)),
                             envelope, req)
            taus.append(max(p["tau_fore"] for p in rr["points"]))
            lcs.append(max(p["p_limit_cycle_deg"] for p in rr["points"]))
        ax.semilogx(cbs, taus, lw=2, color=SER[0], label="slowest forebody τ [s]")
        ax.semilogx(cbs, lcs, lw=2, color=SER[1], label="worst limit cycle [°/s]")
        ax.axvline(c_visc, color=INK2, ls=(0, (3, 3)), lw=1.2)
        ax.set_xlabel("bearing viscous coupling $c_{visc}$ [N·m/(rad/s)]", color=INK2)
        ax.legend(fontsize=8, frameon=False)
        st.pyplot(f, clear_figure=True)
        st.caption("Free bearing (left) leaves the controlled forebody a near-integrator in "
                   "roll: the plant pole approaches the origin and quantisation wanders "
                   "instead of settling. Seized (right) restores full fin damping.")
 
# -------------------------------------------------------------------- export
with t4:
    payload = dict(envelope=envelope, canard=canard.__dict__, vehicle=veh.__dict__, servo=servo.__dict__,
                   linkage=link.__dict__, bearing=brg.__dict__, requirements=req,
                   capability={k: float(v) for k, v in cap.items()},
                   geometry=g, margins=res["margins"], score=res["score"],
                   points=res["points"],
                   note=("C_ldelta and C_lp referenced to S_ref = pi d^2/4 and length d, "
                         "p_hat = p d / (2 V). transonic=true points carry +/-30% and no "
                         "sign-reversal model."))
    txt = json.dumps(payload, indent=2, default=float)
    st.download_button("Download parameter set (JSON) for the MATLAB loop",
                       txt, file_name="canard_plant.json", mime="application/json")
    st.code(txt[:2500] + ("\n…" if len(txt) > 2500 else ""), language="json")
 
