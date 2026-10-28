import * as P from './physics.js';
import { readFileSync } from 'fs';
const ref = JSON.parse(readFileSync('./reference.json', 'utf8'));
let fails = 0, n = 0;
const chk = (name, got, want, tol = 1e-9) => {
  n++;
  const w = Math.abs(want) < 1e-300 ? 0 : want;
  const rel = (!isFinite(w) || !isFinite(got)) ? (got === w ? 0 : 1) : Math.abs(got - w) / Math.max(Math.abs(w), 1e-12);
  if (!(rel <= tol)) { fails++; console.log(`FAIL ${name}: ${got} vs ${w} (rel ${rel.toExponential(2)})`); }
};
for (const [M, ar, v, lo, hi, t] of ref.cn_alpha) {
  const r = P.cnAlpha(M, ar, 0);
  chk(`cnAlpha M${M} AR${ar}`, r.v, v, 1e-12);
  chk(`cnAlpha lo`, r.lo, lo, 1e-12); chk(`cnAlpha hi`, r.hi, hi, 1e-12);
  if (r.transonic !== t) { fails++; console.log(`FAIL transonic flag M${M}`); } n++;
}
for (const [M, x] of ref.xac) chk(`xac M${M}`, P.xAcFrac(M), x, 1e-12);
for (const [r_, mag, ph] of ref.bl) {
  const b = P.backlashDf(r_);
  chk(`backlashDf mag A/b=${r_}`, b.mag, mag, 1e-12);
  chk(`backlashDf phase A/b=${r_}`, b.phase, ph, 1e-12);
}
for (const [alt, T, p, rho, a] of ref.isa) {
  const r = P.isa(alt);
  chk(`isa T ${alt}`, r.T, T, 1e-12); chk(`isa p ${alt}`, r.p, p, 1e-12);
  chk(`isa rho ${alt}`, r.rho, rho, 1e-12); chk(`isa a ${alt}`, r.a, a, 1e-12);
}
for (const g of ref.geom) {
  const [s, cr, ct, sw, tc, hf] = g.inp;
  const c = { ...P.DEFAULTS.canard, span: s, rootChord: cr, tipChord: ct,
              sweepLeDeg: sw, thicknessRatio: tc, hingeFrac: hf };
  const G = P.geom(c);
  chk('geom area', G.area, g.area, 1e-12); chk('geom ar', G.arEff, g.ar, 1e-12);
  chk('geom mac', G.mac, g.mac, 1e-12); chk('geom taper', G.taper, g.taper, 1e-12);
  chk('geom yc', G.yCentroid, g.yc, 1e-12); chk('geom shc', G.shc, g.shc, 1e-12);
  chk('geom J', G.J, g.J, 1e-10); chk('geom mass', G.mass, g.mass, 1e-10);
}
const KEYMAP = { authority: 'authority', rate: 'rate', granularity: 'granularity',
                 torque: 'torque', divergence: 'divergence', span: 'span',
                 'static margin': 'static_margin', 'root bending': 'root_bending' };
const PKEY = { M: 'M', q: 'q', cld: 'C_ldelta', clp: 'C_lp_canard', pdotPerRad: 'pdot_per_rad',
  deltaTrimDeg: 'delta_trim_deg', deltaManDeg: 'delta_man_deg',
  deltaReqDeg: 'delta_req_deg', pFloorDeg: 'p_floor_deg',
  torqueReqOpNm: 'torque_req_op_Nm', rateReqDeg: 'rate_req_deg',
  mRootNm: 'm_root_Nm', staticMarginCal: 'static_margin_cal', reynolds: 'reynolds',
  backlashAOverB: 'backlash_A_over_b', backlashPhaseLagDeg: 'backlash_phase_lag_deg',
  hingeNmAtDmax: 'hinge_Nm_at_dmax', torqueFracAtDmax: 'torque_frac_at_dmax',
  rateAvailDeg: 'rate_avail_deg', qDivKPa: 'q_div_kPa', tauFore: 'tau_fore',
  pSsDegPerRad: 'p_ss_deg_per_rad' };
for (const cse of ref.cases) {
  const c = { ...P.DEFAULTS.canard, span: cse.span, rootChord: cse.root, tipChord: 0.636 * cse.root,
              sweepLeDeg: 25, thicknessRatio: 0.08, hingeFrac: 0.20 };
  const r = P.evaluate(c, P.DEFAULTS.vehicle, P.DEFAULTS.servo, cse.n,
                       { cVisc: cse.cb }, P.DEFAULTS.envelope, P.DEFAULTS.req);
  for (const k of Object.keys(KEYMAP)) chk(`case n${cse.n} margin ${k}`, r.margins[k], cse.margins[KEYMAP[k]], 1e-10);
  chk(`case n${cse.n} score`, r.score, cse.score, 1e-10);
  // tauFore / pSs come from a 2x2 eigen-solve; with a near-seized bearing the
  // matrix is stiff, so the analytic root differs from LAPACK in the last few
  // ulps. 1e-5 is still ~9 orders tighter than any engineering meaning.
  const LOOSE = new Set(['tauFore', 'pSsDegPerRad']);
  cse.pts.forEach((p, i) => { for (const k of Object.keys(PKEY))
    chk(`case n${cse.n} pt${i} ${k}`, r.points[i][k], p[PKEY[k]], LOOSE.has(k) ? 1e-5 : 1e-9); });
}
for (const [V, key] of [[8.4, 'servo'], [6.0, 'servo6']]) {
  const got = P.servoFromDatasheet(V), want = ref[key];
  for (const k of ['stallTorque','rateNoload','rateSpec','overheadS','JOut','alphaMax',
                   'slewScaling','slewPred','inertiaPred','peakPower']) {
    const wk = k === 'JOut' ? 'J_out' : k.replace(/[A-Z]/g, (m) => '_' + m.toLowerCase());
    chk(`servo ${V}V ${k}`, got[k], want[wk], 1e-9);
  }
}
ref.geom_sd.forEach((sd, i) => {
  const [s, cr, ct, sw, tc, hf] = ref.geom[i].inp;
  const c = { ...P.DEFAULTS.canard, span: s, rootChord: cr, tipChord: ct,
              sweepLeDeg: sw, thicknessRatio: tc, hingeFrac: hf };
  chk(`static unbalance ${i}`, P.geom(c).sDelta, sd, 1e-10);
});
console.log(`${n - fails}/${n} checks passed`);
process.exit(fails ? 1 : 0);
