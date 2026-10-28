/* canard plant model — direct port of canard_model.py.
   Verified against the Python reference in reference.json (see test_physics.mjs). */
const GAMMA = 1.4, R_AIR = 287.053, G0 = 9.80665;
export const TRANSONIC = [0.80, 1.20], TRANSONIC_BAND = 0.30;
export const DEG = Math.PI / 180;

export function isa(alt) {
  let T, p;
  if (alt < 11000) { T = 288.15 - 0.0065 * alt; p = 101325 * Math.pow(T / 288.15, -G0 / (-0.0065 * R_AIR)); }
  else { T = 216.65; p = 22632.1 * Math.exp(-G0 * (alt - 11000) / (R_AIR * T)); }
  const rho = p / (R_AIR * T);
  return { T, p, rho, a: Math.sqrt(GAMMA * R_AIR * T) };
}

/* trapezoid over n samples of f on [0, L] — matches numpy.trapezoid with linspace */
function trapz(f, L, n = 400) {
  const h = L / (n - 1); let s = 0;
  for (let i = 0; i < n; i++) s += f(i * h) * (i === 0 || i === n - 1 ? 0.5 : 1);
  return s * h;
}

/* Blue Bird BMS-127WV+ manufacturer table: (V, stall N m, 60-deg time s). */
export const BMS127WV = { V: [3.7, 6.0, 7.4, 8.4],
  T: [2.5, 4.0, 4.7, 5.3].map((x) => x * 9.80665e-2),
  t60: [0.11, 0.07, 0.06, 0.05] };

/* Separate slew from accel in a published 60-degree travel time by fitting
   t60 = a/V + b.  `b` is the accel/decel overhead; it yields the true no-load
   slew rate and the reflected output inertia. */
export function servoFromDatasheet(voltage, d = BMS127WV) {
  const n = d.V.length;
  let sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (let i = 0; i < n; i++) { const x = 1 / d.V[i], y = d.t60[i];
    sx += x; sy += y; sxx += x * x; sxy += x * y; }
  const a = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  const b = (sy - a * sx) / n;
  const w0 = d.t60.map((t) => (60 * DEG) / (t - b));
  let J = 0; for (let i = 0; i < n; i++) J += (b * d.T[i]) / (2 * w0[i]);
  J /= n;
  const interp = (arr) => {
    if (voltage <= d.V[0]) return arr[0];
    if (voltage >= d.V[n - 1]) return arr[n - 1];
    let i = 0; while (d.V[i + 1] < voltage) i++;
    const f = (voltage - d.V[i]) / (d.V[i + 1] - d.V[i]);
    return arr[i] + f * (arr[i + 1] - arr[i]);
  };
  const T = interp(d.T);
  return { stallTorque: T, rateNoload: interp(w0),
           rateSpec: interp(d.t60.map((t) => (60 * DEG) / t)),
           overheadS: b, JOut: J, alphaMax: T / J,
           slewScaling: d.t60[0] / d.t60[n - 1],
           slewPred: d.V[n - 1] / d.V[0],
           inertiaPred: Math.sqrt(d.V[n - 1] / d.V[0]),
           peakPower: (T * interp(w0)) / 4 };
}

export function geom(c) {
  const { span: s, rootChord: cr, tipChord: ct, sweepLeDeg: swle, thicknessRatio: tc, hingeFrac: hf } = c;
  const area = 0.5 * (cr + ct) * s;
  const taper = ct / cr;
  /* Aspect ratio convention: "panel" = s^2/S_f (the exposed panel's own),
     "image" = 2 s^2/S_f (mirrored in the body surface). They differ by 2x,
     which is ~1.9x in C_Nalpha at AR ~ 1. Checked against Barrowman's closed
     form for a fin set, "panel" lands within ~10% and "image" is ~107% high,
     so "panel" is the default and it matches the Simulink plant function. */
  const arBase = s * s / area;
  const arEff = (c.arConvention === "image") ? 2 * arBase : arBase;
  const mac = (2 / 3) * cr * (1 + taper + taper * taper) / (1 + taper);
  const yCentroid = s * (cr + 2 * ct) / (3 * (cr + ct));
  const dx = 0.5 * ct - 0.5 * cr;
  const shc = Math.atan((s * Math.tan(swle * DEG) + dx) / s);
  const chord = (xi) => cr + (ct - cr) * xi / s;
  const J = trapz((xi) => {
    const ch = chord(xi), xh = hf * ch, teq = 0.7 * tc * ch;
    return c.materialDensity * teq * (Math.pow(ch - xh, 3) + Math.pow(xh, 3)) / 3;
  }, s);
  const mass = trapz((xi) => c.materialDensity * 0.7 * tc * chord(xi) * chord(xi), s);
  const sDelta = trapz((xi) => {
    const ch = chord(xi), xh = hf * ch, teq = 0.7 * tc * ch;
    return c.materialDensity * teq * ch * (0.5 * ch - xh);
  }, s);
  return { area, taper, arEff, mac, yCentroid, shc, chord, J, mass, sDelta };
}

export function cnAlpha(M, ar, shc = 0) {
  const sub = (Mx) => {
    const b2 = Math.max(1 - Mx * Mx, 1e-6);
    return 2 * Math.PI * ar / (2 + Math.sqrt(4 + ar * ar * b2 * (1 + Math.pow(Math.tan(shc), 2) / b2)));
  };
  const sup = (Mx) => {
    const b = Math.sqrt(Math.max(Mx * Mx - 1, 1e-9)), bar = b * ar;
    if (bar >= 1) return (4 / b) * (1 - 1 / (2 * bar));
    return (Math.PI * ar / 2) * (1 + (4 / Math.PI - 1) * bar);
  };
  if (M < TRANSONIC[0]) { const v = sub(M); return { v, lo: v * 0.9, hi: v * 1.1, transonic: false }; }
  if (M > TRANSONIC[1]) { const v = sup(M); return { v, lo: v * 0.9, hi: v * 1.1, transonic: false }; }
  const lo = sub(TRANSONIC[0]), hi = sup(TRANSONIC[1]);
  const f = (M - TRANSONIC[0]) / (TRANSONIC[1] - TRANSONIC[0]);
  const v = lo + f * (hi - lo);
  return { v, lo: v * (1 - TRANSONIC_BAND), hi: v * (1 + TRANSONIC_BAND), transonic: true };
}

export const xAcFrac = (M) => M < TRANSONIC[0] ? 0.25 : (M > TRANSONIC[1] ? 0.50
  : 0.25 + 0.25 * (M - TRANSONIC[0]) / (TRANSONIC[1] - TRANSONIC[0]));

export const sRef = (veh) => 0.25 * Math.PI * veh.diameter * veh.diameter;
export const interference = (c, veh) => 1 + (0.5 * veh.diameter) / (c.span + 0.5 * veh.diameter);

export function rollDerivatives(c, veh, M) {
  const G = geom(c);
  const ca = cnAlpha(M, G.arEff, G.shc);
  const a = ca.v * interference(c, veh);
  const r = 0.5 * veh.diameter;
  const cld = c.nFins * a * G.area * (r + G.yCentroid) / (sRef(veh) * veh.diameter);
  const integ = trapz((xi) => G.chord(xi) * Math.pow(r + xi, 2), c.span);
  const clp = -2 * c.nFins * a * integ / (sRef(veh) * veh.diameter * veh.diameter);
  return { cld, clp, transonic: ca.transonic };
}

export function hingeMoment(c, veh, M, q, delta) {
  const G = geom(c), a = cnAlpha(M, G.arEff, G.shc).v * interference(c, veh);
  return q * G.area * a * delta * (xAcFrac(M) - c.hingeFrac) * G.mac;
}

export function qDivergence(c, veh, M, kDelta) {
  const G = geom(c), a = cnAlpha(M, G.arEff, G.shc).v * interference(c, veh);
  const arm = (xAcFrac(M) - c.hingeFrac) * G.mac;
  return arm <= 0 ? Infinity : kDelta / (G.area * a * arm);
}

/* Describing function of backlash: |N|/K and phase [deg]. Unlike a dead zone
   this is complex — backlash injects phase lag and is the nonlinearity that can
   actually destabilise an otherwise stable loop. Reported as a flag only. */
export function backlashDf(ratio) {
  if (!(ratio > 1)) return { mag: 0, phase: -90 };
  const u = 1 - 2 / ratio;
  const re = (1 / Math.PI) * (Math.PI / 2 + Math.asin(u) + u * Math.sqrt(Math.max(1 - u * u, 0)));
  const im = -(4 / (Math.PI * ratio)) * (1 - 1 / ratio);
  return { mag: Math.hypot(re, im), phase: Math.atan2(im, re) / DEG };
}

/* Crossover above which the transport delay alone eats more than
   phaseBudgetDeg of phase margin. */
export function omegaCDelayCeiling(servo, phaseBudgetDeg) {
  const tau = servo.delayS + 1 / servo.frameHz;
  return tau > 0 ? (phaseBudgetDeg * DEG) / tau : Infinity;
}

export function refer(servo, ratio) {
  const n = ratio;
  return {
    torque: n * servo.stallTorque * servo.efficiency,
    rate: servo.rateNoloadDeg * DEG / n,
    deadband: servo.deadbandDeg * DEG / n,
    resolution: servo.resolutionDeg * DEG / n,
    backlash: servo.backlashDeg * DEG / n,
    stiffness: n * n * servo.stiffness * servo.efficiency,
  };
}

export function availableRate(c, veh, M, q, delta, servo, ratio) {
  const cap = refer(servo, ratio);
  const load = Math.abs(hingeMoment(c, veh, M, q, delta));
  const frac = cap.torque > 0 ? Math.min(load / cap.torque, 1) : 1;
  return { rate: cap.rate * Math.max(1 - frac, 0), frac };
}

export function rollPlant(c, veh, brg, M, q, V) {
  const { cld, clp, transonic } = rollDerivatives(c, veh, M);
  const k = q * sRef(veh) * veh.diameter;
  const hd = veh.diameter / (2 * Math.max(V, 1e-6));
  const dFore = k * (clp + veh.bodyClp) * hd, dTail = k * veh.finClp * hd, cb = brg.cVisc;
  const A = [[(dFore - cb) / veh.IxxFore, cb / veh.IxxFore],
             [cb / veh.IxxTail, (dTail - cb) / veh.IxxTail]];
  const B0 = k * cld / veh.IxxFore;
  const tr = A[0][0] + A[1][1], det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  const disc = tr * tr - 4 * det;
  let re1, re2;
  if (disc >= 0) { const s = Math.sqrt(disc); re1 = (tr + s) / 2; re2 = (tr - s) / 2; }
  else { re1 = re2 = tr / 2; }
  // p_ss = solve(-A, B)  ->  first component
  const pss = det !== 0 ? (-A[1][1] * B0) / det : Infinity;
  const slow = (re1 < 0 && re2 < 0) ? Math.min(Math.abs(re1), Math.abs(re2)) : 0;
  return { A, tauFore: slow > 0 ? 1 / slow : Infinity, pSsPerRad: pss,
           cld, clp, pdotPerRad: B0, transonic, eig: [re1, re2] };
}

export const MARGIN_KEYS = ["authority", "rate", "granularity", "torque",
                            "divergence", "span", "static margin", "root bending"];

export function evaluate(c, veh, servo, ratio, brg, envelope, req) {
  const cap = refer(servo, ratio), G = geom(c);
  const dmax = req.deltaMaxDeg * DEG;
  const MU = 1.79e-5;
  const pts = [];
  const JTotal = G.J + ratio * ratio * servo.JOut;
  const clDistTail = veh.finCldelta * veh.finCantDeg * DEG;
  const clDistCan = veh.canardRigErrorDeg * DEG;
  let mAuth = Infinity, mRate = Infinity, mGran = Infinity, mTorque = Infinity,
      mDiv = Infinity, mBend = Infinity, smWorst = Infinity;

  for (const e of envelope) {
    const at = isa(e.alt), V = e.V, M = V / at.a, q = 0.5 * at.rho * V * V;
    const pl = rollPlant(c, veh, brg, M, q, V);
    const cld = pl.cld;

    const dTrim = brg.cVisc > 0 ? (cld > 0 ? clDistTail / cld + clDistCan : Infinity) : clDistCan;
    const dMan = pl.pdotPerRad > 0 ? (req.pdotReqDeg * DEG) / pl.pdotPerRad : Infinity;
    const dReq = dTrim + dMan;

    const step = cap.deadband + cap.backlash;
    const pdotGran = pl.pdotPerRad * step;
    const pLc = pdotGran / req.omegaC;
    const bHalf = 0.5 * cap.backlash;
    const aOverB = bHalf > 0 ? Math.min(dReq, dmax) / bHalf : Infinity;
    const bl = isFinite(aOverB) ? backlashDf(aOverB) : { mag: 1, phase: 0 };

    const dRateReq = req.omegaC * Math.min(dMan, dmax);
    const dAccelReq = req.omegaC * req.omegaC * Math.min(dMan, dmax);
    const hDmax = hingeMoment(c, veh, M, q, dmax);
    const tReqOp = Math.abs(hingeMoment(c, veh, M, q, Math.min(dReq, dmax))) + JTotal * dAccelReq;
    const fracMax = cap.torque > 0 ? Math.min(Math.abs(hDmax) / cap.torque, 1) : 1;
    const rateAvail = availableRate(c, veh, M, q, Math.min(dReq, dmax), servo, ratio).rate;
    const qdiv = qDivergence(c, veh, M, cap.stiffness);

    const aCn = cnAlpha(M, G.arEff, G.shc).v * interference(c, veh);
    const mRoot = q * G.area * aCn * dmax * G.yCentroid;
    const cnaCan = c.nFins * aCn * G.area / sRef(veh);
    const cnaTot = cnaCan + veh.cnaRest;
    const xCp = (veh.cnaRest * veh.xCpRest + cnaCan * veh.xCanard) / cnaTot;
    const smCal = (xCp - veh.xCg) / veh.diameter;
    const reC = at.rho * V * G.mac / MU;

    mAuth = Math.min(mAuth, dReq > 0 ? dmax / dReq : Infinity);
    mRate = Math.min(mRate, dRateReq > 0 ? rateAvail / dRateReq : Infinity);
    mGran = Math.min(mGran, pLc > 0 ? (req.pFloorBudgetDeg * DEG) / pLc : Infinity);
    mTorque = Math.min(mTorque, fracMax > 0 ? 1 / fracMax : Infinity);
    mDiv = Math.min(mDiv, isFinite(qdiv) ? qdiv / (q * req.divMargin) : Infinity);
    mBend = Math.min(mBend, mRoot > 0 ? req.mRootAllow / mRoot : Infinity);
    smWorst = Math.min(smWorst, smCal);

    pts.push({ name: e.name, M, q, V, cld, clp: pl.clp, pdotPerRad: pl.pdotPerRad,
      pdotAtDmaxDeg: (pl.pdotPerRad * dmax) / DEG,
      deltaTrimDeg: dTrim / DEG, deltaManDeg: dMan / DEG, deltaReqDeg: dReq / DEG,
      pdotGranDeg: pdotGran / DEG, pFloorDeg: pLc / DEG,
      backlashAOverB: aOverB, backlashPhaseLagDeg: bl.phase,
      rateReqDeg: dRateReq / DEG, rateAvailDeg: rateAvail / DEG,
      hingeNmAtDmax: hDmax, torqueReqOpNm: tReqOp, torqueFracAtDmax: fracMax,
      mRootNm: mRoot, staticMarginCal: smCal, reynolds: reC,
      qDivKPa: qdiv / 1000, tauFore: pl.tauFore,
      pSsDegPerRad: pl.pSsPerRad / DEG, transonic: pl.transonic });
  }

  const margins = { authority: mAuth, rate: mRate, granularity: mGran, torque: mTorque,
    divergence: mDiv, span: req.spanMax / c.span,
    "static margin": smWorst / req.smMinCal, "root bending": mBend };
  const score = Math.min(...Object.values(margins));
  return { points: pts, margins, score, capability: cap, geometry: G, JTotal };
}

export const DEFAULTS = {
  canard: { span: 0.018, rootChord: 0.045, tipChord: 0.02925, sweepLeDeg: 25, thicknessRatio: 0.08,
            hingeFrac: 0.20, nFins: 4, section: "smoothed hexagon", materialDensity: 1600,
            arConvention: "panel" },
  vehicle: { diameter: 0.062, IxxFore: 5.0e-4, IxxTail: 1.1e-4, finClp: -12.0, bodyClp: -0.15,
             finCantDeg: 0.20, finCldelta: 8.0, canardRigErrorDeg: 0.15,
             xCg: 0.700, xCanard: 0.260, xCpRest: 0.900, cnaRest: 22.0 },
  servo: { stallTorque: 0.5198, rateNoloadDeg: 1355, deadbandDeg: 0.21, resolutionDeg: 0.029,
           backlashDeg: 0.15, delayS: 0.006, frameHz: 250, stiffness: 30, efficiency: 0.85,
           JOut: 6.76e-5 },
  ratio: 4.0,
  bearing: { cVisc: 0.02 },
  req: { deltaMaxDeg: 12, pdotReqDeg: 500, pFloorBudgetDeg: 5, omegaC: 30, phaseBudgetDeg: 20,
         divMargin: 2, spanMax: 0.030, smMinCal: 2.0, mRootAllow: 3.0 },
  envelope: [ { name: "rail exit", alt: 200, V: 30 }, { name: "mid boost", alt: 500, V: 150 },
              { name: "max q", alt: 1200, V: 270 }, { name: "burnout", alt: 2000, V: 310 },
              { name: "coast", alt: 3000, V: 150 } ],
};
