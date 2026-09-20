function delta_cmd = controller(p_e, r_e, t, P)
%CONTROLLER  Roll-rate damper with a slow bounded trim path.
%
%   SIGNATURE CHANGED: t is now an input, needed for the dither. In the
%   MATLAB Function block add a third input port and drive it from a Clock
%   block; keep P as a Parameter-scope symbol. r_e (the external integrator)
%   is still only read in legacy PI mode. Paste this whole file
%   into the MATLAB Function block -- main function AND the local function
%   below it -- exactly as you did for plant.
%
%   P.Controller.Mode
%     0  proportional only        -- no bias rejection; any Cl_0 stands
%     1  proportional + trim      -- DEFAULT
%     2  legacy PI                -- kept so the Monte Carlo can A/B it.
%                                    Produces the 3.3 Hz stick-slip cycle.
%
%   TWO PATHS, DELIBERATELY SEPARATED
%
%   The damper is fast and proportional: Kp = c/q_e, scheduled to hold the
%   open-loop crossover constant. It rejects transients. It cannot reject a
%   constant disturbance -- that is what proportional droop means.
%
%   The trim path is slow, bounded and effectively feed-forward. It exists
%   because of Cl_0: the standing roll moment from fin cant, canard rigging
%   error and body asymmetry, which does not go away and is not small. At
%   Cl_0 = 0.03 -- about 0.21 deg of equivalent fin cant, an ordinary build
%   error -- proportional damping alone sits 110 deg/s off the commanded rate.
%
%   The obvious cure, adding an integrator to the damper, is what produces the
%   limit cycle: the integrator ramps until the command clears the servo dead
%   zone, the surface snaps across, the error reverses, it ramps back.
%
%   What makes a separate trim path the right answer rather than a workaround
%   is that the deflection needed to cancel a standing Cl_0,
%
%       delta_ff = -Cl_0 / Cl_delta
%
%   is INDEPENDENT of q and V -- disturbance and control both scale with q, so
%   one fixed deflection cancels the bias at every flight condition. It is a
%   constant to be learned once, not a signal to be tracked. That is why the
%   trim path can afford to be an order of magnitude slower than the damper,
%   and being that slow is what stops it participating in the loop dynamics.

%#codegen
Kp = P.Controller.c / P.Controller.q_e;

switch P.Controller.Mode
    case 2                                   % legacy PI
        Ki = Kp * P.Controller.Ti;
        delta_cmd = p_e*Kp + r_e*Ki;
    case 1                                   % proportional + slow bounded trim
        d_damp = p_e*Kp;
        delta_cmd = d_damp + roll_trim(p_e, d_damp, Kp, P);
    otherwise                                % proportional only
        delta_cmd = p_e*Kp;
end

% ---- DITHER: the fix for the dead-zone / quantiser lattice limit cycle ----
%
% THE PROBLEM. The servo's dead zone, the 12-bit command quantiser and the
% gear lash together form a lattice. Whether the loop hunts depends on where
% the equilibrium deflection happens to fall on it: measured over the flight
% envelope, most conditions settle dead flat and the low-q corner limit-cycles.
% Dispersion then sprinkles that condition through the Monte Carlo, which is
% why the 95th percentile of ripple_pp is far worse than the median condition.
%
% Quote this in the RATE domain (ripple_pp, rad/s). The roll-ANGLE ripple
% metric that earlier versions of this comment used has been withdrawn: it is
% the detrended integral of rate over the tail and its resolution guard cannot
% separate "has not converged" from "nothing is oscillating".
%
% WHAT IT IS NOT. It is not the trim integrator: freezing the trim leaves the
% cycle intact (p pp 0.483 rad/s with a frozen integrator). It is not
% correctable by dead-zone compensation on the command -- 80% and 100%
% compensation both changed the result by less than 0.4%, because the dead
% zone is inside the SERVO'S OWN position loop, not on the command path, and
% you cannot cancel a loop-internal nonlinearity from outside the loop. And it
% is not fixable by replacing the integrator with an algebraic Cl_0 estimator:
% unity loop gain through a filter leaves exactly half the disturbance
% (measured 0.467 against 0.952, 8.114 against 16.498 -- half at every
% condition). Only integral action gives zero steady-state error.
%
% WHAT IT IS. The integrating element is the SERVO'S OWN rate-to-position
% integration, inside its position loop, driven through its dead zone. That is
% consistent with the ablation above: freezing the trim integrator changes
% nothing because the trim integrator was never the one responsible. Topology
% aside, this is the most-studied loop pathology in process control, where it
% appears as valve stiction, and the established cures there are the same three
% available here: remove the integrator, add dither or knocker pulses, or fix
% the actuator. Only the last two are open to us, because the integrator in
% question is inside a part we did not build.
%
% WHY DITHER WORKS AND COSTS NOTHING. It is injected on the COMMAND, so the
% servo's internal error |cmd - theta| is swept back and forth across the dead
% zone continuously and the threshold is smeared out. The SURFACE does not have
% to follow it: at the flown 125 Hz against f_N = 12.7 Hz (80 rad/s) the
% second-order actuator attenuates by (12.7/125)^2 = 1%, so measured surface
% activity does not rise when dither is added -- the hunting it removes costs
% more motion than the dither adds.
%
% Measured effect: ripple_pp at the 95th percentile falls by an order of
% magnitude between no dither and 1.5x dead zone, flat in activity. The cost
% appears as a small steady rate bias, not as oscillation, and a future
% attitude loop absorbs a rate bias for free. Figures live with the Monte Carlo
% campaign output, not here, so this comment cannot go stale again.
%
% AMPLITUDE IS SET AGAINST AN UNMEASURED DEAD ZONE. P.Servo.Deadband is still
% an assumption, so size the dither against the UPPER end of the plausible
% range, not the nominal. Too much dither costs only bias; too little leaves
% the cycle.
%
% FRAME RATE -- OPEN ITEM. Dither only exists if the servo frame rate can
% represent it. At the flown 125 Hz that needs roughly 600 Hz of frame rate for
% a clean sine; a 333 Hz digital frame gives 2.7 samples per cycle, which is
% above Nyquist but badly reconstructed, and anything at or below 250 Hz aliases
% outright. CONFIRM the BMS-127WV+ maximum frame rate before flying 125 Hz. If
% it caps at 333 Hz, drop the dither to 60-80 Hz, which still sits two decades
% above the roll mode and attenuates almost as hard.
if P.Dither.On
    delta_cmd = delta_cmd + P.Dither.Amp * sin(2*pi*P.Dither.Freq*t);
end

delta_cmd = max(min(delta_cmd, P.Servo.Travel), -P.Servo.Travel);
end


% =========================================================================
function delta_trim = roll_trim(e, delta_damp, Kp, P)
%ROLL_TRIM  Slow bounded estimate of the deflection that cancels Cl_0.
%
%   Four gates, each doing a distinct job:
%
%   1  BANDWIDTH. Ki is sized for ~0.3 Hz at q = 10 kPa, an order below the
%      3.3 Hz stick-slip frequency, so this path is structurally incapable of
%      sustaining that cycle. It uses the highest-q (fastest) case, so it is
%      slower, never faster, elsewhere in the envelope.
%
%   2  DEAD ZONE ON THE INTEGRATOR INPUT -- on the INPUT, not a switch on the
%      integrator. Subtract the band from the error and integrate what is
%      left, so the integration rate falls smoothly to zero as the error
%      approaches it. The band is the roll-rate error one dead zone of
%      deflection produces; the control term dominates the damping term by
%      ~28x so dp/d(delta) ~ 1/Kp, giving
%
%          e_band = (dead zone referred to the canard) / Kp
%
%      which self-schedules with q because Kp does.
%
%      TWO WRONG VERSIONS, both of which I built before this one:
%        * "freeze when the command stops moving" latches shut permanently
%          once the loop settles, and the trim never converges at all.
%        * "freeze when |e| < band" is a dead zone wrapped around an
%          integrator -- the same stick-slip generator, just slower. It gives
%          a sustained 0.44 Hz cycle, 0.77 rad/s peak to peak, that never
%          decays. Subtracting the band instead settles dead flat.
%
%      Do NOT low-pass the error ahead of this. It looks like it should help
%      and does the opposite: the added phase lag creates new hunting points
%      (1.17 rad/s pp at Cl_0 = 0.03, against 0.000 without it).
%
%   3  SATURATION FREEZE. Classic anti-windup, and it covers the low-q end
%      where the damper alone can demand most of the travel. Saturation is
%      genuinely binary, so a hard freeze is correct here.
%
%   4  CLAMP AND RATE LIMIT. Bounds the worst case regardless of the above and
%      stops a trim excursion looking like a control input.
%
%   RESIDUAL. A steady hang-off of roughly +/-5% of the rate command remains.
%   That is the actuator's own resolution floor, not a tuning failure, and the
%   only lever that shrinks it is P.Linkage.n.
%
%   REPLACING THIS WITH THE MEKF LATER. Estimate Cl_0 as a filter state and
%   pass -Cl_0_hat/Cl_delta in as P.Trim.External with P.Trim.Mode = 1. Keep
%   gates 3 and 4: without a canard position encoder an estimator cannot
%   separate a real Cl_0 from dead-zone hang-off, so it will absorb actuator
%   error into an aero coefficient unless it is gated the same way.

persistent trim init
if isempty(init)
    trim = 0;
    init = true;
end

if P.Trim.Mode == 1                          % external estimate (MEKF, later)
    delta_trim = max(min(P.Trim.External, P.Trim.Clamp), -P.Trim.Clamp);
    return
end
if P.Trim.Mode == 0                          % disabled
    delta_trim = trim;
    return
end

dz_canard = P.Servo.Deadband / P.Linkage.n;
e_band    = P.Trim.BandK * dz_canard / max(Kp, eps);

if abs(e) > e_band                           % gate 2
    drive = sign(e)*(abs(e) - e_band);
else
    drive = 0;
end

% OPEN DEFECT -- gate 3 tests the PRE-dither command, but the travel clamp in
% the main function acts on the POST-dither command. Within one dither
% amplitude of the stop the gate can therefore miss a saturation event and the
% trim keeps integrating against a clamped output. Fix:
%     sat_margin = P.Dither.On * P.Dither.Amp;
%     saturated  = abs(delta_damp + trim) + sat_margin >= P.Servo.Travel*0.999;
% NOT applied yet: it changes behaviour at the low-q end, so the dither cost
% figures have to be re-measured after it goes in.
saturated = abs(delta_damp + trim) >= P.Servo.Travel*0.999;   % gate 3

if ~saturated
    step = P.Trim.Ki * drive;
    step = max(min(step, P.Trim.Rate), -P.Trim.Rate);         % gate 4
    trim = trim + step*P.Servo.dt;
    trim = max(min(trim, P.Trim.Clamp), -P.Trim.Clamp);
end

delta_trim = trim;
end
