# Roll-tuning: one working folder

Settled 2026-08-24. Read this before touching the roll-control MATLAB/Simulink work.

## The working folder

**`C:\Hyades Rocket Files\Hyades-Flight-Computer\Simulation\Matlab Roll Tuning`**

`cd` here before running anything. It holds the live `Simulink_Roll_Model_1.slx`
and all current `.m` files.

## What went wrong, so it does not happen again

The work was split across two folders and each held half of a working set:

| | `Hyades-Flight-Computer\Simulation` | `Flight Controller V6\Testing` |
|---|---|---|
| `Simulink_Roll_Model_1.slx` | current (wired with `controller.m` + `canard_servo_reduced`) | June stub: legacy PI controller, Aerospace Blockset actuator |
| `.m` scripts | June copies, plus a 936-byte parameters stub | current |

`run_all.m` opened with `restoredefaultpath; addpath(pwd)` from the V6 folder,
which stripped the Hyades path and forced MATLAB to load the **June** model
while running the **current** scripts. The visible symptom was
`check_tunability` reporting that dispersing `Servo.Deadband` and `Servo.Lash`
changed nothing — correct, because the June model contains the Aerospace
Blockset actuator, which has no dead zone or backlash at all. The initial
diagnosis (fast restart not re-evaluating structural block parameters) was
wrong; the parameters simply were not in the model that got loaded.

The lesson is not "check the path". It is that a `restoredefaultpath` +
`addpath(pwd)` opener silently picks the model nearest the current directory,
so a duplicated model is not merely untidy — it changes which system you are
simulating without saying so.

## What was done

- All current `.m` files copied into the Hyades folder.
- Every stale duplicate renamed `*.OLD` in both folders — nothing deleted:
  - Hyades: the June `montecarlo_roll_control.m`, `parameter_sweep_roll_control.m`,
    `optimise_roll_schedule.m` and the parameters stub.
  - V6: every `.m`, plus `Simulink_Roll_Model_1.slx` → `.slx.OLD-20260628`,
    its `.slxc`, its `.autosave` and `slprj\`.
- The live `.slx` was not touched.
- `wire_model.m` was not carried over. It existed to wire the June model and
  the live model is already wired; `check_tunability` covers the question it
  was really answering.

`Flight Controller V6\Testing\Matlab Roll Tuning` is now an archive. Its
siblings (`Canard Sizing`, the SPICE folders) are unaffected and still live.

## Still outstanding in the live model

`Plant/Plant` ends with

    fprintf('Cl_delta', Cl_delta); disp(Cl_delta);
    fprintf('Cl_p', Cl_p);        disp(Cl_p);

inside the derivative function — four printed lines on every time step, about
80,000 per 20 s run at `dt = 1e-3`. This is a significant part of the sim's
slowness and would make `parsim` output unreadable. The `fprintf` calls are
also malformed (format string with no conversion specifier, so the value is
discarded). Strip them and re-save.

*Update 2026-11-07:* the `Plant` block in the saved `Simulink_Roll_Model_1.slx`
no longer contains these `fprintf`/`disp` lines (checked while porting the
plant for HITL-01), so this item is closed.
