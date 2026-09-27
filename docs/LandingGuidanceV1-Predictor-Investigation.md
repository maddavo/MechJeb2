# V1 predictor repair: target-aware powered endpoint

## Purpose

This document records the narrow V1 predictor repair made after the V1 Beta
Mun investigations. Update it with every V1 commit that changes predictor
inputs, numerical integration, terrain contact, or the meaning of the
published landing endpoint.

V1 Beta remains the controller baseline. This repair does not change its
autopilot phases, throttle law, attitude law, RCS behaviour, staging, warp,
or window layout.

## Evidence and root cause

The user observed a shallow Mun trajectory continuing long of the target, yet
the active V1 marker showed a powered landing short of it and placed virtual
braking near the vessel. Aborting autoland returned a ballistic marker near
the intended path.

Those observations are consistent with two predictor paths already present in
V1 Beta:

| Mode | Predictor behaviour |
|---|---|
| Autoland inactive | Propagates the current ballistic orbit to surface contact. |
| Autoland active | Uses `SafeDescentSpeedPolicy`, chooses a virtual braking start from total surface speed, then simulates powered descent. |

`SafeDescentSpeedPolicy` uses a vertical stopping-speed envelope. It compares
the vessel's total surface speed with the speed that could be stopped over the
remaining radial altitude. In a shallow orbital descent that can declare a
braking start much too early. The target is not an input to that choice, so
the result can be safe in the scalar speed sense yet geometrically short of
the selected site.

The inspection also found a separate simulator defect: after an analytic coast
to virtual braking, the adaptive integration reference point could be taken
from a pooled simulation's old start state. The repaired simulator records the
actual coast-complete state before it begins numerical stepping.

## Repair

For an active, targeted, airless V1 landing, the predictor now performs the
following bounded planning pass:

1. Run the existing V1 simulation normally to determine the viable part of
the current orbital pass.
2. Evaluate five evenly spaced virtual braking-start times from the original
speed-policy start through the projected surface contact.
3. Run each candidate using the existing `ReentrySimulation`, the actual
vessel snapshot, thrust acceleration, descent policy, and body rotation.
They run on a worker thread.
4. Reject candidates that do not land or whose simulated terminal surface
speed exceeds 15 m/s.
5. Select the safe candidate whose powered endpoint has the smallest surface
distance to the selected target. Resolve real terrain contact only for the
selected candidate on the flight thread.
6. Publish that selected prediction for five seconds. During that interval,
scalar-policy refreshes remain diagnostic inputs and cannot overwrite the
target-aware result. The plan then refreshes from the current vessel state.

The selected result exposes its simulated braking start in the same trajectory
field V1 Beta already reads for orientation, coast, auto-warp, and its existing
deceleration burn. The repair therefore improves the predictor data supplied
to the V1 controller rather than replacing controller behaviour.

## Performance and safety bounds

- Candidate count is fixed at five and planning refreshes no more than once
every five seconds.
- Candidate integration is worker-thread work. The only added flight-thread
terrain query is for the selected result.
- A target change advances the predictor generation. Late plan results from a
previous target are discarded.
- If no safe candidate exists, the normal V1 prediction remains in place;
the code does not invent a burn time.

## Validation required

Offline validation covers candidate selection, unsafe-candidate rejection,
and no-candidate handling. Build validation covers the full solution. The
broader repository test suite currently has three pre-existing environment or
baseline failures (`StaticTests.ToSITest` formatting and two PSG ascent numeric
baselines); focused landing-prediction tests pass.

In KSP, review `LandingGuidanceV1.trace.jsonl` after a Mun test with **Log trace
data** enabled. The trace should show `predictor target-aware` records with a
braking start and target error. Compare the selected powered endpoint with the
actual V1 phase and verify that its predicted braking point is later than the
incorrect immediate virtual brake seen in the failing trace.
