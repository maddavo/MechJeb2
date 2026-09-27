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

## Build and installation record

- Source commit: `057f5f3c` (`fix: make V1 powered prediction target-aware`).
- Build identity shown in the Landing Guidance window: `V1 Beta Predictor
  Diagnostics 2026-09-27 r6`.
- Build: `dotnet build MechJeb2.sln -c Release --no-restore` succeeded.
- Focused landing predictor tests: 16 passed, 0 failed.
- The full repository test run completed with 8,405 passing and three existing,
  unrelated failures: `StaticTests.ToSITest` expects the old Infinity glyph and
  two PSG Kerbin ascent numeric baselines differ from their stored values.
- Installed after confirming `KSP_x64` was not running. The prior DLL is backed
  up as `C:\Users\Dave\Documents\KSP Backups\MechJeb2-LandingGuidanceV2\MechJeb2-20260927-170915-pre-r6.dll`.
- Installed DLL SHA-256: `900AAA3509CEE689DF4C1489FD13D46D5494238C26712903E0993DE76A4F8F97`.

## r7 trace-backed correction

The r6 Mun trace showed that no target-aware result was published: every
record had `inputForcedBrakingStartUT: null`. The r6 terminal-speed gate was
wrong because the airless simulator reaches sea level after V1's virtual
braking envelope ends 200 m above the landing site; it rejected all candidate
results before selection.

The same trace exposed the circular terrain input. The selected target was
492 m ASL, but active predictor records used a 3.9–4.3 km `inputDecelerationEndASL`
from the short predicted endpoint. r7 gives the **predictor only** the selected
target's terrain height, so its 200 m braking envelope is anchored at the
landing target. V1's controller still uses its original control altitude and
phases.

r7 accepts a candidate terminal speed up to the gravity-only speed acquired
through V1's existing 200 m final-descent buffer, plus a 5 m/s numerical
allowance. It logs an explicit `predictor target-aware rejected all candidates`
event if no candidate qualifies.

## r7 build and installation

- Source commit: `01066d16` (`fix: anchor V1 predictor braking at target terrain`).
- Window build identity: `V1 Beta Predictor Diagnostics 2026-09-27 r7`.
- Build succeeded and focused landing-prediction tests passed: 18/18.
- Installed after confirming `KSP_x64` was not running.
- Backup: `C:\Users\Dave\Documents\KSP Backups\MechJeb2-LandingGuidanceV2\MechJeb2-20260927-172040-pre-r7.dll`.
- Installed DLL SHA-256: `6B1A5641A0FE4A3E5F2DDC3D6C4895C9EC2788917EA9CAEAEF8757EEC05E56A9`.

## r8 correction: publish target-aware candidates

The r7 KSP log proved that the target-aware planner ran but rejected all five
candidates. Its terminal-speed test was inappropriate: `ReentrySimulation`
ends virtual braking at the V1 final-descent handoff, while V1's existing
final-descent controller owns the remaining vertical speed reduction.

r8 selects from every finite `LANDED` candidate by powered endpoint distance to
the selected target. Terminal surface speed remains in each candidate trace as
diagnostic data; it is no longer a hard rejection gate. With trace logging
enabled, each planning batch now writes five `target_aware_candidate` JSONL
records containing the forced braking start, endpoint error, virtual Δv, and
terminal speed before the selected plan is published.
