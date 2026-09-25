# Landing Guidance V1 predictor investigation — 2026-09-25

## Scope

This work is V1-only. V2 controller development remains paused. The V1 landing-window layout, controls, and landing-phase controller remain unchanged.

## Runtime evidence

The Minmus V1 trace recorded incompatible terrain branches during one coast: a lowland endpoint near 0 m ASL and a mountain endpoint near 1.8 km ASL. Course correction received opposing kilometre-scale downrange errors.

The subsequent live test of the initial repair recorded an additional defect: the terrain-height bisection held V1 in Course Correction at zero throttle. The trace showed the simulated contact height stuck at 2.3 m while the terrain at the resulting endpoint remained 311–353 m. It never produced a new prediction version.

## Root cause

`ReentrySimulation` historically represents terrain with one spherical contact radius. Feeding an endpoint's terrain height into the next simulation makes terrain height alter the endpoint, then makes that endpoint alter terrain height again. The live trace established that the terrain map along this path is not a valid one-dimensional root function: a ridge and a lowland can create different endpoints and invalidate scalar bisection.

## Correction

For airless V1 predictions:

1. The simulator always runs to the body sea-level sphere. Its vacuum trajectory is independent of terrain until the first surface crossing.
2. V1 examines the already recorded descending trajectory on the KSP flight thread, using actual `TerrainAltitude` data only in the final portion below the body’s maximum terrain height.
3. It selects the first path sample that reaches the real terrain surface and publishes that point as the prediction endpoint.
4. The next airless simulation again starts at sea level. No endpoint terrain height is fed back into the simulator.
5. The existing two-consecutive-prediction consensus gate remains in place, so a transient endpoint cannot drive a correction.

This removes both the flat/mountain feedback loop and the bisection stall while avoiding a high-volume terrain query across the orbital trajectory.

## Live validation

On 2026-09-25 the installed terrain-profile build completed a Minmus V1 landing at the selected target. The live run showed a temporary Course Correction consensus wait at zero throttle, then resumed and completed the landing. That outcome means the wait is a bounded predictor confirmation state, not a safe basis for loosening the consensus thresholds. The proposed local-slope threshold relaxation was therefore not installed and was reverted from source.

## Adaptive course-correction pulse policy

The successful landing trace showed that V1 calculates varying correction vectors (about 22 m/s down to 1.3 m/s in the sampled run). A correction vector is the local solver's answer for moving the predicted surface endpoint to the requested target.

Each V1 course-correction pulse takes a bounded fraction of that predicted **surface effect**, then waits for two settled predictions before commanding another pulse:

- remote target error: begins at half of the calculated endpoint movement;
- middle range: one quarter;
- near target: one tenth;
- close target or a direction reversal: one twentieth.

After each remote pulse, V1 compares the measured endpoint improvement with the predicted improvement. Two consecutive responses within 25 percent of the prediction raise the remote effect fraction by one tenth, up to four fifths. A weak or adverse response halves the fraction down to one quarter. There is no correction-pulse delta-V cap. The pulse delta-V is whatever produces the selected fraction of the currently predicted landing-position movement, so it adapts to the body, trajectory, speed, gravity, and vehicle response. This is limited to V1 Course Correction. It does not alter V2, the V1 window layout, deorbit, braking, descent, attitude, RCS, staging, or warp logic. Focused tests cover gain increase, cap, reduction, effect bands, and reversal behavior.

## Regression criteria

- V1 UI first 176 source lines remain identical to its protected baseline.
- V1 controller phase, attitude, throttle, RCS, staging, and warp source remain protected by `tools/Verify-V1Baseline.ps1`.
- Focused tests cover first real ridge contact, ignoring later terrain, and invalid profile samples.
- Release build and V2/Hoverslam regression suite pass.
- A new Minmus trace must show stable accepted prediction versions through Course Correction. It must not contain the former `terrain_bracket_bisect` loop.
