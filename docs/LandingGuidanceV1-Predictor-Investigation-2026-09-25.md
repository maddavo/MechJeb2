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
6. For those airless profile endpoints only, the gate accepts a bounded local contact movement: 300 m in surface position, 20 seconds in arrival, and 250 m in local terrain ASL. Atmospheric predictions retain the strict pre-existing thresholds. A kilometre-scale terrain branch still fails the spatial gate.

This removes both the flat/mountain feedback loop and the bisection stall while avoiding a high-volume terrain query across the orbital trajectory.

## Regression criteria

- V1 UI first 176 source lines remain identical to its protected baseline.
- V1 controller phase, attitude, throttle, RCS, staging, and warp source remain protected by `tools/Verify-V1Baseline.ps1`.
- Focused tests cover first real ridge contact, ignoring later terrain, invalid profile samples, stable local-slope consensus, and rejection of a kilometre-scale branch.
- Release build and V2/Hoverslam regression suite pass.
- A new Minmus trace must show stable accepted prediction versions through Course Correction. It must not contain the former `terrain_bracket_bisect` loop.
