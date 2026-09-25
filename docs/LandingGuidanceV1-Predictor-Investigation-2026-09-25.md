# Landing Guidance V1 predictor investigation — 2026-09-25

## Scope

This work is V1-only. V2 controller development is paused. The V1 landing-window layout and its normal landing sequence remain unchanged.

## Runtime evidence

The Minmus V1 trace recorded two incompatible endpoint branches while the vessel coasted toward the same selected target:

| branch | latitude | longitude | terrain ASL | target error |
| --- | ---: | ---: | ---: | ---: |
| flat terrain | 3.072 degrees | -39.55 degrees | 0 m | about 1.24 km |
| mountain terrain | 3.154 degrees | -34.03 degrees | about 1,790 m | about 4.93 km |

The course-correction solver then received opposite downrange errors, approximately +1.25 km and -4.52 km. It was therefore acting on alternating physical assumptions, rather than one predicted trajectory.

## Root cause

`ReentrySimulation` represents terrain as one spherical contact radius. Before this correction, the next airless simulation used the terrain height from the previously *published* endpoint. A flat endpoint supplied a zero-height contact sphere. A later simulation could then cross that sphere over a mountain, and its terrain height moved the next contact sphere again. This is a feedback loop, not a reliable prediction.

The first fixed-point guard correctly stopped the unsafe alternating control output, but the live 2026-09-25 test established that the terrain-height map can form a two-cycle: 1,382 m supplied to the simulator produced 2,162 m at the queried endpoint, and 2,162 m produced 1,382 m. The correct numerical response is bracketed root solving, not repeated substitution. V1 now bisects those two heights and continues until simulated and queried endpoint terrain agree.

## Correction and regression criteria

1. An airless simulation is published only when its input terrain height agrees with the terrain queried at its own endpoint within 2 m.
2. A terrain mismatch starts another immutable simulation using a bounded terrain-height root solver. When observations bound the physical contact height, the next immutable run uses their midpoint; it does not alternate between the two endpoints.
3. Two consecutive self-consistent simulations must agree before the prediction version advances. An A/B/A branch sequence cannot publish either alternating branch.
4. Starting a new V1 targeted landing clears prior target terrain iteration and published endpoint before any new correction can use it.
5. Atmospheric prediction handling retains its existing behaviour.
6. The V1 landing buttons, phase sequence, throttle, attitude, RCS, staging, and UI layout are not changed by this predictor correction.

## Validation required before installation

- Unit coverage for terrain fixed-point convergence and branch alternation.
- Release build and focused V1/V2 shared regression suite.
- Verify the V1 UI guard and V2 source isolation guard.
- Inspect a new V1 Minmus trace for `terrain_iterate` events followed by one stable sequence of `candidate_accept` events, with no alternating endpoint branches reaching `PredictionVersion`.
