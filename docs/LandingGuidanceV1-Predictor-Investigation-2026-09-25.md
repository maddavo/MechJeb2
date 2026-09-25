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
5. Normal continuous endpoint updates require two consecutive compatible predictions. A candidate more than 200 m from the published endpoint requires three compatible predictions before it can replace it.

This removes both the flat/mountain feedback loop and the bisection stall while avoiding a high-volume terrain query across the orbital trajectory.

### Displaced-branch publication repair

The later live Minmus run at approximately 17 km altitude showed a second, distinct publication failure. The terrain-profile solver produced internally consistent **pairs** of contacts, then switched to another pair on a different feature. For example, it published contacts near 263 m ASL, 157 m ASL, 92 m ASL, 46 m ASL, and sea level while the vehicle coasted without a correction burn. Each pair met the ordinary local agreement tolerance, so the old two-sample gate successively replaced the public endpoint. The target marker and reported target error consequently jumped by hundreds of metres.

The replacement gate now distinguishes an ordinary moving endpoint from a change of terrain branch. It keeps the ordinary two-sample cadence for nearby updates. If the candidate is more than 200 m from the currently published endpoint, it must persist for three compatible simulations. An A/A, B/B, C/C sequence cannot therefore move the published endpoint away from A; a real B/B/B trajectory change can. Trace records include the candidate sample count, requirement, and distance from the published endpoint so a future live run can verify this decision directly.

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

## Low-gravity terminal translation repair

The same live Minmus run exposed a terminal-phase defect after the vehicle entered `KillHorizontalVelocity` at about 205 m ASL. It initially reduced a 5.3 m/s descent, but then held exactly 5% throttle for approximately 19 seconds while horizontal speed remained 3–5 m/s. The trace measured 1.06 m/s² of upward thrust acceleration at that throttle, more than Minmus gravity. The vessel climbed from 200 m to 292 m and required manual takeover.

The cause was the general MechJeb minimum-throttle limiter. `KillHorizontalVelocity` correctly calculated that it needed roughly 2–3% throttle to hold height, but requested it through the generic API, which raised every nonzero request to the Landing Guidance window's 5% setting. This terminal translation step now bypasses that generic lower floor and commands its closed-loop physical throttle directly, including a value below 5% or zero. It does not change the setting, its UI, or any other landing phase. Focused tests cover low-gravity hover, descent arrest, and no-vertical-thrust handling.

## Regression criteria

- V1 UI first 176 source lines remain identical to its protected baseline.
- V1 controller phase, attitude, throttle, RCS, staging, and warp source remain protected by `tools/Verify-V1Baseline.ps1`.
- Focused tests cover first real ridge contact, ignoring later terrain, invalid profile samples, normal two-sample publication, and rejection of alternating displaced terrain pairs.
- Focused tests cover the Minmus terminal translation case where the physical hover request is below the general 5% throttle floor.
- Release build and V2/Hoverslam regression suite pass.
- A new Minmus trace must show stable accepted prediction versions through Course Correction. It must not contain the former `terrain_bracket_bisect` loop.
