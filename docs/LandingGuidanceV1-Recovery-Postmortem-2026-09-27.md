# V1 landing guidance recovery postmortem — 2026-09-27

## Decision

This iteration restores the V1 Alpha deorbit controller byte-for-byte. V1 Alpha is the source identified by Dave as the most accurate completed landing. The restoration deliberately retains only two later **predictor-information** repairs:

1. terrain-profile local-index mapping (`92c36f04`); and
2. persistent displaced-terrain-branch publication (`c2165a54` / `353b0535`).

No candidate planner, deorbit endpoint gate, retained inertial aim point, new phase, new warp rule, or new throttle law remains in V1. V1 UI and all V2 source are outside this restoration.

## Evidence from the 27 September Mun test

The installed `ac6cabdd` build completed Plane Change and entered V1 DeorbitBurn. Its trace selected a nominal 681.3 m endpoint candidate and began a roughly 30.9 m/s burn. That candidate was not a valid control target:

- before engine thrust, the retained residual rose from 30.9 to 34.2 m/s as the frame evolved;
- after the burn began, the vessel developed positive vertical speed;
- apoapsis rose from about 25.2 km to 75.5 km while the controller continued to command approximately 0.76 throttle; and
- the remaining burn estimate rose from 26.6 m/s to 46.7 m/s.

The screenshots independently show the resulting radial-out trajectory. This is a control regression introduced by the candidate planner, not a predictor-only issue.

## What was good in V1 Alpha

V1 Alpha completed the established sequence: Plane Change, Deorbit Burn, Course Correction, Coast, Braking, and Terminal guidance. Its deorbit calculation continually recomputed the geometric target direction and remaining burn from the current vessel state. The controller therefore retained its original burn-completion behaviour and did not lock an inertial velocity target into a changing local horizontal frame.

The Alpha controller was not claimed to be perfect. The later test history identified predictor branch changes, local terrain-profile indexing, occasional small/far-apart correction pulses, and terminal low-gravity concerns. Those are separate from the deorbit phase and must be evidenced separately before changing a controller phase.

## Why later deorbit techniques failed

| Technique | Intended benefit | Live outcome | Why it was unsuitable |
|---|---|---|---|
| Target-normal or phase gate | Prevent early deorbit | First version blocked ignition; later phase gate still left large misses. | A diagnostic angle was treated as an authority gate. It changed phase-entry behaviour without proving the original calculation wrong. |
| Endpoint candidate search | Validate the actual redirected burn | Initially rejected all candidates with 382–472 km false error. | The evaluator mixed body-centred and world-position coordinates by subtracting the body origin twice. |
| Corrected endpoint candidate search | Permit valid deorbit | Selected a candidate but held its initial 1,154 m/s burn indefinitely. | Candidate output was used as a permanent control command instead of a pre-ignition check. |
| Retained post-burn velocity residual | Make the burn finish | Produced a radial-out burn that raised apoapsis. | The retained velocity was built in one instantaneous local horizontal frame, then controlled as an inertial target while the local frame changed. It altered the vector control law across the deorbit sequence. |

The common error was introducing a replacement deorbit planner inside a proven controller, then using live KSP tests to discover its control implications. That process stops here.

## Predictor repairs retained

The terrain improvements are retained because they address measured predictor-data faults without changing V1 phase ownership or control authority:

- **Terrain profile contact:** simulate an airless trajectory to sea level, then determine first real terrain contact from the recorded descending path. Terrain height is not fed back into the next vacuum trajectory.
- **Local terrain index:** use the profile-local index for terrain heights and the translated full-path index only for trajectory lookup.
- **Branch publication:** require persistent agreement before replacing a materially displaced terrain endpoint, preventing alternating ridge/lowland branches from driving V1 corrections.

## Required proof before another V1 control change

1. Capture the exact Alpha start state, selected target, initial deorbit vector, burn magnitude, phase transitions, and first prediction endpoint.
2. Replay the original Alpha computation against that captured state and assert the same direction, decreasing burn residual, and phase transition.
3. Add one focused regression case for the measured defect. The test must show improvement while the Alpha replay remains unchanged.
4. Check Plane Change, Deorbit, Course Correction, Coast, Braking, and Terminal contracts before any DLL is built.
5. Install only a build that passes the replay and focused regression. KSP then validates integration rather than serving as first discovery of a changed control law.

## Current iteration acceptance

- `DeorbitBurn.cs` matches V1 Alpha exactly.
- Predictor source differs only by the two retained terrain-information repairs.
- No V1 UI or V2 source changed.
- This restoration is a controlled reference build, not an assertion that every previously observed minor V1 issue is fixed.
