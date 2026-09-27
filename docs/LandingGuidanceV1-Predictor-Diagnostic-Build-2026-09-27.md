# V1 Beta predictor diagnostic build — 2026-09-27

## Purpose

This is a V1-only diagnostic build. It restores the exact V1 Beta predictor and controller source from `d8ac3dd5` while collecting the evidence needed to determine why a displayed landing prediction can disagree with the visible ballistic surface intersection.

## Runtime scope

- The V2 controller, planner, UI, trace writer, and V2-only tests are removed from the build.
- The Landing Guidance window shows only V1 controls plus:
  - `Log trace data`
  - `V1 Beta Predictor Diagnostics 2026-09-27`
- Logging is disabled by default. It commands no actuator and changes no V1 guidance calculation.

## V1 Beta baseline

The earlier Alpha-labelled diagnostic build retained two later predictor modifications. This build removes them: V1 Beta uses the exact two-sample predictor publication and terrain-profile contact behaviour from the proven Minmus DLL. Diagnostics observe that behaviour only. It also retains the correction for the original trace field-ordering defect; the generated JSONL records use the documented field names.

## Trace output

When `Log trace data` is enabled during a targeted V1 landing, the build writes:

`GameData/MechJeb2/Plugins/PluginData/MechJeb2/LandingGuidanceV1.trace.jsonl`

Two compact JSONL record types are written:

- `guidance_state`: phase, status, UT, warp rate, vessel position and velocity, commanded throttle, thrust acceleration, and the current published prediction identity and endpoint.
- `predictor_result`: every completed normal predictor result and its decision (`initial_pending`, `candidate_accept`, `branch_pending`, or `replace`). It includes the predictor input state and age, simulator endpoint before terrain-profile resolution, resolved terrain endpoint, terrain profile indices and sampled terrain height, trajectory start, and the osculating-orbit impact vector computed from the same input orbit.

The trace reuses existing simulator results and terrain samples. It does not run a second simulation, query terrain beyond the existing final-path profile, or write every trajectory point. State records are rate-limited to the existing V1 trace cadence; predictor records are limited by the existing five-per-second predictor cadence.

## Diagnostic use

For a Mun test, enable `Log trace data`, select a target, start the normal V1 `Land at target` sequence, and retain the trace after deorbit and the first course-correction decision. Compare the same-input `osculatingImpact`, `simulatorEndpoint`, and `resolvedEndpoint` fields before changing V1 control code.
