# Development continuity

## Duna M1 (2026-11-25)

Pinned development baseline: `907349e78b3212cf67b30ef4eaea37dd1412e829`. Historical installed trace source: `2db649e697937f3a4ecbdc4dca6ca4f609ec6d0f` only. Diagnostic-only changes; see [Duna-M1](docs/Duna-M1.md) for design, schema, tests, limitations and next work. Build blocked in cloud by missing .NET SDK and KSP/Unity managed references. No flight validation. Next: local Release build, focused tests, matched no-chute Duna full-run captures, airless regression. Do not change predictor/control output or repair chutes until separate authorization.

## Duna braking handoff (2026-11-26)

Parent `718247a9c8e0ddbd5a555267774b9b0900519bc2`: M1 Actions Release build succeeded; focused 4/4 tests, broader 37 pass/54 fail identically to pinned V1 reference-assembly baseline. Installed M1 flight impacted after delayed handoff. Pilot reported full braking throttle, so NaN not established as cause. See [implementation and validation](docs/Duna-Braking-Handoff.md). New isolated test branch; guard uses no-drag-credit rotating 3D projection, existing airless timing helper, no forecast dependency. First test isolates braking commitment, not convergence or general NaN repair. Test build pending CI and matched live flight; do not infer safety from projection. Next: inspect actual earlier thrust, clearance and landing outcome, then revise model or terminal handoff based on evidence. No installation or release by this chat.
