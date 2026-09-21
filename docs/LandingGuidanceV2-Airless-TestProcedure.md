# Landing Guidance V2 airless auto-warp test procedure

Use this procedure for one objective V2 airless landing test. It uses the
structured trace as the result, rather than a subjective timing observation.

## Before launch

1. Use an airless body. The Mun is the first target case.
2. Put the vessel in a stable, low orbit with enough vacuum delta-V for the
   displayed V2 plan plus its margin. Select a position target on that body.
3. Open **Landing Guidance**. Leave the complete V1 block untouched. In the
   separate V2 panel, enable **V2 terminal auto-warp**, **Enable V2 estimator
   and preflight diagnostics**, and **Write V2 structured trace**.
4. Delete or rename no trace data. The trace is append-only; note the UTC or
   mission time immediately before pressing Start.

## Test

1. Do not use **Land at target** or **Land somewhere**. Those are V1 controls.
2. Press **Start V2 airless landing** once. The only player prerequisite is the
   selected target. V2 either accepts its own fresh plan or rejects with the
   exact preflight reason before it commands the vessel.
3. Let V2 manage warp and the landing. Do not alter throttle, attitude, RCS,
   staging, target, or warp during the run.
4. During the local phase, the V2 panel may show its active target and local
   N/S/E/W controls. Leave them unchanged for this baseline landing.
5. After touchdown, quit KSP before retrieving the trace.

## Evidence to inspect

Read only the newly appended records in:

```
C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\PluginData\MechJeb2\LandingGuidanceV2.trace.jsonl
```

The successful path must contain `v2_phase_transition` events in this order:

```
WarpToStrategic
AlignPlane / PlaneAlignment (only when required)
AlignStrategicBurn
StrategicBurn
AlignTrim / BoundedTrim (only when required)
Coast
BrakingApproach
VisualAssessment
TerminalDescent
Complete
```

For every phase boundary, verify that the record has a fresh
`snapshotVersion`, `v2Phase`, `v2Status`, and `warpRate`. For finite burns,
compare `v2PlannedBurnDeltaV` with `v2DeliveredBurnDeltaV`, and check the
corresponding `burn_start` and `burn_end` events. The local stage must contain
one `visual_rebase` event, a passing `v2SiteAccepted` value, and no accepted
target divert. The final record must be `Complete` with
`isLandedOrSplashed: true` and `flightDataValid: false`; that explicitly marks
post-touchdown estimates as non-flight data.

If V2 rejects, retain the trace and report its final `v2Status` and
`planReason`. A rejection before `burn_start` demonstrates that V2 stopped at
the plan gate and did not command a strategic burn.
