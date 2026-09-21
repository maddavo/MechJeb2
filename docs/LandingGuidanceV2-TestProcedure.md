# Landing Guidance V2 validation procedure

Use this procedure to produce objective V2 validation data.  V1 controls and
V1 behaviour are not part of this test.

## Common setup

1. Select a position target on the current body.
2. Open **Landing Guidance**. Leave the contiguous V1 controls unchanged.
3. In the separate **Landing Guidance V2** section, enable **V2 auto-warp**,
   **Enable V2 estimator and preflight diagnostics**, and **Write V2 structured
   trace**.
4. Note mission time before pressing **Start V2 landing**. This button has one
   player prerequisite: the selected target. It may hold in V2 preflight while
   it creates and validates a plan, but must not command the vessel in that
   phase.
5. Do not use **Land at target** or **Land somewhere**, and do not alter
   throttle, attitude, RCS, staging, target, or warp during the run.

The append-only trace is:

```
C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\PluginData\MechJeb2\LandingGuidanceV2.trace.jsonl
```

## Airless auto-warp case

Run this first on the Mun from a stable low orbit with the displayed V2 plan
margin positive.  A successful trace contains `v2_phase_transition` records
through `Preflight`, `WarpToStrategic`, `AlignStrategicBurn`, `StrategicBurn`,
`Coast`, `BrakingApproach`, `VisualAssessment`, `TerminalDivert` or
`VelocityNull`, and `Complete`.  `PlaneAlignment` and `BoundedTrim` occur only
when required.

Check every `warp_exit` against a later snapshot used by the next command
phase.  For each finite burn, compare `v2PlannedBurnDeltaV` with
`v2DeliveredBurnDeltaV` and retain its `burn_start` and `burn_end` records.
The terminal records must show `v2SiteAccepted: true`; the final landed record
must show `isLandedOrSplashed: true` and `flightDataValid: false`.

## Atmospheric case

Run separate Kerbin and Duna cases.  Before V2 asks for attitude or throttle,
the trace must show a `WaitingForEstimate` atmospheric plan and then a
burn-correlated candidate result.  Match these fields:

- `atmosphericCandidateSnapshotVersion`
- `atmosphericCandidatePlanSnapshotVersion`
- `atmosphericCandidateOutcome`
- `atmosphericStrategicEntryDeltaV`
- `atmosphericStrategicEntryBurnUT`

If an entry burn is needed, the sequence includes
`WarpToAtmosphericEntry`, `AlignAtmosphericEntryBurn`, and
`AtmosphericEntryBurn`.  On the warp exit V2 returns to `Preflight` and creates
a fresh candidate; no prior candidate may authorize the burn.  A trajectory
already entering the atmosphere has a zero strategic-entry delta-V and moves
from the validated candidate to `AtmosphericEntry` without a second burn.

## Failure evidence

If V2 rejects, retain the trace and record the final `v2Status`, `planReason`,
`atmosphericPlanReason`, plan snapshot versions, and any `burn_*` records.
A rejection before a `burn_start` demonstrates that V2 stopped at a plan gate.
