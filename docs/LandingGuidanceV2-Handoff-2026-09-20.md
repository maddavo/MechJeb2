# Landing Guidance V2: implementation handoff — 2026-09-20

## Read this first

This document is the operational handoff for the next developer or chat.  It
records what is *actually implemented and exercised*, separately from the
approved target architecture in [LandingGuidanceV2.md](LandingGuidanceV2.md).
Do not infer that a design-section feature exists merely because it appears in
that architecture document.

The immediate rule is:

> Preserve the restored, working V1 controller.  V2 is passive and opt-in until
> it has a complete plan, phase manager, command authority, and validation
> evidence.

## Repository and deployment topology

| Item | Location / revision | Meaning |
| --- | --- | --- |
| V2 design repository | `C:\Users\Dave\Documents\GitHub\MechJeb2-landing-guidance-v2` | This repository; branch `design/landing-guidance-v2`. |
| V2 foundation commit | `a032fc97` | Passive estimator, preflight UI, and optional structured trace. |
| V1 restoration build repository | `C:\Users\Dave\Documents\GitHub\MechJeb2-solver-basis-v1-build` | Separate reconstruction worktree used for the installed DLL. |
| V1 restoration commit | `c47337ff` | Restored the known-good Solver Basis V1 behaviour and placed the passive V2 UI below V1 controls. |
| Installed DLL | `C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\MechJeb2.dll` | Hash after installation: `034921CF46F90494EDF25570D1EAAF6BC79C26FE39CE20FEDE13BEADCCA13F54`. |
| Proven Solver Basis reference DLL | `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Predictor Closed-Loop Deorbit\20260920-005719\MechJeb2.dll` | SHA-256 `7320BB280ECF082AEA3DD3FF8C22FAC836A5BE930B57B08F27C7D0166936D049`. |
| Installation rollback | `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Solver Basis V1 Restored plus Passive V2\20260920-163026\MechJeb2.dll` | DLL immediately preceding the restored build. |

The V2 repository currently contains the passive foundation.  The V1 build
repository contains the installed integration.  Before changing V2, reconcile
or cherry-pick intentionally; do **not** treat either worktree as disposable,
and do not overwrite the main dirty `MechJeb2` checkout.

## Verified V1 baseline that V2 must not alter

The user rejected a succession of experimental V1 changes that produced stalled
or poor landings.  The selected reference was the Solver Basis DLL above.  It
completed a successful landing during the original test sequence and, after
reconstruction, completed another user-confirmed landing on 2026-09-20.

The previous reconstruction error that caused a visible stall was specific and
important:

```
PlaneChange completed
  WRONG reconstructed path: LowDeorbitBurn -> could wait/stall
  Proven Solver Basis path: DeorbitBurn -> CourseCorrection -> Coast/Descent
```

`c47337ff` restores the proven functional path and its associated geometric
deorbit, coast/RCS, correction-pulse, and predictor behaviour.  It also retains
the pre-existing V1 landing trace.  Do not refactor, retune, or move V1 UI while
developing V2 unless a separately approved V1 change is requested.

The latest accepted runtime test did the following:

1. Plane change completed.
2. Geometric deorbit burn completed.
3. V1 entered course correction for the familiar deorbit overshoot.
4. Vessel landed successfully.

This proves the restored V1 path works.  It does **not** prove that V1's
strategic deorbit overshoot is solved.  That residual is an input to V2 design,
not licence to silently change V1.

## What V2 currently implements

The current code is deliberately passive:

- `MechJeb2/MechJebModuleLandingGuidanceV2.cs`
  - Captures an immutable per-refresh snapshot of orbital position/velocity,
    mass, available vacuum delta-V, maximum acceleration, body, and target.
  - Runs `AirlessImpactEstimator.Estimate(snapshot)`.
  - Calculates only an impact-velocity cancellation lower bound.  It is **not**
    a landing feasibility calculation.
  - Performs no throttle, attitude, RCS, staging, target, or warp command.
- `MechJeb2/LandingGuidanceV2/AirlessImpactEstimator.cs`
  - Independent airless, rotating-body impact estimator for the preview.
- `MechJeb2/LandingGuidanceV2/LandingGuidanceV2Models.cs`
  - V2 snapshot, estimate, and preflight data models.
- `MechJeb2/MechJebModuleLandingGuidance.cs`
  - Renders V2 **after** all existing V1 controls, at the bottom of the landing
    window.  The V1 layout and operation must remain unchanged.

### Current V2 controls and exact behaviour

| Control | Required state | What it does |
| --- | --- | --- |
| `Enable V2 estimator and preflight diagnostics` | Target exists and craft is in Flight scene | Refreshes the passive V2 snapshot/preflight automatically every 0.5 seconds of vessel UT. |
| `Write V2 structured trace` | Preview enabled | Appends each V2 preflight record to the JSONL trace described below. |
| `Refresh V2 preflight` | Flight scene, valid target, vessel, and body | Forces one immediate passive V2 snapshot. It is not required while automatic preview refresh is active. |

The V2 controls do not require V1 landing guidance to be active, but a valid
position target is required.  They must never alter vessel commands.

## Structured trace: actual output and limitations

When both V2 toggles are enabled, V2 writes to this file, **not** `KSP.log`:

```
C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\PluginData\MechJeb2\LandingGuidanceV2.trace.jsonl
```

Each line is a JSON object. The correlated records include:

- `snapshotVersion`, `ut`, `body`, `targetLat`, `targetLon`
- inertial `position` and `velocity`
- `mass`, `availableDeltaV`, `maxAcceleration`
- estimator `outcome`, `impactUT`, `targetError`
- `brakingDeltaVLowerBound`, `deltaVAboveLowerBound`
- V1 `v1Phase`, `v1Status`, `v1PredictionVersion`, `v1PredictionOutcome`,
  predicted endpoint latitude/longitude/UT, and `v1TargetError`
- `warpRate`, `attitudeErrorDegrees`, commanded and flight-control throttle,
  actual thrust-acceleration magnitude and forward component, MechJeb and
  action-group RCS state, and the current RCS control command
- `isLandedOrSplashed`, `estimatorApplicable`, `flightDataValid`, and
  `validityReason`
- `estimatorRepeatOutcome`, `estimatorRepeatImpactUT`,
  `estimatorDeterministic`, and `estimatorValidationDetail`; these describe a
  second independent evaluation of the same immutable V2 snapshot
- `preflightState`, `preflightLocalGravity`, `preflightReason`, and the fixed
  `v2CommandAuthorized: false`; the current preflight screen rejects only hard
  lower-bound failures and labels all other airless cases `NeedsCompletePlan`

Regular rows use `recordType: "sample"`. Transition rows use
`recordType: "event"` and identify `phase_transition`, `burn_start`,
`burn_end`, `warp_enter`, or `warp_exit`, with `eventFrom`/`eventTo` and the
same correlated state fields. Transition detection is sampled at the V2
refresh cadence (0.5 seconds of vessel UT); it is not a higher-frequency event
hook. `v1PredictionVersion` is a trace-local counter that advances when the
published V1 prediction result object changes; it is not a V1-native version.
The V2 `snapshotVersion` identifies the exact V2 snapshot carried by each
sample and transition record. V2 remains passive and records no V2 commands.
Landed or splashed snapshots now return the explicit `NotFlight` estimator
outcome instead of a ballistic impact estimate.

The current preflight screen is not a landing-feasibility verdict. It has no
strategic deorbit, trim, terminal-divert, terrain, contingency, or protected
reserve plan, so it cannot authorize any V2 command.

### Evidence from the 2026-09-20 landing

The file exists and contains 1,579 V2 records from the successful test.  The
trace therefore verifies that the preview and structured-trace toggles were on
and the passive V2 estimator executed.

However, this is **not yet sufficient diagnostic data to design a control law**:

- It has no V1 phase/step, commanded throttle, delivered delta-V, warp state,
  attitude error, RCS command, or V1 predictor result on the same record.
- It does not mark launch, deorbit-burn start/end, course-correction start/end,
  braking start, or touchdown.
- It continues after landing.  The airless ballistic estimator's post-touchdown
  `Impact` and large target-error values are not meaningful and must be filtered
  rather than interpreted as a flight failure.
- `brakingDeltaVLowerBound` is an inertial impact-speed lower bound; it excludes
  gravity loss, engine response, attitude settling, terrain, terminal divert,
  and reserve.  Negative `deltaVAboveLowerBound` is not by itself a proof that
  the landing is impossible.

The established V1 trace remains in `KSP.log` under `[MechJebLandingTrace]`.
It records phase/status changes, accepted prediction data, and rate-limited
state.  Use UT to correlate it with V2 JSONL records.  Never confuse a V2 JSONL
absence with `KSP.log` absence: they are intentionally separate outputs.

## Required next diagnostic increment (before any V2 control authority)

Extend the structured V2 record, or write an explicitly correlated companion
record, with these fields:

1. `v1Step`, `v1Status`, `v1PredictionVersion`, V1 predicted endpoint, and V1
   target error.
2. Autopilot command/plant state: warp rate, attitude error, commanded throttle,
   actual thrust acceleration, RCS enabled/command, mass, and current delta-V.
3. Explicit event records: preflight accepted/rejected; plan creation;
   auto-warp enter/exit; burn start/end; planned versus delivered delta-V;
   phase transitions; correction accepted/rejected; target rebase/divert; and
   touchdown.
4. Snapshot lineage: plan/snapshot ID and the snapshot ID used by every estimate
   and command.  This is necessary to enforce the no-stale-command invariant.
5. Validity context: landed/splashed state and estimator applicability, so
   post-landing records are clearly classified rather than treated as impacts.

This is a logging-only step.  It must not modify V1 commands or turn on any V2
authority.  It supplies the exact comparison data needed to assess the V1
deorbit overshoot and validate a future V2 planner.

## V2 design decisions already approved

The full rationale is in `LandingGuidanceV2.md`; the following are non-negotiable
constraints for subsequent work:

- V2 is a separate, opt-in, plan-led controller, not a predictor-output filter
  or a V1 retune.
- Predictor output is an estimate, never an immediate steering command.
- Each estimate derives from one immutable vessel/body snapshot; no estimate may
  become input to a later estimate.
- V2 must work on airless and atmospheric bodies and on large/small vessels,
  with or without RCS.  Any acceleration/authority limit is body- and
  vessel-aware, not a fixed Minmus/Mun tuning constant.
- A strategic deorbit is a finite, counted feed-forward burn.  It must stop in a
  validated long-side corridor and must not deliberately cross to the short side
  of the target.  The target-crossing overshoot was the user’s primary V1
  complaint.
- Corrections are bounded trim actions.  They must preserve terminal-divert
  reserve and cannot simply chase a fluctuating predicted marker.
- Preflight must calculate usable delta-V, plan cost, trim budget, terminal
  reserve, contingency, and margin.  If negative, reject before commanding a
  burn.
- Auto-warp is managed at phase boundaries.  It is permitted during safe
  airless coasts, but not during burns, meaningful atmosphere/drag, visual
  assessment, terminal divert, or touchdown.  Every warp exit invalidates stale
  estimates and requires revalidation.
- Red remains the active desired target and blue remains predicted touchdown.
  NSEW always moves red.  At a local visual-assessment gate, red may be rebased
  once to blue; it must never continuously chase blue.
- Local site assessment is only meaningful within several hundred metres.  A
  divert must be feasibility/reserve checked and never silently change target.

## Research already incorporated

The architecture deliberately draws from real guidance patterns, not from a
claim that KSP is physically identical to Apollo or Falcon:

- Apollo descent: separate braking, approach, and landing phases; crew visual
  target assessment and retargeting near the site.
- SpaceX booster recovery: a planned sequence of major burns and terminal
  guidance, rather than continuous long-range target chasing.
- Model-predictive/trajectory-tracking principle: planning, estimator, phase
  authority, and actuators are separate responsibilities.

Primary references and links are retained in `LandingGuidanceV2.md`:

- NASA Apollo 11 Mission Report.
- NASA Apollo lunar descent/ascent trajectory guidance material (P63–P66).
- SpaceX Falcon User’s Guide (public recovery-sequence material).
- NASA/JPL G-FOLD precision-landing context.

## Development order and acceptance gates

1. **Logging correlation only.** Implement the required V2 trace fields above;
   build, install only with KSP closed, and verify actual records from multiple
   landings.  No V1 command changes.
2. **Estimator validation.** Deterministic airless estimator tests from recorded
   snapshots: same snapshot must yield same result; invalid/no-impact and
   post-landing cases must be explicit; compare against V1 only as a diagnostic.
3. **Preflight/planner.** Implement feasibility, budget/reserve, strategic
   long-side corridor, and a rejected-plan explanation.  Still no engine command
   until plan validation and tests exist.
4. **Opt-in airless phase manager.** Add phase-owned warp, alignment, counted
   deorbit, bounded trim, braking, and terminal tracking.  Validate no stale
   command across warp boundaries and planned vs delivered delta-V.
5. **Visual assessment/divert.** Implement one-time rebase, NSEW red-target
   changes, bounded local divert, terrain checks, and reserve gating.
6. **Atmospheric path.** Add robust entry/energy management separately; do not
   present atmospheric prediction as exact touchdown.
7. **Validation matrix.** Mun, Minmus, Tylo, Kerbin, and Duna; low/high TWR;
   with/without RCS; auto-warp on/off; feasible/infeasible delta-V; target
   adjustment and unsafe-site cases.

Do not begin steps 4–6 because one successful V1 landing occurred.  Each gate
needs recorded evidence and an explicit user review.

## Operational rules for the next chat

- Start by reading this file and `LandingGuidanceV2.md` completely.
- State whether a claim is from source, trace, build/install verification, or
  observed runtime.  Do not substitute one for another.
- Inspect the V2 JSONL path before claiming that V2 has or has not logged data.
- Preserve the current installed DLL and timestamped backup before any build or
  install.  Install only while `KSP_x64` is closed and hash-verify the copied
  DLL.
- Use Git commits for all source/document changes.  Never rely on an uncommitted
  working DLL as the only record of behaviour.
- Do not change V1 or move its UI while doing a V2 logging-only change.
- Do not ask the user to perform subjective timing observations when a trace can
  measure the event.  Improve the trace first.
