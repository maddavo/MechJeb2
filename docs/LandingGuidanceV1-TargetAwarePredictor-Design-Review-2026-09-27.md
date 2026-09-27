# V1 targeted airless predictor: verified design review

**Reviewed:** 2026-09-27; `design/landing-guidance-v2` at `8db84cf3`
**Reference only:** V1 Beta `d8ac3dd5` still has the Mun early-braking fault.
**Status:** Passive deterministic-capture facility and offline reader added and built on 2026-09-28. A Mun run now supplies a structurally complete predictor capture. Active predictor repair remains deferred.

This review checks the earlier version of this document against the current V1 source, V1 Beta, `LandingGuidanceV1-Predictor-Investigation.md`, `LandingGuidanceV2.md`, `LandingGuidanceV2-Handoff-2026-09-20.md`, `LandingGuidanceV1.trace.jsonl`, and its matching `KSP.log` event. The V2 handoff is historical; the current branch is authoritative. The V2 architecture is a separate opt-in project, not an implementation path for this V1 repair.

## Evidence and limits

The counts in the next paragraph describe the trace as it stood at the original 2026-09-27 review. The later Mun capture and appended trace session are analysed below.

The current V1 JSONL trace is an append-only 8,917,536-byte file with four distinguishable UT-reset sessions. Of 9,162 physical lines, 9,160 parse as JSON and two are malformed at session boundaries (lines 4593 and 6689); analysis must segment by UT reset and not merge repeated UTs across sessions. Only the final session (lines 7474–9162) has the r9 later-braking planner: **one** batch of nine `target_aware_candidate` records at process UT 24,603,456.888 and 24 `retain_target_aware_plan` records. `KSP.log` records its window (earliest 24,603,456.83, target-radius intersection 24,604,573.95) and its selected forced start 24,604,014.39. Candidate longitude moves from −17.93° through 26.76° past the target longitude 23.473° while unsigned target distance falls from about 141 km to 11.5 km, then rises. The trace does not contain a signed approach-frame error; the closest grid point is **not** a converged target landing. Candidate records have `terrainProfileApplied:false`; their ranking used spherical endpoints.

The selected result appears in `guidance_state` at UT 24,603,457.49 as version 171, input UT 24,603,456.83, endpoint longitude 26.57° (target 23.473°). Ordinary immediate-braking results are retained until input UT 24,603,461.81. At input UT 24,603,462.21 an ordinary result is accepted; version 173 has endpoint longitude −17.58° at state UT 24,603,462.59. There is no second target-aware batch in this session. This **directly proves replacement after the five-second guard**. The source's cooldown/flag ordering explains the missing refresh; the trace alone does not prove the flag's value.

The final session ends during `CourseCorrection`, not at touchdown. Its later `NO_REENTRY` records and `guidance_state` rows show commanded throttle near 0.167 while a correction pulse remains marked burning. This is evidence that publishing an invalid result alone does not clear an already commanded pulse. The trace does not contain a complete immutable vessel/terrain snapshot, worker IDs, terrain-query timing, or plan runtime, so it cannot by itself validate a new simulator or performance budget. The JSONL also does **not** record the selected plan as a `predictor_result` publication event; that event is in `KSP.log`.

## Verified V1 lifecycle at the reviewed baseline

| Path | Creation, publication, expiry, and ownership |
| --- | --- |
| Setup | `LandingGuidance` can hold the predictor enabled for display. `LandAtPositionTarget` adds the autopilot as a predictor user, calls `ResetTargetedLandingPrediction`, removes maneuver nodes, then selects `CourseCorrection`, `PlaneChange`, or geometric `DeorbitBurn`. UI updates `maxOrbits` and `noSkipToFreefall`; `LandingAutopilot.Drive` supplies predictor policy, target ASL + 200 m braking end, chute multiplier, and separate V1 controller policy. These updates occur on UI/drive paths, not as one captured transaction. |
| Ordinary simulation | `OnModuleEnabled` starts a run; `OnFixedUpdate` drains ordinary and plan queues before attempting another. `TryStartSimulation` targets five ordinary runs per second, plus a separately timed parachute-error stream. `StartSimulation` obtains a reentering patch or current orbit and passes `patch.StartUT`, an orbit, copied curves/vessel, current policy/acceleration, and on airless bodies `probableLandingSiteASL = 0`. It stores generation in the worker job. `ReentrySimulation.Init` copies the orbit into `_initialOrbit`, but `Result.InputInitialOrbit` refers to the supplied orbit object rather than that copy. A design must not treat that result field as an immutable snapshot. |
| Simulator | `OrbitReenters` checks periapsis against braking/atmosphere radii. `FindFreefallEndTime` analytically coasts, unless its speed condition is already true or skipping is disabled. `RunSimulation` records `StartPosition`, `SimulatedBrakingStartUT`, an integrated trajectory, outcome, endpoint, end velocity/speed, virtual Δv, and input diagnostics. `Landed` uses the supplied *probable landing-site radius*, not actual terrain. `NO_REENTRY` returns before normal body/endpoint/trajectory fields are filled. `Result` pooling resets only `AeroBrake`; old numeric/position fields can remain on a reused non-impact result. Trace `NO_REENTRY` coordinates and steps must not be interpreted as a valid new endpoint. `_resultId++` is shared by worker threads without an atomic increment; it is not a reliable transaction identity. |
| Ordinary worker/queue | A worker checks generation before enqueuing and releases an already-stale result. The queued result has no generation tag, and `CheckForResult` does not recheck on dequeue. `ERROR` is logged and released, leaving the previous published result in place. A thrown worker exception logs but can leave `SimulationRunning`/timing state set, preventing refresh. `ResetTargetedLandingPrediction` releases current/candidate/error results and queued plans, but does not drain `readyResults`; disable also leaves the published result and ordinary queue. |
| Main-thread terrain/acceptance | `CheckForResult` queries endpoint terrain for every non-error result, routes error-multiplier results to `errorResult`, and for ordinary airless `LANDED` results finds the first recorded path sample at/below actual terrain. Two consecutive landing results must agree on endpoint, arrival time, and terrain ASL before publication; non-landing normally invalidates immediately. `PublishNormalResult` releases its predecessor and increments `ResultVersion`. A retained target-aware result is checked **before** non-landing invalidation, so a fresh loss of impact can be hidden temporarily. `PredictionReady` only tests `result != null && Outcome == LANDED`; it has no age, body, target, or generation check. |
| Parallel plan | `StartTargetAwareBrakingPlan` requires active targeted airless V1 and a normal `LANDED` result. It tests `CompareExchange(brakingPlanRunning,1,0)` **before** the five-second cooldown; a cooldown return leaves the flag at 1. It launches nine simulations from normal policy brake start to two seconds before the target-ASL radius crossing. Nine simulated vessels and curve sets, and repeated target-height queries, are prepared on the flight thread while draining `readyResults`. The worker processes candidates serially, then queues the set. On an exception, unprocessed simulations are not all guaranteed released. |
| Plan selection/publication | The flight thread checks generation and target existence, but not the original target coordinates, body, landing mode, or result age. It computes unsigned current-target distance; `TargetAwareBrakingPlan.Candidate.DownrangeError` receives that unsigned value and crossrange is set to zero. It ranks spherical endpoints, resolves actual terrain **only for the winner**, then publishes directly without two-snapshot consensus or a final target check. A changed target can select or publish an old plan; an old queued plan can appear after abort if the predictor stays enabled. |
| Replacement/stop | `AcceptNormalResult` discards ordinary results for less than five seconds after selected *input UT*, then admits them through ordinary consensus. There is no explicit plan expiry or refresh guarantee. `StopLanding`/autopilot disable removes its predictor user and clears the descent policy, but predictor disable and generation reset depend on other users; `LandUntargeted` does not itself clear an old targeted result/plan. Releasing a published pooled result while a reader retains its object reference risks observing a reused object. |

`ResolveAirlessTerrainProfileContact` changes `EndPosition`, `EndUT`, and `EndASL`, but does not update `EndVelocity`, `EndSurfaceSpeed`, `TimeToComplete`, `DeltaVExpended`, or truncate the trajectory. It is a terrain-contact correction, not a complete physical result rewrite. The candidate simulator can also stop at target ASL before encountering lower actual terrain; if profile contact is absent, a spherical `LANDED` outcome can remain. A terrain profile may hit a ridge *before* the target-radius intersection. Both cases make a single target-radius impact time an insufficient contact bound.

### Every published-result reader and writer

| Reader/writer | Dependency and effect |
| --- | --- |
| `MechJebModuleLandingPredictions` | Only ordinary consensus and selected-plan paths assign `result`; reset releases it. `Result`/`GetResult` expose the same pooled object. Map view reads endpoint/trajectory for the blue marker; aerobrake-node maintenance reads atmospheric outcome/orbit. `GetErrorResult` re-queries and writes error-result `EndASL`. |
| `MechJebModuleLandingAutopilot` | `Prediction`, `PredictionReady`, and `PredictionVersion` expose the result to phases. `_landingAltitude` re-queries terrain and **writes published `EndASL`**. `LandingSite` and `ComputeCourseCorrection` read endpoint and end UT. The latter's orbit-intersection radius, correction geometry, and impact-preserving pulse check use `DecelerationEndAltitude()`, which in V1 depends on the **predicted endpoint terrain**. Changing the endpoint therefore changes V1 control inputs even if its code is untouched. Parachute planning reads normal/error results and stores pooled-result references/IDs for deduplication; preserve its atmospheric behavior. |
| `PlaneChange`, `DeorbitBurn`, `UntargetedDeorbit` | No prediction read. The restored primary `DeorbitBurn` has a geometric cutoff and feeds `CourseCorrection`. V1 Beta is not a solution to early virtual braking. |
| `LowDeorbitBurn` | Its code reads the predicted site for steering, range comparison, and braking transition, but **no current source path constructs this step**. `UseLowDeorbitStrategy()` selects `PlaneChange`, which transitions to `DeorbitBurn`; this is dormant code, not an active alternate route. |
| `CourseCorrection` | Endpoint drives completion and pulse direction. Post-pulse it requires a newer version with input UT at least 0.75 s after burn completion and two direction-consistent candidate predictions. On `!PredictionReady`, it simply returns, leaving prior throttle/attitude state; the current trace shows this during `NO_REENTRY`. |
| `CoastToDeceleration` | `Drive` updates RCS only on a new result version; `!PredictionReady` returns without clearing a previous RCS command. `OnFixedUpdate` always zeroes throttle, then reads `LandingSite` without a readiness guard when targeted, and may set warp. It uses `Trajectory.First().UT` for coast orientation. |
| `DecelerationBurn` | `OnFixedUpdate` dereferences `Prediction.Trajectory` without a readiness guard for warp, and `ComputeCourseCorrection(false)` uses the endpoint during braking. Loss of the published result can throw through `AutopilotModule`'s catch, leaving prior control state. Actual braking throttle follows V1's separate speed policy. |
| `KillHorizontalVelocity`, `FinalDescent` | The former returns on `!PredictionReady` without clearing a previous hover command. The latter does not read prediction; it uses vessel/terrain state for touchdown. |
| Landing Guidance UI, trace, compatibility | UI reads result outcome, endpoint, ASL, error, Δv and time. `LandingPredictorTrace` reads fields for JSONL and state rows. `GetResult()` is the compatibility access point; there is no direct repository caller. Maneuver Planner only excludes the predictor's aerobrake node. |

## Exact root cause

`SafeDescentSpeedPolicy.MaxAllowedSpeed` computes `0.9 × sqrt(2 × (thrust acceleration − gravity) × radial height above braking end)`: a stopping envelope that assumes the full speed must be removed within remaining *vertical* distance. `ReentrySimulation.FreefallEnded` compares this with **total rotating-surface speed**. On a shallow descending Mun trajectory, horizontal orbital speed can exceed the envelope immediately, so `FindFreefallEndTime` returns the current input UT; numerical stepping and `LimitSpeed` begin virtual retrograde braking there. The policy never evaluates target downrange. The powered endpoint is therefore short even though unpowered contact is long. Predictor target-terrain anchoring removes the earlier circular altitude input but cannot fix this timing geometry. The separate pooled-simulation `_startX` correction is already present and must be retained.

## Corrections to the previous design

1. **No invented terrain revision.** KSP supplies no verified terrain revision token in this path. Capture target/body identity and target terrain ASL; re-query on the flight thread at publication and on meaningful refresh, with defined tolerance, and re-resolve candidate contact. A terrain change found there invalidates the transaction. No worker thread performs Unity/PQS terrain queries.
2. **No target-radius-only or sea-level-only impact claim.** A target-radius crossing bounds the search but can miss an earlier ridge or fail for a trajectory whose periapsis stays above sea level while crossing elevated ground. The unpowered pass needs a first actual terrain-profile contact or an explicit `NoContact/UnknownTerrain` result. Candidate propagation must continue far enough for first contact even when local terrain lies below target ASL. A missing profile crossing is not `LANDED`.
3. **No nearest-grid-point feasibility claim.** Use a fixed body-rotating approach frame from the *captured ballistic pass*, not candidate endpoint or current target. Project target-to-powered-contact displacement onto signed downrange (positive long) and crossrange. Bracket a sign change, refine brake UT within bounded iterations, and report residuals separately. Non-monotonic or discontinuous terrain/contact branches require branch-aware rejection or a bounded local search. Never manufacture a zero crossing by minimising unsigned distance. `LANDED` in this simulator means crossing its stopping radius, not safe touchdown or available fuel; do not use a hard 15 m/s terminal-speed gate or label candidates flight-safe on that basis.
4. **One owner, including refresh.** Active targeted airless V1 has a single transaction scheduler and publication gate. Ordinary scalar results cannot own or replace the control-facing slot in that mode, before or after a timer. The next complete transaction must use a newer captured snapshot; old worker/queue items are checked at dequeue. Candidate expiry is logged and prevents its publication; it does not clear an earlier committed valid result or add a V1 controller response. No result derived from another result's powered endpoint or terrain may seed a new airless transaction.
5. **Published object consistency.** Complete contact resolution before publication. Store contact state (including velocity/speed/time/Δv at contact) or distinguish unavailable post-contact diagnostics; never expose an endpoint/UT from contact with velocity/trajectory/Δv from later simulated travel as though all describe one event. Retain the V1 meaning of `Trajectory.First().UT` as virtual brake start and preserve `InputUT` for the post-burn gate. The legacy `ReentrySimulation.Result` remains the compatibility type, but publication metadata/validity belongs to an immutable sidecar; internal readers must not mutate the published object after acceptance.
6. **Preserve existing V1 no-prediction behavior.** The source and trace show that correction and hover can leave prior commands in place, coast can leave an RCS command, and braking can assume a result. This repair does not add an Abort system, recovery maneuver, phase guard, command clearing, or any other controller response. The active predictor's contract is narrower: a failed, stale, incomplete, or invalid candidate never replaces a previously committed valid result. It logs the failure and leaves V1's existing behavior authoritative.

## Corrected active transaction design

**Capture.** On the flight thread, take one coherent orbit-at-UT/vessel/body/target/policy snapshot, including target ASL, mass, available thrust acceleration, body rotation, current phase, and generation. Validate finite values and target body. Avoid `Result.InputInitialOrbit` as snapshot storage; store an owned orbit copy or equivalent immutable state. Capture once per refresh; do not create nine vessel/curve copies or repeat target PQS queries on a physics-frame drain. Keep a bounded worker count and coalesce refresh requests during a running job.

**Ballistic contact.** On the same captured orbit, find a bounded descending pass, use target ASL only as an initial radius estimate, and sample the near-surface path against terrain on the flight thread within an explicit query budget. Record first contact UT, rotating-body position, approach tangent, and terrain source. Detect no crossing, elevated-terrain-only crossing, pass expiration, and uncertainty due to coarse sampling. Do not use the early powered `source.EndUT` as the horizon.

**Powered search.** Evaluate forced coast-to-brake continuations before first ballistic contact from that same snapshot. Coarse evaluations may use a bounded spherical approximation, but any candidate used to establish the final bracket or winner must have first-contact terrain resolution and a consistent contact state. Refine the signed downrange root with bounded iterations; check crossrange and contact quality independently. Record `NoBracket`, `CrossrangeOutOfBounds`, `TerrainUnresolved`, `SimulationError`, and `NoContact` explicitly. Model output remains a predictor estimate, not a V2 feasibility or fuel-reserve verdict.

**Publish and refresh.** In targeted airless mode, one flight-thread gate owns the control-facing result slot. Its committed result has target-aware provenance and immutable transaction lineage: landing/target generation, body, target coordinates and terrain ASL, input snapshot UT, transaction sequence, model, terrain contact, and residuals. `DeorbitBurn` to `CourseCorrection` is a phase transition within that same landing/target generation; it neither clears the committed result nor changes its provenance. A refresh captures newer vessel state and runs the **same target-aware ballistic/coast/brake/terrain model**. Ordinary immediate-braking results may be captured for diagnostics but never enter the active slot or advance `ResultVersion` in this mode, regardless of elapsed time or their agreement with each other. The gate checks current generation, unchanged mode/target/body/terrain, monotonic transaction lineage, bounded snapshot age, and a complete consistent first-terrain-contact result. It then atomically swaps the complete compatible `Result` and immutable sidecar, advances `ResultVersion` once, and releases the predecessor only after the new commitment is established. Out-of-order, failed, stale, incomplete, or terrain-unresolved work is traced and released without replacing the previous committed valid result. A phase label alone is not a strict equality key: a valid deorbit snapshot may finish during course correction if all lineage and freshness checks pass. On target edit, abort, or mode change, trace/cancel outstanding work and preserve V1's existing published-result and controller behavior; no new V1 response is introduced. `Result`/`GetResult` remain the externally visible slot.

**Phase contract.** Preserve the exact current V1 phase transitions and its behavior when `PredictionReady` is false or a result is absent. In particular, `CourseCorrection.Drive` and `KillHorizontalVelocity.Drive` return without clearing prior commands; `CoastToDeceleration.Drive` returns without clearing prior RCS state, while its `OnFixedUpdate` still sets throttle to zero and can read `LandingSite`; `DecelerationBurn.OnFixedUpdate` assumes `Prediction.Trajectory`; dormant `LowDeorbitBurn` continues its existing burn logic when no result is ready. The predictor repair must avoid causing these paths through bad candidate publication. Improving those controller behaviors is explicitly outside this repair.

## V1 interfaces and nominal behavior to preserve

- `MechJebModuleLandingPredictions.Result`, `GetResult()`, `ResultVersion`, `GetErrorResult()`, and `ReentrySimulation.Result` fields read by existing V1 clients; blue predicted touchdown and red selected target meanings.
- `LandAtPositionTarget` entry choices; `PlaneChange → DeorbitBurn → CourseCorrection → CoastToDeceleration → DecelerationBurn → KillHorizontalVelocity/FinalDescent` where those existing gates apply. Preserve geometric deorbit cutoff. Do not introduce `LowDeorbitBurn` into the active sequence; its source remains dormant.
- Nominal V1 correction pulse limits, post-burn input-UT/independent-result gate, impact-preserving pulse bound, coast RCS hysteresis, braking attitude/throttle speed law, low-altitude velocity control, gear/chute/staging behavior, and V1 auto-warp controls. The prediction supplied to those laws changes, so their *outcomes* may change and must be validated.
- Landing Guidance window layout, controls, and manual/untargeted/atmospheric predictor behavior; aerobrake nodes and parachute-error simulations. V2 UI/control authority is outside this repair.

These preservation requirements include the existing no-prediction paths. The passive capture step changes only structured logging and an offline reader.

## Offline validation harness and gates

The harness must drive **production** snapshot, simulation, terrain-contact, scheduler, publication, and phase-read logic through injectable clock, terrain, worker queue, and actuator interfaces. A pure candidate-selector unit test is insufficient. The current JSONL supplies regression geometry and chronology but lacks a complete immutable Mun snapshot; capture or reconstruct one before claiming deterministic replay of the incident. Use the four trace sessions separately.

| Gate | Required demonstration |
| --- | --- |
| Geometry and terrain | Reproduce ballistic long/immediate powered short; signed bracket across the nine observed r9 starts; refine toward target with independent crossrange residual. Test ridge-before-target-radius, high terrain with periapsis above sea level, lower local terrain than target, no contact, non-monotonic branch, contact interpolation, and internally consistent endpoint/UT/velocity/trajectory/Δv. |
| Repeated refresh/async | Drive more than two five-second refresh cycles, normal-result arrivals on either side of each boundary, out-of-order and post-reset worker completions, target/body/terrain changes, stop/restart, error/timeout, and pooled reuse. Assert no scalar overwrite, permanent running flag, stale publication, result resurrection, double release, or stuck scheduler. Failed/incomplete candidates leave the last committed valid result unchanged. Measure version and `InputUT` lineage. |
| Deorbit/correction/coast | Traverse restored geometric deorbit to correction; verify a selected endpoint does not alter deorbit cutoff. Simulate pulse completion, settling time, two independent post-burn snapshots, rejection of pre-burn/stale candidates, and unchanged V1 command/warp behavior when no prediction is ready. Assert `LowDeorbitBurn` remains unreachable through current entry/transition paths. |
| Braking/hover/abort | Check selected brake UT reaches `Trajectory.First().UT`, coast and braking warp gates, existing no-prediction behavior in braking/hover, final-descent handoff, manual abort and untargeted switch. A late failed/stale plan must not replace a committed valid result; no new abort or recovery response is introduced. |
| Non-targeted regression | Manual ballistic marker, atmosphere/parachute-error regression, aerobrake node, UI fields, pooled result deduplication, and target-edit behavior remain compatible. |
| Performance | Record wall time and allocations on the flight thread separately from worker time: snapshot/copy preparation, PQS queries, queue drain, trace writes, and result release. Enforce an agreed per-frame/burst bound and maximum concurrent work; compare repeated refresh against V1 Beta/current baseline on representative Mun and Minmus terrain. No worker may call unsafe Unity/PQS APIs. |

### Approved passive capture and provisional acceptance

Capture is enabled only with V1 **Log trace data** during targeted airless prediction and runs at predictor submission/result cadence, not physics-frame cadence. A separate versioned JSONL stream records each ordinary and forced-candidate submission and completion with a stable submission ID: UT, body gravity/radius/rotation axes and angular velocity, orbital position/velocity, vessel mass and thrust-model inputs, target coordinates/terrain ASL, policy and integration settings, phase, outcome/validity, brake timing, simulated endpoint, and terrain-contact resolution. The offline reader pairs records and rejects incomplete or non-finite required inputs; it does not run the active repair. Capture has no path into result selection, V1 control, or UI.

**Capture and reader format.** `LandingGuidanceV1.capture.jsonl` is a separate append-only stream beside the existing V1 trace, written only while targeted airless V1 tracing is enabled. Schema 1 uses `(captureSession, submissionId)` as its key across game restarts. `submission` records input UT, capture UT, body radius/μ/gravity/rotation period, three body axes and angular velocity, BCI position/velocity, mass and thrust fields, selected target and queried height, policy name, solver settings, forced brake UT, phase, generation, and wall-clock counter/frequency. `worker_result` records completion, outcome, field completeness, simulated brake/start/end, trajectory first sample/count, end velocity/speed and virtual Δv; `worker_exception` and `submission_abandoned` record jobs that did not produce a result. `resolved_result` records main-thread processing UT, phase, and generation, disposition, terrain sample quadruples `[lat, lon, trajectory ASL, terrain ASL]` from the existing resolver, contact index/height and measured query count/time. `selection_decision` records the ordinary result's existing consensus/retention decision and compared submission ID; `published` records the control-facing result version, generation, phase, and UT. `discarded` and `lifecycle` distinguish replacement, stale worker/queue releases, target reset, untargeted switch, stop, and predictor disable from a truncated file. `capture_error` makes a snapshot failure visible. A simulator `LANDED` outcome and an actual terrain contact are separate flags; current V1 can publish incomplete/non-contact outcomes, and capture observes that without changing it. A stale job may have a worker record without a resolution record.

Run `python tools/landing_predictor_capture_reader.py LandingGuidanceV1.capture.jsonl --output mun-replay.json` to validate and export `mechjeb-v1-predictor-replay-input` format version 1, with `cases` pairing the raw submission, worker, resolved, publication, and discard records plus an ordered lifecycle list. Strict mode rejects missing submissions/completions, publication without resolution, snapshot capture errors, malformed JSON, and non-finite required inputs. `--allow-incomplete` exports explicit `gaps` for an interrupted recording. The reader deliberately does not reject an existing V1 publication merely because its captured `complete` or terrain-contact flag is false; that would hide evidence of the present lifecycle. The exported fixture is data for a later offline simulator and lifecycle harness, not a validated Mun replay by itself. Only terrain queries V1 actually made are captured; unseen candidate paths need a separate bounded terrain oracle or explicit uncertainty in the later terrain-aware harness. Do not claim an exact terrain replay for unqueried positions.

Provisional active-repair criteria, to be calibrated from the captured Mun replay:

1. A solution has a signed-downrange bracket or a refined local minimum around the target. Timing interval and signed residual must both be demonstrably smaller than the intended targeting tolerance before refinement stops.
2. Log crossrange separately; brake-time refinement must not claim to solve it.
3. Publish only one complete result from one snapshot and one predictor model. Failed/stale/incomplete/invalid candidates never replace a committed valid result.
4. Perform no blocking simulation on the main thread. Bound and measure main-thread terrain queries and capture overhead.
5. Derive final downrange, crossrange, timing, refresh-age, and performance limits from offline Mun replay; do not invent final numerical values before that evidence exists.

Before **active predictor implementation**, require the passive capture to produce a complete replayable Mun fixture, reader validation, calibrated thresholds, and a reviewed design against those data. Before **any KSP DLL test**, require production-path lifecycle harness gates, focused landing tests, a successful solution build, and a reviewed diff proving V1 controller/UI preservation. A build or plausible candidate endpoints alone is not a gate.

## Mun runtime capture, 2026-09-28

The installed passive-capture build produced `LandingGuidanceV1.capture.jsonl` at
`C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\PluginData\MechJeb2\`.
Its 2,651,828 bytes have SHA-256
`9F6E573C0E02183DC918F279D32A67ABB4C54E457B1C8229C85A6A3B8678BA44`.
Strict `landing_predictor_capture_reader.py` validation finds **3,763 records,
712 submissions, zero structural gaps, and 461 publications** in one capture
session. There are 703 ordinary and nine forced-candidate submissions; every
submission has a worker result and a flight-thread resolution. No capture error
was recorded. The output is suitable as the versioned input to a future offline
simulator and lifecycle harness; the reader does not itself replay trajectories.

The matching appended V1 trace session is physical lines 9163–11250. Its final
line 11251 is truncated; the separate capture file parses completely. The Mun
run passed through `PlaneChange`, `DeorbitBurn`, and `CourseCorrection`, then
logged `landing_stopped` and `landing_module_disabled` at UT 24,603,567.09.
This captured run did **not** reach braking, hover, or touchdown. Earlier trace
sessions have braking states, but lack this deterministic capture.

At input UT 24,603,456.77, ordinary submission 172 started virtual braking
immediately and resolved to longitude −18.479°, about 147 km short of target
longitude 23.473°. Its terrain contact was confirmed. The nine forced candidates
shared one captured input snapshot and ranged from immediate braking to UT
24,604,511.81. Candidate 177 delayed braking to UT 24,603,984.29 and reached
longitude 23.187°, about 1.01 km from the target by spherical surface distance.
The bracket's next candidate passed to longitude 38.061°. Projecting the
trace's unpowered osculating position through the captured body axes and
rotation state gives an approximate target-height crossing near longitude 89°.
That is evidence of a long ballistic path, **not** a terrain-resolved impact.

Candidate 177 had no confirmed terrain contact. Its last recorded sample was
about 492 m ASL over terrain about 450 m ASL, leaving roughly 42 m to local
ground. The simulator stopped at the target-height radius; `resolvedEndASL`
remained zero despite an endpoint radius implying about 492 m ASL. The earlier
planner nevertheless published it as result version 261. This is an incomplete
target-aware result, not a validated landing near the target. The capture has
terrain samples only along paths V1 queried; it cannot establish unseen ground
contact for the candidate or the ballistic arc.

V1 retained 24 subsequent immediate-braking results for its five-second guard.
Ordinary submission 207 replaced the selected result as version 262 at UT
24,603,462.15, **5.32 seconds** after selection. There was no second forced
batch. By the end, V1 had published `NO_REENTRY` version 550 in
`CourseCorrection`; the last state still showed a previously commanded throttle
near 0.19. This documents existing V1 behavior and does not authorise a new
controller response.

### Exact deorbit-to-correction publication sequence

The newest completed capture session is
`a398605cbd7942b0837d4fe4a3493229` (712 paired submissions, target reset
through landing stop/module disable). Its publication records and the matching
V1 `guidance_state` rows resolve the visual jump more precisely than the phase
label alone:

| Event | Input and processing UT | Published result / phase | Evidence |
| --- | --- | --- | --- |
| Near-target selection | Submission 177 input 24,603,456.768967; published 24,603,456.828967 | Version **261**, `DeorbitBurn`; forced brake UT 24,603,984.289534 | Endpoint 0.631151°, 23.186603°; about 1,011 m spherical target error. Worker `LANDED`/`complete:true`, but `terrainContactConfirmed:false`; it was not a complete terrain-resolved landing. `guidance_state` still displayed version 261 at UT 24,603,461.348967. |
| Guard expires | Submission 205 input 24,603,461.728967 was retained; submission 206 input 24,603,461.888967, processed 24,603,461.928967 | No publication yet; ordinary candidate 206 replaces pending ordinary candidate 172 | The guard compares ordinary **input UT** with selected result **input UT**, not publication UT. Submission 206 was 5.12 s newer, so `retain_target_aware_plan` no longer applied. Its `replace` decision compared candidate 172, not the active 177. |
| First overwrite | Submission 207 input 24,603,462.108967; published 24,603,462.148967 | Version **262**, still `DeorbitBurn`; ordinary immediate brake UT 24,603,462.108967 | Terrain contact confirmed at 1.029053°, −17.579287°; trace resolved target error about 140,243 m. Decision `candidate_accept` compared ordinary 206. `PublishNormalResult` discarded/released active 177. This is the actual near-to-far publication change, **before** the phase transition. |
| Last deorbit publication | Submission 209 input 24,603,462.508967; published 24,603,462.568967 | Version **263**, `DeorbitBurn`; ordinary | It replaced 262 after agreement with ordinary 208; endpoint longitude −17.560244°. |
| First correction publication and use | Submission 210 input 24,603,462.728967 was pending; submission 211 input 24,603,462.948967 published 24,603,462.988967 | Version **264**, `CourseCorrection`; ordinary | Contact at 1.031008°, −17.541998° after agreement with 210. First `CourseCorrection` state at UT 24,603,463.388967 consumed version 264 and logged target error about 141,210 m, downrange error about 131,291 m, status "correction of about 18.4 m/s," and a 1 m/s pulse. |

The actual phase change is bounded by the last `DeorbitBurn` publication at UT
24,603,462.568967 and the first `CourseCorrection` result processing at UT
24,603,462.768967. The first sampled `CourseCorrection` guidance state is later.
Thus the observed jump *at* the transition was the visible consequence of an
ordinary overwrite that had already happened during deorbit. The phase did not
invalidate version 261. Source `AcceptNormalResult` has only a five-second
retention check; after it expires, ordinary two-result consensus can call
`PublishNormalResult` on the same control-facing slot. Source
`StartTargetAwareBrakingPlan` tests `CompareExchange(brakingPlanRunning,1,0)`
before its cooldown and can leave the flag set on an early return. The capture
shows no second forced batch, which is consistent with that path but does not
record the flag itself.

**Required invariant from this case:** while one targeted airless landing keeps
the same target/body generation, phase transitions and refresh timers cannot
change the active slot's target-aware provenance. Each refresh uses a newer
snapshot with the same target-aware model; only a fully resolved newer
target-aware transaction may atomically replace the committed result. Ordinary
results have no route to that slot. A failed, stale, incomplete, or
terrain-unresolved transaction cannot displace it. In particular, historical
candidate 177 would **fail** the new terrain-contact gate: this rule protects a
previous *valid* target-aware result, not that incomplete historical candidate.

For the 348 `LANDED` worker results, captured simulation time had median
0.632 ms and maximum 3.369 ms. The 340 resolutions that queried terrain had
median 15 and maximum 17 queries, with measured query time median 0.101 ms
and maximum 0.159 ms. These are observations from this run, not acceptance
budgets. Main-thread snapshot/copy cost, unqueried terrain, and absent later phases
prevent final numerical performance and targeting thresholds. The offline
harness must use this fixture to test the one-snapshot transaction and repeated
refresh, and must explicitly mark unknown terrain rather than treating the
spherical candidate as confirmed contact.

## Readiness and remaining design issues

**Passive capture has been exercised in KSP and its Mun session passes strict reader validation. The capture resolves the ownership/overwrite mechanism and the deorbit-to-correction publication chronology, but active predictor implementation is not ready.** It does not supply confirmed contact for the selected target-aware candidate, terrain samples beyond its target-height stop or along the unpowered impact path, a refined signed-downrange/crossrange solution, final calibrated targeting and performance limits, or deterministic braking/hover/abort coverage. The remaining design work is a terrain-bounded offline simulator and production-path lifecycle harness, explicit treatment of unqueried terrain, and calibrated downrange, crossrange, contact, timing, refresh-age, and performance thresholds from replay. V1 no-prediction behavior remains unchanged by decision.
