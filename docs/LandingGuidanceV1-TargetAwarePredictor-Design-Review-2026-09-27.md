# V1 targeted airless predictor: verified design review

**Reviewed:** 2026-09-27; `design/landing-guidance-v2` at `8db84cf3`
**Reference only:** V1 Beta `d8ac3dd5` still has the Mun early-braking fault.
**Status:** A source-only active predictor prototype and offline replay are in progress on 2026-09-28. No revised DLL has been installed or tested in KSP. The original Mun capture is complete for the old predictor; terrain along newly refined paths and actual terminal-controller behaviour remain unobserved.

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

`ResolveAirlessTerrainProfileContact` changes `EndPosition`, `EndUT`, and `EndASL` when it finds contact, but does not update `EndVelocity`, `EndSurfaceSpeed`, `TimeToComplete`, `DeltaVExpended`, or truncate the trajectory. That correction cannot be published as one physically consistent contact state. The simulator's `LANDED` means crossing its probable-landing-site stopping radius, not touching terrain. A powered candidate may stop above lower local terrain at a valid terminal-descent handoff; lack of contact is not by itself invalid. Conversely, a ridge may intersect the coast or braking path before handoff. Terrain clearance along the path, endpoint clearance, and terminal-phase capability must be evaluated separately.

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

1. **No invented terrain revision.** KSP supplies no verified terrain revision token in this path. Capture target/body identity and target terrain ASL; re-query on the flight thread at publication and on meaningful refresh, with defined tolerance, and revalidate candidate terrain clearance. A terrain change found there invalidates the transaction. No worker thread performs Unity/PQS terrain queries.
2. **Separate ballistic impact from powered handoff.** A target-radius crossing bounds the search but can miss an earlier ridge or fail for a trajectory whose periapsis stays above sea level while crossing elevated ground. The unpowered pass needs a first actual terrain-profile contact or an explicit `NoContact/UnknownTerrain` result to establish its long-path bound. A powered candidate instead ends at its predictor stopping-radius handoff: reject terrain intersection anywhere on its coast or braking path *before* that endpoint, and require known local terrain and acceptable nonnegative clearance there. Do not demand touchdown or propagate below the endpoint merely to manufacture contact. A simulator `LANDED` outcome only means the stopping radius was reached; absence of terrain contact can be a valid above-ground handoff. V1's actual controller phase transition occurs earlier and needs its own replay check.
3. **No nearest-grid-point feasibility claim.** Use a fixed body-rotating approach frame from the *captured ballistic pass*, not candidate endpoint or current target. Project target-to-powered-handoff displacement onto signed downrange (positive long) and crossrange. Bracket a sign change, refine brake UT within bounded iterations, and report residuals separately. Non-monotonic or discontinuous terrain-clearance branches require branch-aware rejection or a bounded local search. Never manufacture a zero crossing by minimising unsigned distance. `LANDED` in this simulator means crossing its stopping radius, not safe touchdown or available fuel; do not use a hard 15 m/s terminal-speed gate or label candidates flight-safe on that basis. Validate the handoff clearance and speed against V1's existing terminal-phase capability instead.
4. **One owner, including refresh.** Active targeted airless V1 has a single transaction scheduler and publication gate. Ordinary scalar results cannot own or replace the control-facing slot in that mode, before or after a timer. The next complete transaction must use a newer captured snapshot; old worker/queue items are checked at dequeue. Candidate expiry is logged and prevents its publication; it does not clear an earlier committed valid result or add a V1 controller response. No result derived from another result's powered endpoint or terrain may seed a new airless transaction.
5. **Published object consistency.** Complete terrain-clearance validation before publication. Record handoff UT/position, local terrain ASL, remaining clearance, vertical and surface speed, and the terminal-descent requirement. If the path intersects terrain early, reject it; if the handoff is above terrain, keep all result fields tied to that same handoff event. Any genuine contact adjustment must update velocity/speed/time/Δv and trajectory consistently, or mark unavailable diagnostics explicitly; never mix fields from different simulated events. Retain the V1 meaning of `Trajectory.First().UT` as virtual brake start and preserve `InputUT` for the post-burn gate. The legacy `ReentrySimulation.Result` remains the compatibility type, but publication metadata/validity belongs to an immutable sidecar; internal readers must not mutate the published object after acceptance.
6. **Preserve existing V1 no-prediction behavior.** The source and trace show that correction and hover can leave prior commands in place, coast can leave an RCS command, and braking can assume a result. This repair does not add an Abort system, recovery maneuver, phase guard, command clearing, or any other controller response. The active predictor's contract is narrower: a failed, stale, incomplete, or invalid candidate never replaces a previously committed valid result. It logs the failure and leaves V1's existing behavior authoritative.

## Corrected active transaction design

**Capture.** On the flight thread, take one coherent orbit-at-UT/vessel/body/target/policy snapshot, including target ASL, mass, available thrust acceleration, body rotation, current phase, and generation. Validate finite values and target body. Avoid `Result.InputInitialOrbit` as snapshot storage; store an owned orbit copy or equivalent immutable state. Capture once per refresh; do not create nine vessel/curve copies or repeat target PQS queries on a physics-frame drain. Keep a bounded worker count and coalesce refresh requests during a running job.

**Ballistic contact.** On the same captured orbit, find a bounded descending pass, use target ASL only as an initial radius estimate, and sample the near-surface path against terrain on the flight thread within an explicit query budget. Record first contact UT, rotating-body position, approach tangent, and terrain source. Detect no crossing, elevated-terrain-only crossing, pass expiration, and uncertainty due to coarse sampling. Do not use the early powered `source.EndUT` as the horizon.

**Powered search.** Evaluate forced coast-to-brake continuations before first ballistic contact from that same snapshot. Coarse evaluations may use a bounded spherical approximation, but candidates establishing the final bracket or winner need bounded terrain-clearance checks along their coast and braking paths and at their handoff points. Reject early intersection, unknown clearance, and a handoff below local ground; allow a nonnegative above-ground clearance within a replay-calibrated range that V1's terminal phase can handle. Record local terrain ASL, remaining clearance, vertical and surface speed, and required terminal deceleration/hover capability. Refine the signed downrange root with bounded iterations; check crossrange, terrain clearance, and terminal-phase capability independently. Record `NoBracket`, `CrossrangeOutOfBounds`, `TerrainUnresolved`, `EarlyTerrainIntersection`, `HandoffBelowTerrain`, `TerminalHandoffUnsupported`, and `SimulationError` explicitly. Reserve `NoContact` for the unpowered ballistic pass; lack of contact before a powered handoff is expected. Model output remains a predictor estimate, not a V2 feasibility or fuel-reserve verdict.

**Publish and refresh.** In targeted airless mode, one flight-thread gate owns the control-facing result slot. Its committed result has target-aware provenance and immutable transaction lineage: landing/target generation, body, target coordinates and terrain ASL, input snapshot UT, transaction sequence, model, terrain-clearance and terminal-handoff state, and residuals. `DeorbitBurn` to `CourseCorrection` is a phase transition within that same landing/target generation; it neither clears the committed result nor changes its provenance. A refresh captures newer vessel state and runs the **same target-aware ballistic/coast/brake/terrain model**. Ordinary immediate-braking results may be captured for diagnostics but never enter the active slot or advance `ResultVersion` in this mode, regardless of elapsed time or their agreement with each other. The gate checks current generation, unchanged mode/target/body/terrain, monotonic transaction lineage, bounded snapshot age, no early terrain intersection, known acceptable handoff clearance, V1 terminal-phase capability, and one complete internally consistent handoff result. It does not require terrain contact. It then atomically swaps the complete compatible `Result` and immutable sidecar, advances `ResultVersion` once, and releases the predecessor only after the new commitment is established. Out-of-order, failed, stale, incomplete, or terrain-unresolved work is traced and released without replacing the previous committed valid result. A phase label alone is not a strict equality key: a valid deorbit snapshot may finish during course correction if all lineage and freshness checks pass. On target edit, abort, or mode change, trace/cancel outstanding work and preserve V1's existing published-result and controller behavior; no new V1 response is introduced. `Result`/`GetResult` remain the externally visible slot.

**Phase contract.** Preserve the exact current V1 phase transitions and its behavior when `PredictionReady` is false or a result is absent. In particular, `CourseCorrection.Drive` and `KillHorizontalVelocity.Drive` return without clearing prior commands; `CoastToDeceleration.Drive` returns without clearing prior RCS state, while its `OnFixedUpdate` still sets throttle to zero and can read `LandingSite`; `DecelerationBurn.OnFixedUpdate` assumes `Prediction.Trajectory`; dormant `LowDeorbitBurn` continues its existing burn logic when no result is ready. The predictor repair must avoid causing these paths through bad candidate publication. Improving those controller behaviors is explicitly outside this repair.

## V1 interfaces and nominal behavior to preserve

- `MechJebModuleLandingPredictions.Result`, `GetResult()`, `ResultVersion`, `GetErrorResult()`, and `ReentrySimulation.Result` fields read by existing V1 clients; blue predicted touchdown and red selected target meanings.
- `LandAtPositionTarget` entry choices; `PlaneChange → DeorbitBurn → CourseCorrection → CoastToDeceleration → DecelerationBurn → KillHorizontalVelocity/FinalDescent` where those existing gates apply. Preserve geometric deorbit cutoff. Do not introduce `LowDeorbitBurn` into the active sequence; its source remains dormant.
- Nominal V1 correction pulse limits, post-burn input-UT/independent-result gate, impact-preserving pulse bound, coast RCS hysteresis, braking attitude/throttle speed law, low-altitude velocity control, gear/chute/staging behavior, and V1 auto-warp controls. The prediction supplied to those laws changes, so their *outcomes* may change and must be validated.
- Landing Guidance window layout, controls, and manual/untargeted/atmospheric predictor behavior; aerobrake nodes and parachute-error simulations. V2 UI/control authority is outside this repair.

These preservation requirements include the existing no-prediction paths. The passive capture step changes only structured logging and an offline reader.

## Offline validation harness and gates

The harness must drive **production** snapshot, simulation, terrain-clearance, scheduler, publication, and phase-read logic through injectable clock, terrain, worker queue, and actuator interfaces. A pure candidate-selector unit test is insufficient. The passive capture now supplies a structurally complete Mun predictor-input snapshot; a terrain oracle for unqueried positions and production-path replay are still needed before claiming deterministic reproduction of a repaired prediction. Keep the historical trace sessions separate from the appended Mun capture session.

| Gate | Required demonstration |
| --- | --- |
| Geometry and terrain | Reproduce ballistic long/immediate powered short; signed bracket across the nine observed r9 starts; refine toward target with independent crossrange residual. Test ridge intersection during coast/braking, high terrain with periapsis above sea level, lower local terrain than target, positive handoff clearance without contact, below-ground handoff, uncertain terrain between samples, non-monotonic branch, and internally consistent endpoint/UT/velocity/trajectory/Δv. A clear path ending above ground can pass the terrain gate; an earlier intersection cannot. |
| Repeated refresh/async | Drive more than two five-second refresh cycles, normal-result arrivals on either side of each boundary, out-of-order and post-reset worker completions, target/body/terrain changes, stop/restart, error/timeout, and pooled reuse. Assert no scalar overwrite, permanent running flag, stale publication, result resurrection, double release, or stuck scheduler. Failed/incomplete candidates leave the last committed valid result unchanged. Measure version and `InputUT` lineage. |
| Deorbit/correction/coast | Traverse restored geometric deorbit to correction; verify a selected endpoint does not alter deorbit cutoff. Simulate pulse completion, settling time, two independent post-burn snapshots, rejection of pre-burn/stale candidates, and unchanged V1 command/warp behavior when no prediction is ready. Assert `LowDeorbitBurn` remains unreachable through current entry/transition paths. |
| Braking/hover/abort | Check selected brake UT reaches `Trajectory.First().UT`, coast and braking warp gates, existing no-prediction behavior in braking/hover, terminal-phase entry, horizontal-velocity kill, and final descent at the candidate's clearance and vertical/surface speed. Replay the actual `DecelerationBurn → KillHorizontalVelocity → FinalDescent` transition with V1's unchanged throttle, attitude, and thrust limits; distinguish predictor stopping-radius state from the controller's earlier phase transition. Exercise manual abort and untargeted switch. A late failed/stale plan must not replace a committed valid result; no new abort or recovery response is introduced. |
| Non-targeted regression | Manual ballistic marker, atmosphere/parachute-error regression, aerobrake node, UI fields, pooled result deduplication, and target-edit behavior remain compatible. |
| Performance | Record wall time and allocations on the flight thread separately from worker time: snapshot/copy preparation, PQS queries, queue drain, trace writes, and result release. Enforce an agreed per-frame/burst bound and maximum concurrent work; compare repeated refresh against V1 Beta/current baseline on representative Mun and Minmus terrain. No worker may call unsafe Unity/PQS APIs. |

### Approved passive capture and provisional acceptance

Capture is enabled only with V1 **Log trace data** during targeted airless prediction and runs at predictor submission/result cadence, not physics-frame cadence. A separate versioned JSONL stream records each ordinary and forced-candidate submission and completion with a stable submission ID: UT, body gravity/radius/rotation axes and angular velocity, orbital position/velocity, vessel mass and thrust-model inputs, target coordinates/terrain ASL, policy and integration settings, phase, outcome/validity, brake timing, simulated endpoint, and existing terrain-resolution observations. The offline reader pairs records and rejects incomplete or non-finite required inputs; it does not run the active repair. Capture has no path into result selection, V1 control, or UI.

**Capture and reader format.** `LandingGuidanceV1.capture.jsonl` is a separate append-only stream beside the existing V1 trace, written only while targeted airless V1 tracing is enabled. Schema 1 uses `(captureSession, submissionId)` as its key across game restarts. `submission` records input UT, capture UT, body radius/μ/gravity/rotation period, three body axes and angular velocity, BCI position/velocity, mass and thrust fields, selected target and queried height, policy name, solver settings, forced brake UT, phase, generation, and wall-clock counter/frequency. `worker_result` records completion, outcome, field completeness, simulated brake/start/end, trajectory first sample/count, end velocity/speed and virtual Δv; `worker_exception` and `submission_abandoned` record jobs that did not produce a result. `resolved_result` records main-thread processing UT, phase, and generation, disposition, terrain sample quadruples `[lat, lon, trajectory ASL, terrain ASL]` from the existing resolver, contact index/height and measured query count/time. `selection_decision` records the ordinary result's existing consensus/retention decision and compared submission ID; `published` records the control-facing result version, generation, phase, and UT. `discarded` and `lifecycle` distinguish replacement, stale worker/queue releases, target reset, untargeted switch, stop, and predictor disable from a truncated file. `capture_error` makes a snapshot failure visible. A simulator `LANDED` outcome and actual terrain contact are separate flags: `terrainContactConfirmed:false` can mean an above-ground stopping-radius handoff and is not automatically invalid. The passive reader preserves V1's actual decisions without endorsing their validity. A stale job may have a worker record without a resolution record.

Run `python tools/landing_predictor_capture_reader.py LandingGuidanceV1.capture.jsonl --output mun-replay.json` to validate and export `mechjeb-v1-predictor-replay-input` format version 1, with `cases` pairing the raw submission, worker, resolved, publication, and discard records plus an ordered lifecycle list. Strict mode rejects missing submissions/completions, publication without resolution, snapshot capture errors, malformed JSON, and non-finite required inputs. `--allow-incomplete` exports explicit `gaps` for an interrupted recording. The reader deliberately does not reject an existing V1 publication merely because its captured `complete` or terrain-contact flag is false; that would hide evidence of the present lifecycle, and no-contact can be a valid handoff state. The exported fixture is data for a later offline simulator and lifecycle harness, not a validated Mun replay by itself. Only terrain queries V1 actually made are captured; unseen candidate paths need a separate bounded terrain oracle or explicit uncertainty in the later terrain-aware harness. Do not claim an exact terrain replay for unqueried positions.

Provisional active-repair criteria, to be calibrated from the captured Mun replay:

1. A solution has a signed-downrange bracket or a refined local minimum around the target. Timing interval and signed residual must both be demonstrably smaller than the intended targeting tolerance before refinement stops.
2. Log crossrange separately; brake-time refinement must not claim to solve it.
3. Publish only one complete result from one snapshot and one predictor model. Failed/stale/incomplete/invalid candidates never replace a committed valid result.
4. Require terrain-clearance validation through the powered coast and braking path, a handoff at/above local terrain within a justified terminal-descent range, and evidence that unchanged V1 terminal control can handle its clearance and vertical/surface speed. Terrain contact before handoff fails; absence of contact does not.
5. Perform no blocking simulation on the main thread. Bound and measure main-thread terrain queries and capture overhead.
6. Derive final downrange, crossrange, timing, clearance/terminal-speed, refresh-age, and performance limits from offline Mun replay; do not invent final numerical values before that evidence exists.

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

Candidate 177 had **no terrain intersection in its 11 captured path samples**.
At the simulator stopping-radius endpoint it was 491.965 m ASL over local
terrain at 450.180 m ASL: **41.785 m of clearance**. The simulator stopped at
the target-height radius; its `LANDED` outcome describes reaching that
terminal-descent handoff state, not touchdown. The old resolver's
`resolvedEndASL` remained zero despite the endpoint radius implying 491.965 m
ASL and the local terrain sample measuring 450.180 m ASL. `EndASL` is V1's
local-terrain-height field; the new transaction must populate it consistently
even when no contact occurs. Its legacy zero must not be treated as measured
terrain or clearance. Version
261 was published with `terrainContactConfirmed:false`, which does **not** make
this above-ground handoff terrain-invalid. Its recorded endpoint surface speed
was 71.847 m/s; reconstructing the radial component from the captured endpoint
and velocity directions gives about 58.423 m/s downward. These numbers require
a separate terminal-control check before calling the full trajectory safe. At
that endpoint, the captured 8.060 m/s² maximum thrust acceleration minus
about 1.620 m/s² gravity gives an optimistic straight-up stopping distance
near **265 m** for that radial speed, much greater than 41.785 m. V1 could not
start arresting that descent only at the recorded endpoint.
The 11 samples do not prove clearance between samples or supply the unpowered
ballistic terrain impact; replay needs bounded terrain queries there.

The unchanged V1 controller does not wait until this 42 m endpoint to begin
terminal control. `DecelerationBurn.OnFixedUpdate` transitions to
`KillHorizontalVelocity` below `DecelerationEndAltitude() + 5 m`, which is
approximately 205 m above the predicted endpoint terrain on airless bodies.
`KillHorizontalVelocity` commands a hover while removing horizontal speed and
then enters `FinalDescent` at at most 1 m/s horizontal speed; `FinalDescent`
uses actual vessel altitude and local terrain for its speed envelope. The
capture ended in `CourseCorrection`, so it does not show the speed or attitude
at that actual phase transition. The 42 m clearance is a valid predictor
terrain state; the endpoint speed by itself is **not** a demonstrated safe
terminal handoff. Whether V1 can manage this candidate's descent requires a
production-path replay through its earlier transition and the terminal phases.

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
| Near-target selection | Submission 177 input 24,603,456.768967; published 24,603,456.828967 | Version **261**, `DeorbitBurn`; forced brake UT 24,603,984.289534 | Endpoint 0.631151°, 23.186603°; about 1,011 m spherical target error. Worker `LANDED`/`complete:true`; `terrainContactConfirmed:false` means no sampled contact here. All 11 sampled clearances are positive, ending 41.785 m above local terrain; terminal capability remains unverified. `guidance_state` still displayed version 261 at UT 24,603,461.348967. |
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
terrain-unresolved transaction cannot displace it. Historical candidate 177
passes the **observed** terrain-clearance test; it must not fail merely because
it did not touch down. Its legacy object did not fill `EndASL` at selection,
although the terrain sample exists; a new transaction must commit that metadata
with the handoff state. The 11 samples leave between-sample terrain unresolved,
and the captured run does not prove that the unchanged terminal controller can
handle its speed. Those separate publication checks require replay. No result
may claim complete clearance or terminal capability from `LANDED` alone.

For the 348 `LANDED` worker results, captured simulation time had median
0.632 ms and maximum 3.369 ms. The 340 resolutions that queried terrain had
median 15 and maximum 17 queries, with measured query time median 0.101 ms
and maximum 0.159 ms. These are observations from this run, not acceptance
budgets. Main-thread snapshot/copy cost, unqueried terrain, and absent later phases
prevent final numerical performance and targeting thresholds. The offline
harness must use this fixture to test the one-snapshot transaction and repeated
refresh, and must explicitly mark unqueried terrain rather than treating the
sparse positive samples as a continuous clearance proof.

## Readiness and remaining design issues

**Passive capture has been exercised in KSP and its Mun session passes strict reader validation. The capture resolves the ownership/overwrite mechanism and the deorbit-to-correction publication chronology, but active predictor implementation is not ready.** Candidate 177 has positive clearance in every captured sample and needs no mandatory terrain contact at its stopping-radius handoff. The capture does not establish clearance between those samples, the unpowered ballistic terrain impact, V1 terminal-phase capability at the actual phase transition, a refined signed-downrange/crossrange solution, final calibrated targeting and performance limits, or deterministic braking/hover/abort coverage. The remaining design work is a terrain-bounded offline simulator and production-path lifecycle harness, explicit treatment of unqueried terrain, and calibrated downrange, crossrange, handoff-clearance/speed, timing, refresh-age, and performance thresholds from replay. V1 no-prediction behavior remains unchanged by decision.

## 2026-09-28 source implementation and replay checkpoint

This section updates the historical readiness assessment above. The branch now contains an **uncommitted, source-only prototype**. It does not assert a flight-safe Mun result or authorise DLL installation.

- `AirlessTargetAwareSimulation` copies the orbit/body/rotation/thrust inputs once and evaluates the unpowered path and forced coast-to-brake continuations on a worker. The airless deceleration end comes from the same V1 target-terrain+200 m policy on that snapshot, independent of update order in the mutable predictor fields. Its legacy-mode numerical endpoint, UT, surface speed and step count match all nine captured forced candidates; the focused candidate-177 C# replay also agrees to sub-metre precision. The actual active path interpolates to the target-height stopping radius, keeping handoff position, UT and velocity at the same event.
- `TargetAwareAirlessPlanner` resolves the first sampled unpowered terrain impact, evaluates nine coarse brake times, refines a signed downrange bracket and logs crossrange independently. A distant coarse candidate may intersect a ridge while still supplying a geometric sign for the search; publication requires the final local bracket and selected powered path to be terrain clear. The selected coast is rechecked before commitment when it enters the terrain envelope. Synthetic ridge tests verify that an unsafe distant candidate cannot conceal a separate clear root, while a ridge on the selected branch or coast fails the transaction. Above-ground handoffs are valid terrain states. The terminal check currently proves only a **necessary, optimistic vertical stopping bound** at V1's actual terrain+205 m transition. It cannot prove attitude response, throttle/hover behaviour, fuel, or touchdown; these remain validation gates.
- A single target-aware transaction owns the published result in targeted airless mode. The ordinary queue and old parallel planner queue cannot publish there, including at the final ordinary publication write. A complete newer transaction passes generation, target/body/terrain, snapshot age, sequence, clearance and terminal gates before swapping the V1-compatible result and incrementing `ResultVersion`. Failure, stale work, cancellation and asynchronous out-of-order completion leave the prior committed result in place for the **same** target and terrain. A changed target/body or changed target terrain invalidates that old commitment; V1 then sees its existing no-prediction state until a new transaction completes. The autopilot phase classes and UI source are unchanged.
- The passive capture now records the **same active snapshot** used by the worker, including target query timing and PQS terrain bounds. It records refresh ownership, worker-stage completions, final clearance/handoff validation and publication lineage separately from legacy `terrainContactConfirmed`. The reader accepts these version-1 optional records while preserving strict validation of older captures. `tools/landing_predictor_lifecycle_harness.py --require-active <capture>` audits the emitted event order, phase observations, committed predecessor, version sequence, ordinary-result exclusion and invalidation. Its synthetic tests cover a phase change, repeated refresh, late abandoned worker, terrain invalidation, braking/hover phase labels and an attempted ordinary overwrite. The original Mun capture has zero new-model publications, so it cannot pass `--require-active`. `LANDED` remains the compatibility outcome for V1 `PredictionReady`, not proof of touchdown.

The captured Mun target terrain was exactly 492.187665 m ASL on all 712 submissions, so the current equality gate has no observed terrain jitter in this case. The 340 measured old-predictor terrain resolutions had median 0.101 ms for 15 queries; target-height queries across the 712 submissions had median 0.0092 ms and maximum 0.0309 ms. On a **flat synthetic terrain oracle** using the captured Mun dynamics, the prototype made 387 ballistic, then 402 coarse, then 29 final-bracket/selected refinement terrain queries (818 total). It found 36.14 m signed downrange error and 164.33 m crossrange error, with a 0.258 s timing interval and an optimistic 172.42 m vertical stopping distance at the 205 m V1 transition. These are geometry and synthetic-terrain results, **not** actual Mun terrain validation. The per-flight-update cap is provisionally 32 queries and 1 ms; even 32 times the slowest captured target-height query is about 0.99 ms, while the explicit timer stops a batch sooner if path queries cost more. The total cap is 1024. Measured active PQS time, warp-dependent snapshot age and final numerical limits still need a live capture before a KSP DLL test.

The Python replay independently reproduces all nine historical forced endpoints to sub-millimetre position and microsecond UT precision. Its spherical signed-root estimate is about −46 m downrange and +164 m crossrange. It deliberately labels the refined path `terrain_unresolved`, because the prior capture contains only sparse terrain samples for candidate 177 and no PQS oracle for the new root; the nearest recorded terrain point to that refined endpoint is about 951 m away. Candidate 177's 41.785 m sampled handoff clearance is accepted as a terrain state; its roughly 58.4 m/s downward endpoint speed is not treated as terminal-safe. The current source prototype rejects any candidate whose **sampled** path collides before handoff and retains the prior committed result when terrain or terminal feasibility is unknown. Samples one second apart cannot establish continuous clearance across an unmeasured sharp ridge; final path spacing and any required clearance margin still need calibration with actual Mun PQS data.

**Remaining gates before deployment:** obtain actual Mun terrain and controller-state evidence for the refined path and V1's braking-to-hover-to-final-descent transition; verify the target-aware capture/reader on repeated refreshes and DeorbitBurn→CourseCorrection, plus abort, invalidation, terrain changes and late worker completion; measure active PQS and worker time at V1 warp rates, then calibrate downrange, crossrange, timing, clearance, age and performance limits. The source-only focused predictor tests and build pass. The full repository test run has three failures in unrelated static/ascent tests (8,416 pass, three fail); those failures have not been attributed to this repair. No game files were changed.
