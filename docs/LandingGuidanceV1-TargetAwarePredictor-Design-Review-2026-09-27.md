# V1 targeted airless predictor: verified design review

**Reviewed:** 2026-09-27; `design/landing-guidance-v2` at `8db84cf3`
**Reference only:** V1 Beta `d8ac3dd5` still has the Mun early-braking fault.
**Status:** **V1 Gamma**, designated after Dave's successful 30 September Mun
landing. The demand-loaded spatial-reuse build supplied visible predictions
during deorbit and completed landing. The saved landed position is 888.58 m
west of the target. Deorbit publication cadence remains inadequate: median
4.96 seconds, maximum 9.98 seconds. Dave observed deorbit overshoot and no course
correction, consistent with the captured endpoint crossing in a 2,056 m update
and direct transition to DecelerationBurn. See
[the Gamma flight record](LandingGuidanceV1-Gamma-2026-09-30.md).
The earlier exact-coordinate-cache DLL failed its Mun deorbit test.
The binding integrated-system and corridor-reuse amendment immediately below
supersedes the exact-coordinate cache contract and conflicting acceptance claims
later in this historical review. It supplements the existing approved file-level
plan and V1 Beta control boundary; it does not replace that plan or authorise new
guidance, controller, UI, or actuator behaviour.

### 30 September implementation: moving-flight reuse and local terrain feedback

**Fault ownership.** Dave reports that the tested landing region has no
significant mountains. The capture establishes repeated nearby queries and
failed forecast delivery, not a terrain-complexity explanation. The earlier
implementation was inadequate: exact-coordinate cache keys missed advancing
samples, selected-coast checks repeated PQS work, and terminal replay treated
small differences between its provisional ground plane and resolved local ground
as candidate failures. Failure to supply a forecast allowed the unchanged
prediction-driven low-deorbit step to continue its burn without landing-site
feedback. A future mountain case needs clearance checks, but does not explain or
excuse this availability failure.
Submission 101's 1,536 PQS calls took 10.3176 ms in total (about 0.0067 ms per
call), while its workers took 141.33 ms. Budget exhaustion and candidate
rejection prevented publication; expensive mountain queries are not established
as the cause of its long missing-forecast interval.

**Implemented predictor slice (explicitly authorised in this turn).**

- `TargetAwareTerrainCache` retains actual dynamic PQS samples at stable,
  body-fixed query locations. The planner requests 25 m cells for preliminary
  ballistic/candidate endpoint screening and 1 m cells for selected coast,
  braking and terminal paths. These are provisional spatial resolutions, not
  final accuracy guarantees. No entire corridor is queried or preloaded, and no
  Mun terrain values are embedded in production code. Changing a target height
  does not change already sampled body-fixed ground; body/PQS identity and
  terrain-bound changes still invalidate coverage.
- `TargetAwareAirlessPlanner` reuses checked selected-coast coverage, with one
  fresh anchor on each selected-path/policy check instead of fresh queries at
  every repeated point. Fresh selected endpoint checks remain. An observed
  change at a checked cached location invalidates the transaction and coverage;
  unobserved terrain edits cannot be detected without querying that location.
- Where actual handoff ground is sufficiently lower than the candidate's
  prospective policy ground that handoff lies above V1's supported 300 m
  terminal-replay branch, recompute that same candidate against its local
  ground. This uses the existing bounded terrain-policy feedback; it does not
  substitute a new high-altitude descent controller.
- Nominal and delayed terminal continuations keep separate resolved ground
  heights. A terminal contact against a different ground plane causes a bounded
  replay of the same existing final-descent equations before speed validity is
  decided. The numerical path may extend 1 m below the provisional plane so a
  slightly lower dynamic terrain contact can be resolved. This does not shift
  the controller's altitude input, declare that lower point a landing, or bypass
  contact/clearance/speed checks. Early coast/braking intersection remains a
  rejected candidate; a braking handoff remains an intermediate state.
- `MechJebModuleLandingPredictions` keeps the existing atomic target-aware slot,
  freshness, ordinary-result exclusion and complete-terminal-result gates.
  Its passive controller-reference metadata now reports the exact first
  trajectory UT/position that Beta actually reads, rather than the brake-search
  parameter. Search parameter, modeled braking-path start, and Beta's earliest
  five-second release must be distinguished in replay. This changes no live
  braking command or published trajectory. Scene teardown retires target work,
  releases queued results and makes late update/map callbacks inert without
  touching game state or joining a worker on the flight thread. This is not
  evidence that the earlier exit crash was caused by MechJeb.
- Capture records query-cell resolutions, maximum query displacement and the
  last rejected-candidate reason beside demanded samples, work and lineage.
  Existing readers/sequence checks recognise the approved safe-timing V1 policy
  provenance while retaining brake-reference, phase and missing-data checks.
  Recognising a provenance string is not proof of flight equivalence.

The live V1 guidance, controller, UI, warp, phase laws and actuator files are
unchanged **by this slice**. The previously approved Beta exceptions remain.
There is no new burn interlock, missing-result response, abort system or relaxed
landing classification. Terrain work remains incremental on the flight thread;
powered simulations remain on workers. Existing caps remain 32 evaluations per
terrain update, 1,536 actual queries per transaction, 32,768 evaluations and
16,384 cached locations. Coarse-cell height is not an exact height everywhere
inside the cell. Selected near-ground paths refine to 1 m; unresolved coverage,
steep terrain between samples and actual PQS cost remain explicit measurement
limitations, not assumed flat terrain or guaranteed mountain clearance.

**Offline evidence from the failed session `d68083650058444fb4fe54a159b6b998`.**
Fixtures retain four actual input snapshots (101–104) and 20,936 distinct PQS
positions/heights captured during submissions 101–115. A local inverse-distance
oracle uses only these captured samples and rejects queries more than 500 m from
coverage. The largest nearest-sample distance actually used is 159.39 m. This
oracle supports comparative regression and demanded-work measurement; it is not
the full real Mun PQS surface, and its spatial interpolation uncertainty is not
the production cache's 1 m query resolution.

| Submission / input UT | Actual terrain queries | Cache hits | Terrain updates | Incremental time at captured 0.02 s tick |
| --- | ---: | ---: | ---: | ---: |
| 101 / 24604035.618424 | 1,027 | 3,096 | 158 | 3.16 s |
| 102 / 24604040.638424 | 370 | 695 | 39 | 0.78 s |
| 103 / 24604045.658424 | 302 | 591 | 37 | 0.74 s |
| 104 / 24604050.678424 | 244 | 474 | 29 | 0.58 s |

All four produce validated nominal/delayed modeled terminal contact, horizontal
stopping-distance information and a complete compatible lineage accepted by the
existing publication gate. Every individual terrain update stays within its
32-evaluation/query bounds. These tick counts omit worker scheduling and real
PQS wall time and therefore do not certify live latency. For the first snapshot,
the same corrected predictor with unquantised queries needs 2,857 queries:
spatial reuse needs 1,027, chooses the same brake-search time, and moves the
modeled terminal endpoint by 1.730 m on this recorded oracle. The reference's
larger query allowance is offline-only; the game's budget was not increased.

Focused acceptance comprises 79 C# predictor tests and 32 Python capture,
lifecycle and sequence tests, including ownership across phases, stale/ordinary
rejection, terrain change/ridge cases, terminal validity, bounded slow-query work,
advancing-state replay and inert teardown callbacks. The historical capture's
lifecycle audit has one publication and no publication-order violations, with
submission 120 incomplete at quit. Its whole-flight audit remains a negative
observation: no horizontal-kill/final-descent/correction-settling flight evidence
exists for that attempt. It is not converted into a successful landing by the
new offline replay. Logs are retained in `MechJebLibTest/TestResults`.

**Controlled KSP validation of this build.** Use the same vessel, target and
starting orbit with passive trace/capture enabled and existing auto-warp. Record
first ballistic intersection, deorbit completion, course correction, braking
entry, horizontal-kill entry and touchdown/impact. At first intersection verify
that a committed powered touchdown forecast and blue reticle appear while
deorbit still has useful cutoff authority. Correlate its submission/version/input
UT with low-deorbit consumption and actual throttle. Record search UT separately
from the first trajectory UT consumed by Beta and its five-second release;
measure actual thrust onset, attitude gate, warp, terminal travel/clearance and
refresh age. Subsequent complete compatible work must replace atomically;
ordinary or failed work must not. A missing forecast, overlong burn, stale
retention, terrain collision, invisible marker or forecast/flight mismatch is
measurement evidence for the next narrow repair. Full landing success is not
claimed by the offline component tests or by deployment.

**Build and explicit deployment — 30 September 2026, 18:42 AEST.** The focused
79 C# / 32 Python acceptance tests passed before the separate Release build.
`MechJeb2.csproj` built successfully with zero warnings/errors and repository-only
outputs; the installed DLL's old hash remained unchanged throughout testing and
building. Exact `KSPDIR` and closed KSP processes were verified by the installer.
Only the authorised `GameData/MechJeb2/Plugins/MechJeb2.dll` was replaced.
Source and installed SHA-256:
`7DC051FED8F66960508012E2AF4841300D70B6E7505E8627FB4DCB559D04A3CD`.
The previous DLL was backed up and verified against
`40999DADBAD8E509D0CA7C97BE9538B8671A4E0300491E8129589E1099638FAE`.
Manifest: `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260930-184253-8753101\manifest.json`.
Installed `alglib.dll`, `MechJebLib.dll`, `MechJebLibBindings.dll` and
`JetBrains.Annotations.dll` retained their pre-install hashes. No companion DLL,
`C:\GameData` tree or temporary staging directory was modified. Live controller
and UI source hashes match this turn's starting state; pre-existing approved
exceptions and other uncommitted work were preserved. This is the bounded
KSP validation build described above, not evidence of a successful new landing.

### 30 September binding amendment: integrated guidance and terrain-corridor reuse

**Engineering responsibility.** User instructions state the intended outcome.
Implementation must account for the practical geometry, changing sample positions,
timing, consumers, and failure consequences needed to achieve that outcome. A
literal feature implementation or an isolated passing test is insufficient when
the component cannot perform its role in the existing landing sequence. Routine
engineering implications belong to this repair; consequential changes to the
approved live control boundary still require separate approval.

**System contract.** The predictor supplies a timely, truthful powered landing
forecast which the existing V1 guidance can consume, then refreshes from the
changed real trajectory. Its selected brake reference must be executable by the
approved V1 braking policies, including the two approved terminal fixes. Guidance
compares the forecast with the target and controls the real deorbit/correction
burns; the predictor must not erase that miss by substituting a target-reaching
trajectory. The integration chain is:

`physical snapshot -> rotation-adjusted terrain coverage -> powered simulation
-> terrain/terminal validity -> atomic publication -> V1 readiness and guidance
consumption -> existing controller commands`, with the same published forecast
feeding the map display. Useful first publication and repeated refresh are part
of correctness, not optional performance improvements. A result published after
the real burn has passed the useful cutoff does not satisfy the contract.

**Latest negative acceptance evidence.** Capture session
`d68083650058444fb4fe54a159b6b998` contains 120 submissions and one powered
publication. The first terrain-intersecting submission, 101, has input UT
24,604,035.6184 and commanded throttle 1. It exhausts the 1,536-query budget
despite 324 cache hits. Submission 116/version 17 is published at UT
24,604,111.1584, approximately 75.54 simulated seconds later. Its input already
has only 0.520 m/s surface speed at 23,504 m ASL. This is not useful validation
of deorbit prediction delivery. The map records no committed result before that
publication, then records a blue-marker request for version 17 with map/module
gates enabled. A marker request proves eligibility, not visible pixels or lack
of occlusion. The incomplete submission 120 at exit is a lifecycle observation,
not proof of the reported KSP crash cause.

**Terrain geometry and reuse.** The ballistic ground curve includes body rotation
at each propagated UT. Terrain height belongs to body-fixed surface position and
the terrain provider/configuration; advancing the vessel or changing the
simulation sampling grid does not change a previously sampled height. Retain a
spatially indexed, demand-loaded profile along that rotation-adjusted curve.
The curve describes where reusable coverage may be needed; it is not a request
to query or preload the entire curve or corridor. Query only positions needed
by the current calculation or necessary local refinement, then reuse those
values when later calculations need the same covered region. Powered candidates
and approved terminal continuation may depart from the ballistic ground curve;
represent those departures as additional covered curves or a narrow corridor.
Do not claim one ballistic curve contains every powered landing site. Real
deorbit burns and corrections can shift the required corridor, but that calls
for extending coverage and recomputing trajectories, not discarding unchanged
terrain data. A lateral adjustment is not the only possible change in coverage.

Exact latitude/longitude equality is inadequate as the primary reuse mechanism.
Use spatial quantisation and controlled interpolation where their accuracy is
adequate for the calculation, with explicit coverage, sample spacing, and
error/clearance uncertainty. Quantisation maps nearby queries to reusable
coverage; it does not make the underlying terrain flat or exact. Coarse estimates
and terrain bounds can establish adequate clearance without querying every
position. Refine locally where uncertainty can materially change the forecast,
candidate eligibility, or terminal contact; do not smooth away a consequential
ridge or silently treat an estimate as an exact measurement. Prioritise
coverage needed to complete a usable powered solution, then extend/refine for
compatible refreshes. Record which terrain samples and uncertainty supported
each published trajectory. Retaining terrain is not retaining an obsolete
prediction; every forecast still belongs to one current physical snapshot and
predictor model.

**Fidelity follows the calculation and phase.** Do not run the most detailed
terrain model for every search candidate or every deorbit refresh. A spherical
or representative-radius surface is appropriate for preliminary intersection,
coarse brake-time search and a rapid powered forecast during deorbit when its
terrain sensitivity is acceptably bounded. This still forecasts the powered V1
sequence; it does not substitute the ballistic intersection as the landing
answer. Fine terrain detail should be acquired only where it affects the
reported endpoint, executable braking, clearance, or terminal validity. A
deorbit prediction of future near-ground flight may therefore need a few local
height checks even though the vessel is currently high; phase alone is not the
only accuracy criterion.

**Mountains near the landing region.** Local target height alone cannot describe
the surrounding terrain. A ridge before the target, higher ground along braking,
or a mountain beside the approach may constrain a candidate even when its final
endpoint is clear. Use the predicted paths and their spatial uncertainty to
request local coverage of these obstacles; do not scan all mountains around the
target indiscriminately. A coarse spherical surface is inadequate where plausible
terrain relief could consume the predicted clearance or change first contact.
Refine those relevant segments before relying on their clearance, including the
remaining horizontal-kill travel. Keep ridge heights and steep slopes visible
to the calculation; interpolation across widely spaced valley samples must not
imply the ridge is absent. Do not inflate the whole body's surface to mountain
height and thereby reproduce artificially early braking. The forecast must
reflect the terrain actually relevant to its approach and landing region.

| Calculation / live phase | Terrain work required |
| --- | --- |
| Deorbit and preliminary candidate search | Use cheap spherical/representative-height geometry and existing coarse coverage first. Obtain only the local samples necessary to constrain the powered landing location and its uncertainty. Do not densely scan every hypothetical path or preload the corridor. |
| Course correction | Reuse the terrain coverage and refine where height uncertainty materially changes the landing-error information consumed by the existing correction law. Preserve consistent resolution between successive forecasts so a change in terrain fidelity is not mistaken for a real trajectory change. |
| Braking | Use more accurate dynamic terrain coverage along the relevant near-surface coast/braking path and handoff region. Add local samples where clearance or stopping capability is sensitive to terrain. Reuse previous coverage instead of restarting the terrain search. |
| Horizontal kill and final descent | Resolve the direct remaining path and local ground at the accuracy needed for contact and stopping conditions. Keep work bounded; do not restart a candidate brake-time search or query unrelated ground. |

The forecast's terrain source, resolution, uncertainty and refinement stage must
be recorded. A coarse powered landing estimate is an estimate with an error
bound, not measured exact touchdown coordinates. Use sufficient fidelity for
the existing publication/validity contract, rather than requiring exhaustive
terrain detail before every deorbit result. Conversely, do not relabel an
unsafe contact or unresolved terminal continuation as a safe landing merely
because the coarse geometry completed. If increasing fidelity materially moves
the endpoint, record the contribution of terrain-model change separately from
the real trajectory change. This staged terrain calculation changes predictor
cost and accuracy, not V1's controller, phase sequence, UI or actuator laws.

Continue querying dynamic KSP/PQS terrain for missing or insufficient coverage
and necessary selected-path checks. Do not hardwire Mun heights. Retain compatible
body-fixed samples across refreshes and phase changes; invalidate affected
coverage when body/PQS configuration or observed heights change. Target/result
lineage changes must not be confused with a change in terrain itself. Bound
storage, lookup/refinement work, and actual PQS queries separately. KSP queries
remain incremental on the flight thread; simulation remains on workers. The
existing budgets are implementation limits to measure against the moving Mun
case, not evidence that timely publication is achieved. Increasing a budget or
accepting an unresolved path is not a substitute for effective reuse.

**Integrated acceptance sequence.** Extend the existing harnesses within the
approved predictor slice; do not build another controller or replace its policy.

| Gate | Required evidence |
| --- | --- |
| Advancing-state terrain reuse | Replay consecutive captured states from the first ballistic intersection through deorbit. Shift sample positions along the rotation-adjusted curve, include powered/terminal departures and real orbit changes, and show demanded terrain is reused while necessary missing coverage is added. Assert that unused corridor regions are not queried or preloaded. Identical-snapshot repeats are a subsidiary test only. Include slope/ridge and changed-height cases; preserve uncertainty rather than treating interpolation as exact PQS. |
| Staged terrain fidelity | Compare the rapid coarse powered forecast with progressively refined local terrain on the same physical snapshot. Measure endpoint/brake-time sensitivity, query savings and uncertainty. Confirm coarse deorbit work remains useful to the existing guidance, detailed braking/terminal work finds consequential terrain, and resolution changes are distinguished from real motion. No dense search over unused candidates or unrelated ground is required. |
| Mountain approach | Include a target in a depression with an intervening ridge, raised terrain under braking/horizontal kill, and a nearby mountain outside the predicted path/uncertainty corridor. Relevant obstacles must change clearance/contact or candidate eligibility; irrelevant obstacles must not force extra queries or globally premature braking. Test adaptive refinement across steep terrain and reuse after a real correction shifts the path toward a ridge. |
| First usable powered result | Follow one snapshot through worker stages, terrain resolution, selection, publication, and `LowDeorbitBurn` consumption. Show a complete powered touchdown forecast becomes available while the existing guidance can still use its steering/cutoff feedback. Ballistic impact, braking handoff, or uncertain terminal continuation cannot satisfy this gate. Measure cold-cache latency as well as later refresh latency. |
| Refresh and ownership | Repeat through deorbit and course correction, including real pulse settling. A newer complete compatible transaction replaces atomically; ordinary immediate-braking, failed, incomplete, stale, or late work cannot replace it. A retained result does not become fresh merely because its successor failed. |
| Controller compatibility | Compare brake reference and earliest commanded braking thrust, phase transitions, policy speed/throttle/attitude gates, horizontal stopping distance, and terminal conditions with the existing V1 equations. Preserve the approved baseline and exceptions. Do not call a handoff a landing or use unmodelled response as an assumed safety margin. |
| Map integration | Correlate published version/input UT, readiness and settling gates, phase, trajectory/endpoint coordinates, and marker request. In KSP confirm the intended path and blue touchdown reticle are visible; distinguish missing publication from drawing/occlusion. Do not bypass validity to make a marker appear. |
| Responsiveness and terminal phases | Measure actual PQS calls, spatial reuse, refinement work, worker time, per-update flight-thread work, refresh age, and memory. No blocking simulation or expensive candidate search in terminal phases. Cached work must also remain bounded. |
| Predictor teardown | Exercise disable, reset, scene unload, and quit with work pending. Retired transactions must become inert; late completions cannot publish, query unloaded terrain, reopen capture, or access destroyed game state. Verify resource/capture closure and bounded pending work. This is predictor lifecycle validation, not approval for a new live abort or actuator law. |

Offline acceptance must cover the mechanics and consumer decisions that can be
established from source and recorded inputs. It must not claim real Mun terrain
equivalence from a flat or sparsely interpolated surrogate. Calibrate spatial
resolution, uncertainty and latency limits against captured terrain and the
guidance's observed useful decision window; do not invent final thresholds or
arbitrary retry counts. Where capture coverage is incomplete, identify that
specific uncertainty. Do not require complete proof of a successful in-game
landing before implementation or a bounded test build. The controlled KSP test
measures remaining game-physics, real-query performance, actuator and map
uncertainties at first intersection/deorbit cutoff, course correction, braking
entry, horizontal-kill entry, and touchdown or impact. An observed mismatch is
evidence for the next narrow repair, and remains a failed acceptance condition.

**Scope and reporting.** The initial amendment authorised documentation only;
the subsequent explicit instruction authorises the narrow implementation, build
and deployment recorded above. Source work follows the existing approved file-level plan:
terrain representation/resolution in `TargetAwareTerrainCache` and
`TargetAwareAirlessPlanner`, transaction/lifecycle integration in
`MechJebModuleLandingPredictions`, and only narrowly necessary existing
capture/harness tests. No new burn interlock, missing-result controller response,
phase law, warp gate, UI behaviour, or unrelated tool change is approved here.
Preserve the explicit Beta exceptions. Report component tests, integrated offline
evidence, build/deployment state, and observed KSP outcome separately. The current
DLL is a failed availability/performance test, not a completed predictor repair.

### Historical 30 September exact-coordinate cache attempt and deployment

This section records the earlier implementation and its offline test evidence.
Its exact-coordinate-only contract is superseded by the binding corridor-reuse
amendment above. Its deployment subsequently failed in session
`d68083650058444fb4fe54a159b6b998`; the repeated-identical-snapshot result below
did not establish adequate moving-flight performance.

The preceding failed session was `c54b4e20f21c491a824b8279e48bfc42`.
Submission 117 first resolved a ballistic terrain intersection at input UT
24,604,030.8893. During the following deorbit burn, powered submissions
repeatedly exhausted 1,536 terrain queries. Submission 127 alone published a
powered touchdown forecast at process UT 24,604,080.5093 (version 16), about
50 seconds after the first ballistic intersection. Subsequent refreshes
failed and retained that version. This establishes a publication-latency
failure; it does not establish that terrain complexity makes a powered
landing mathematically impossible. The old capture lacks the complete PQS
samples needed to replay each exhausted query. Sparse interpolated terrain
fixtures are diagnostic terrain models, not the real Mun terrain map.

**Implemented cache (historical; inadequate for the required reuse).** All ballistic, candidate, refinement, braking,
and terminal terrain lookups pass through one flight-thread cache shared
across compatible refreshes. Cache keys are exact body-fixed latitude and
longitude; height/UT/inertial position do not identify a terrain sample.
There is no coordinate rounding, terrain smoothing, corridor interpolation,
or reuse of a landing result. An unseen coordinate still queries KSP/PQS.
This preserves the current sampled-clearance calculation exactly. It does
not assume that a ballistic orbital plane describes all powered paths.

The cache retains at most 16,384 finite samples with FIFO eviction. Reset,
predictor disable, body/PQS identity change, terrain radius-bound change, or
target terrain-height change clears reuse. A final selected-coast check
queries fresh heights even where a cached sample exists. A changed sampled
height clears the cache and rejects that transaction. Exceptions and
non-finite heights are not cached. These rules assume an unchanged PQS
configuration between those checks; arbitrary unannounced terrain edits
elsewhere cannot be detected without a new query there.

Each transaction retains the existing limit of 1,536 actual PQS calls.
Cached evaluations do not consume that call budget, including nominal and
delayed terminal replays. They still consume the 32-sample-per-update and
1-ms-work-per-update limits. A separate 32,768-evaluation ceiling bounds
total cached work. A single slow PQS call can exceed the time slice; the
next call is deferred. These storage/work limits do not tune braking or
terminal control. Workers perform simulation; KSP terrain queries remain
incremental on the flight thread. No terrain cache access occurs on workers.

Publication ownership, fresh-result ordering, selected brake policy,
terminal validity, and the approved V1 guidance exceptions remain unchanged.
Failed or incomplete terrain work never publishes. No new interlock,
abort/recovery response, warp rule, phase handoff, actuator law, UI control,
or impact-as-landing route is introduced by this amendment. Ballistic
intersection remains a request to seek a powered solution, not a landing
result. The blue marker still requires a committed predicted touchdown.

**Capture and offline evidence.** Each transaction records total evaluated
samples, actual PQS calls, cache hits, and the unique body-fixed coordinates
and finite heights used (including cache hits). This closes the previous
missing-terrain-data gap for later replay. Validation summaries report
terminal touchdown/contact rather than mislabelling a completed terminal
replay as `NoForecast`/`BrakingHandoff`. The reader accepts terminal worker
stages and validates the additive cache records; older captures remain
readable.

The existing terrain-truncated terminal result has no integrated total
expended delta-v. The reader preserves that value as unknown (`null`) when
the terminal contact state is complete, rather than inventing a fuel
estimate or rejecting the entire contact record. This is an existing
diagnostic limitation and does not change the live result or V1/UI.

Map observations record result version/submission, phase, view/module gates,
the correction-settling visibility gate, outcome, and whether a blue marker
was requested. They are emitted only on result/gate changes, with logging
enabled. They do not change drawing. A requested marker may still be hidden
by camera occlusion; this observation does not claim the renderer drew
visible pixels.

The deterministic repeated-snapshot Mun test preserves the selected brake
UT and all touchdown position components exactly. It reduces PQS calls from
1,030 on the first run to 287 on the repeat, while retaining the fresh coast
check (746 cache hits). The first flat run saves only three duplicate calls;
this result must not be advertised as proof that every new, moving-vessel
refresh or the real first-intersection workload now fits the budget.
The captured-input sampled-terrain fixture also remains a powered forecast
within the budget. Tests cover bounded storage, no nearby-coordinate alias,
context/height invalidation, invalid samples, changed terrain, per-update
cached-work bounds, terminal contact, and incompatible-result rejection.

**Controlled KSP check specified for that build (subsequently failed).** Use the same Mun vessel/target with trace logging
and auto-warp enabled. Record the first ballistic intersection, first powered
publication and blue-marker request, deorbit cutoff, course-correction pulse
and settled refresh, actual braking entry, horizontal-kill entry, and
touchdown or impact. Compare cache hits/calls and publication latency with
the failed session above. If no marker appears, correlate `map_draw` with
`published` and `selection_decision`; do not infer absence of a result from
the screenshot alone. The remaining validation is whether new real-flight
PQS samples and controller motion yield timely accurate powered forecasts.
This bounded test build does not certify a successful Mun landing offline.

**Acceptance evidence for this slice:** 75 focused C# landing tests and 30
Python reader/lifecycle/sequence tests pass. The reader validates all 1,034
records and 132 submissions in the selected failed session without gaps;
its lifecycle audit confirms only submission 127/version 16 was committed
and failed refreshes did not replace it. The historical whole-flight audit
still fails: it does not recognize that DLL's newer model-provenance label
as equivalent, and the aborted flight has no observed correction-settling,
horizontal-kill or final-descent sequence. That failed flight is a negative
availability/latency fixture, not a passing flight of this terrain-cache
change. The offline gates above establish reuse, result invariance and
publication mechanics; new KSP data establishes real-flight timing and
landing performance.

**Installed test build, 30 September:** Release build succeeded with zero
warnings/errors. The game DLL hash remained unchanged throughout building.
After verifying KSP was closed and the exact authorised `KSPDIR`, the
separate installer replaced only `MechJeb2.dll`. Source and installed SHA-256:
`40999DADBAD8E509D0CA7C97BE9538B8671A4E0300491E8129589E1099638FAE`.
The original DLL was backed up and hash-verified against
`3F2D55AEB3515175697D3B2F6F19E7466C81F5EB69AACEB053BBD0A0438825BF`.
Manifest: `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260930-171247-0738656\manifest.json`.
Installed `alglib.dll`, `MechJebLib.dll`, `MechJebLibBindings.dll`, and
`JetBrains.Annotations.dll` were hash-compared before/after and unchanged.
No live V1 controller or UI source was changed in this terrain-reuse slice;
the previously approved guidance/terminal exceptions remain in the build.

### 30 September flight evidence and corrected publication test

In capture session `e7f437c03a704f5fbcb5b98e400a69a9`, submission 100 was
the first ballistic terrain intersection (`inputUT=24604031.74`). The active
planner ran powered candidates, but repeatedly changed its prospective V1
speed-policy terrain and required nearly exact agreement with the final
endpoint terrain. It rejected submission 100 after eight repropagations and
1,001 terrain queries. Submissions 101, 104, 105, and 110 rejected terminal
paths that ended above lower local ground without extending V1 final descent
to that ground. Other refreshes exhausted the 1,536-query budget. The sole
published powered result was submission 111 at UT 24604087.20, about 55 s
after the first intersection. Later failed refreshes retained that old result;
the real deorbit burn had already passed the useful cutoff. The trace confirms
one publication but does not establish why its blue reticle was not visible.
The post-crash pause is not attributable to the predictor from this capture.

These are **predictor validation failures, not proofs that no powered landing
is physically possible**. Before another deployment, a replay beginning at
submission 100 must produce a terrain-resolved powered terminal contact
within the transaction's bounded terrain work. A modelled final-descent path
that ends above lower PQS terrain must continue with the newly resolved
terrain height, then be checked again; it must not be discarded as “no
contact.” Prospective endpoint-terrain policy feedback is acceptable when
successive fully terrain-checked landing sites are within the intended
targeting tolerance and each retains terminal speed and throttle margin.
One-metre equality of policy and endpoint heights is not a meaningful
publication criterion on sloped terrain. Moving or unsafe terminal endpoints,
coast/braking terrain intersections, incomplete work, and query failures are
still not publishable landings. Retaining a stale result cannot substitute for
a timely first publication. No missing-result controller interlock is added.

## Binding V1 Beta guidance and predictor contracts — amended 30 September 2026

This is a **predictor and guidance-interface repair**, not a predictor-only
change. A selected brake UT is consumed by live V1, and changing result
availability can change V1's decisions even without editing a phase class.
The approved exceptions are separated below. The no-result burn safeguard was
explicitly rejected for this repair and is deferred; it is not an
implementation or acceptance requirement. An unapproved controller change
cannot be inferred from a predictor acceptance test. Earlier sections remain
investigation history, not authority where they differ from this section.

### Control-change boundary

| Status | Control-visible behavior |
| --- | --- |
| Approved exception to Beta | At low-orbit `PlaneChange` completion, enter `LowDeorbitBurn` instead of Beta's geometric `DeorbitBurn`. `LowDeorbitBurn` existed in Beta but was dormant on this path. Its existing steering, throttle, and cutoff equations are otherwise the baseline. The high-orbit path remains geometric. |
| Approved terminal exceptions | Horizontal-kill thrust direction opposes measured horizontal velocity. Targeted airless final descent uses the validated 0.40 braking-distance lead factor; all other modes retain Beta's 0.90 factor. These same equations are inputs to the forecast. |
| Approved prediction-input contract | The predictor owns one active target-aware result; ordinary immediate-braking work cannot overwrite it. A completed course-correction pulse makes its pre-pulse forecast unusable until the existing post-burn settling and fresh-result rules are met. `CourseCorrection` still accepts only a complete `PredictionReady` landing and keeps Beta's pulse law, thresholds, and two-result consensus. The result-availability gate is control-visible and must be regression-tested as such. |
| Explicitly deferred | No low-deorbit burn interlock, throttle hold, warp change, automatic abort, or other controller response to a missing forecast is part of this repair. Concentrate on producing a timely powered landing prediction for the existing guidance. |
| Outside this repair without separate approval | Accepting ballistic `ImpactForecast` as a landing for course correction, changing correction-pulse limits or geometry, changing `CoastToDeceleration`/`DecelerationBurn` triggers or throttle law, new abort/recovery manoeuvres, target-fixed descent, UI changes, or any other phase/actuator change. |

The predictor may change the *data* supplied to these unchanged consumers:
landing position/UT, braking-release UT, endpoint terrain height, freshness,
and availability. Those are consequential control inputs, so replay must show
each consumer's decision from the new result. A forecast is not a new command
to burn at its chosen time; V1's actual phase and policy still determine when
and how a burn occurs.

**Baseline behavior to preserve:** `PlaneChange` retains its plane-change
decision and actuator law except for its low-orbit exit destination.
`LowDeorbitBurn` retains Beta's target-range trigger, retrograde thrust,
landing-site steering, throttle calculation, and predicted-site cutoff except
without any new no-result response. `CourseCorrection` retains
its landing-error calculation, pulse magnitude/direction, attitude and
throttle gates, post-pulse settling, and two-result consensus; no
`ImpactForecast` shortcut changes its accepted result type.
`CoastToDeceleration` retains its speed/altitude entry conditions, RCS,
attitude, and warp decisions. `DecelerationBurn` retains its five-second
brake-UT guard, attitude gate, target-steering component, speed envelope, and
closed-loop throttle. `KillHorizontalVelocity` and `FinalDescent` retain their
phase transitions and throttle logic, subject only to the two listed terminal
exceptions. High-orbit, atmospheric, untargeted, staging, and abort control
retain Beta behavior, as does the V1 UI layout and button behavior. Changed
forecast values may cause these same
equations to choose different outputs; that is the intended feedback, and it
must be measured rather than described as a controller-code change.

1. **Guidance baseline.** V1 Beta commit `d8ac3dd5` is the behavioral reference
   for `LandingAutopilot` and its deorbit, correction, coast, braking,
   horizontal-kill, and final-descent steps. Restore its decision and actuator
   equations exactly. Passive tracing and read-only predictor snapshot access
   may remain only if they cannot change a decision or command. In particular,
   remove the newer impact-ready correction gate, automatic unrecoverable stop,
   target-fixed live descent policy, and altered pulse limits. Two narrowly
   approved terminal corrections are retained: horizontal-kill attitude
   opposes measured horizontal travel, and targeted airless final descent uses
   the validated 0.40 braking-distance lead factor. Other bodies and modes
   retain Beta's 0.90 factor. The predictor must use these same two policies.
2. **Prediction-driven deorbit.** The low-orbit
   `PlaneChange` handoff invokes `LowDeorbitBurn`, the prediction-driven step
   present but dormant in V1 Beta; Beta actually handed off to geometric
   `DeorbitBurn`. This is a deliberate guidance change, not a restoration of
   Beta's active path. `LowDeorbitBurn` consumes `PredictionReady` and
   `LandingSite` to steer and stop the real deorbit burn; the high-orbit
   geometric path remains Beta. The predictor must supply a usable powered
   result as soon as the ballistic path intersects terrain and refresh it as
   the real burn changes the orbit. This repair makes no change to the step's
   missing-prediction behavior, actuator commands, warp, or phase transitions.
3. **Ballistic intersection is the trigger, not the answer.** At the first
   terrain-intersecting ballistic snapshot, search brake-start times along that
   current ballistic coast. For each candidate, simulate the powered V1 coast,
   braking, horizontal-velocity kill, and final descent against dynamic terrain
   and the two approved terminal policies. A candidate brake start is usable
   only if V1 can reach and execute it through its existing phase transitions,
   five-second guard, attitude gate, speed-triggered early entry, and throttle
   law. The search must reject a nominally later time if live V1 would already
   have commanded braking thrust earlier, lacked time to orient, crossed
   terrain, or exceeded available thrust. Select from the executable
   candidates with validated terrain clearance, terminal speed, and throttle
   margin. The selected
   brake time comes from physical state and the executable V1 braking policy;
   target error is not a shortcut that moves the predicted endpoint to the
   target. Publish the safe powered touchdown location and time so V1 can
   compare it with the target and adjust the *real* deorbit burn or perform its
   existing course-correction pulses. A ballistic intersection, a virtual
   braking handoff, and an uncertain terminal continuation are intermediate
   states, never a landing solution. During a real deorbit burn, each refresh
   forecasts downstream flight *if that burn stopped at its captured state*;
   after a real trajectory change, solve again from the new physical state.
   The simulation must use the same decision equations as the live V1 braking
   step, including its attitude-gated throttle, speed envelope, early-entry
   conditions, and terminal stopping distance. Live braking target steering
   and finite attitude/throttle response must be represented to the extent
   needed to establish the claimed margin; unresolved effects are explicit
   uncertainty, not an assumed successful touchdown. The search cannot call
   a geometrically reachable endpoint "safe" before this policy equivalence
   is demonstrated.
4. **Beta result interface.** `PredictionReady` means a complete, safe,
   terrain-resolved powered touchdown (`Outcome.LANDED`), as Beta expects.
   `Trajectory.First().UT` is the selected braking-release time consumed by
   coast and deceleration; never put the current-state or ballistic-coast
   sample first. `EndPosition`/`EndUT` describe terminal terrain contact, not
   ballistic impact or braking handoff. The contact height used by Beta's live
   `DecelerationEndAltitude()` comes from the published endpoint. Resolve the
   policy/endpoint feedback to a bounded landing-site displacement before publication. If no safe candidate exists,
   or terrain/policy work is incomplete, record an explicit pending, failed,
   or infeasible diagnostic state, never `LANDED`.
5. **Active publication.** Target-aware work owns the result slot through
   deorbit, correction, coast, and braking. An ordinary immediate-braking
   result cannot replace it. Only a complete, newer, terrain-resolved,
   controller-compatible safe contact can be presented to Beta as `LANDED`.
   Keep ballistic collision, braking handoff, and uncertain terminal
   continuation diagnostic; none may be disguised as a landing to get a
   reticle or activate Beta's correction/braking gates. A failed, stale, or
   incomplete refresh cannot overwrite a committed valid result. Its failure
   state must be recorded. Retaining an obsolete landing across a changed physical
   trajectory is not readiness.
6. **Validation.** Apply the integrated acceptance sequence and corridor-reuse
   requirements in the binding amendment at the start of this document. Compare
   the full controller-bearing source diff with Beta:
   the only approved decision changes are the `PlaneChange` handoff and the
   two approved terminal corrections. Offline
   replay must start at the **first ballistic terrain intersection** and cover
   the actual deorbit cutoff, proving timely powered-solution publication and
   consumption by `LowDeorbitBurn`, a later brake reference in
   `Trajectory.First()`, endpoint and terrain-policy consistency, terminal
   stopping-distance and throttle-margin validity, and atomic publication
   under repeated refresh. At each candidate, compare the forecast's phase
   transition, brake-start UT, attitude gate, speed target, throttle command,
   and terminal-policy output with the live V1 equations on the same state;
   use shared pure policy calculations where practical without changing live
   outputs, and compare against the live phase decisions rather than testing
   the predictor against another copy of itself. Include the
   `LowDeorbitBurn`-to-`DecelerationBurn` path and the
   `CoastToDeceleration` speed/altitude early-entry paths. Record the earliest
   *actual commanded braking thrust*, which can differ from phase-entry UT;
   demonstrate that the selected later start is executable and that any
   model/actuator uncertainty fits within measured throttle and clearance
   margins. A mismatch disqualifies the candidate from `LANDED` publication.
   A pending, failed, stale, terrain-unresolved, or infeasible search must
   leave no false landing. The primary replay gate is that the powered result
   becomes available early enough for the existing deorbit controller to use.
   Replay must measure the terrain-query budget and
   refresh delay at the first intersection; the search must not block the
   flight thread.
   The captured Mun case must not be labelled safe if the retained terminal
   policies predict impact. A KSP flight is the final physics check, not a
   reason to substitute a different live controller or a different simulator.

**Implementation gate:** The selected-time search may be developed within the
approved predictor scope, but no candidate may be published as a safe
`LANDED` result on search geometry alone. The offline policy-equivalence and
transaction checks above gate active publication and a test build; the KSP
flight then tests remaining physical uncertainty. No no-result controller
response is authorized or required in this repair.

This review checks the earlier version of this document against the current V1 source, V1 Beta, `LandingGuidanceV1-Predictor-Investigation.md`, `LandingGuidanceV2.md`, `LandingGuidanceV2-Handoff-2026-09-20.md`, `LandingGuidanceV1.trace.jsonl`, and its matching `KSP.log` event. The V2 handoff is historical; the current branch is authoritative. The V2 architecture is a separate opt-in project, not an implementation path for this V1 repair.

**30 September test build and deployment (rejected by the subsequent flight).**
The controller-bearing phase files match V1 Beta except for the
prediction-driven low-deorbit handoff, the two approved terminal corrections,
and read-only predictor state access. The predictor uses the same
horizontal-kill direction and targeted-airless final descent speed policy.
Its publication gate requires terminal continuation and terrain checks, and
does not label the braking handoff a landing. In the actual flight this gate
published **no** target-aware result after ballistic contact, exposing the
unguarded `LowDeorbitBurn` no-result path.
The focused C# landing suite passed 64/64 and the Python capture, lifecycle,
and sequence suite passed 22/22. The complete C# suite passed 8,453/8,456;
the three failures were the previously observed static-conversion test and two
ascent tests, outside this repair. The Release build had zero warnings and
errors. KSP was closed, the exact authorised KSP directory was verified, and
only `MechJeb2.dll` was installed. Source and installed SHA-256 are
`AD8A3EA25172244ADF7792F46133338371AAE7CE52F9B170859C01D0CE9C07A1`.
The previous DLL and timestamped manifest are in
`C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260930-044315-0465800`.
The completed Mun attempt did not validate this build. The offline suite did
not replay the first ballistic terrain intersection or assert that
`LowDeorbitBurn` consumed a powered result before its cutoff.

### Failed Mun deorbit attempt: binding acceptance case

The newest capture session is `1db7182a1e3a4db9b85b43a0fa71b0b7` in
`LandingGuidanceV1.capture.jsonl`. It contains 127 target-aware submissions,
**zero target-aware publications**: 100 `NoBallisticTerrainPass`, 26 complete
`BallisticImpact`/`ImpactForecast` candidates discarded as
`beta_impact_diagnostic_not_active`, and one failed terminal terrain resolution
(`TerminalTerrainUnresolved:resolved=False:contact=False:queries=1536`,
submission 110). These impact records were evidence that the ballistic path
crossed Mun terrain, not usable powered landing solutions.

`PlaneChange` handed off to `LowDeorbitBurn` at UT 24,603,448.57. Its burn
reached full throttle around UT 24,604,029.36. Submission 101 first found a
ballistic terrain impact at UT 24,604,034.30, longitude 80.710°E against a
target at 23.473°E. Submission 109 at UT 24,604,074.46 projected 24.742°E;
submission 110 at UT 24,604,079.48 projected 22.582°E but exhausted its
terrain-query budget; submission 111 at UT 24,604,084.50 projected 20.278°E.
Thus the ballistic intersection swept across the target while no powered
landing result was published. The burn continued until abort at UT
24,604,149.26, with horizontal speed near zero and altitude about 23.5 km
ASL. Final projected impact was 18.35°E, roughly 17.9 km short. This is not
evidence that the two terminal corrections failed: the flight never reached
those phases.

The current `LowDeorbitBurn` defaults to full throttle; its predicted-site
steering and cutoff are gated by `PredictionReady`. Consequently no published
powered landing meant no prediction-driven cutoff. The blue reticle appearing
after abort came from the ordinary predictor, whose legacy `LANDED` outcome
had about 261.6 m/s end surface speed and about 17.86 km target error. That
unsafe ordinary result must not be mistaken for the missing active solution.

**Acceptance from this capture:** replay each submission from the first
ballistic terrain crossing through the deorbit cutoff, including the rapid
long-to-short sweep, asynchronous completions, and terrain-budget failure.
The replacement must publish a complete feasible powered touchdown early
enough for `LowDeorbitBurn` to consume it. A missing or invalid solution is a
failed predictor acceptance case, not a reason to modify the guidance here.
It must never substitute the ballistic impact or braking handoff or accept an
ordinary immediate-braking overwrite. Verify the selected brake UT, terrain
clearance, stopping
distance, throttle margin, freshness, and terminal continuation before a
`LANDED` publication. This gate precedes another KSP deployment.

### Predictor-only raised-terrain repair under offline validation

The first contact in submission 101 was at 80.710°E, where the capture
resolved 2,931.76 m terrain ASL, versus the 492.19 m target terrain used as
the first provisional speed-policy height. A deterministic replay of that
captured vessel state with a **synthetic** 2,932 m plateau beyond 40°E
reproduced the old planner's ballistic-only fallback. The same input with
flat target-height terrain produced a powered result. This isolates the
planner's failure to retry a candidate when the prospective endpoint terrain
must raise V1's speed-policy height; it does not reproduce Mun's full PQS map.

The predictor-only source change keeps terrain-intersecting braking paths as
eligible for a bounded retry when their encountered terrain is higher than
the provisional endpoint policy. It rejects coast impacts, re-runs the same
brake UT with that terrain height, resolves the entire path and actual terminal
contact again, and checks the final endpoint-policy feedback and throttle
margin before `LANDED` can publish. On the synthetic replay it produced a
terminal-validated powered result with brake reference 15 s after the first
submission, policy terrain 2,932 m ASL, and 364 terrain queries within the
existing 1,536-query cap. This predicted landing remains far long of the
target, giving existing deorbit guidance a real miss to correct. The test
establishes the missing search branch, not actual KSP terrain or landing
accuracy; the next game flight must verify those.

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

This section records the pre-flight source checkpoint. The implementation was subsequently committed as `0bce8507` and its single-DLL Release build was installed for the 2026-09-28 Mun test. The synthetic replay results below do not assert flight safety.

- `AirlessTargetAwareSimulation` copies the orbit/body/rotation/thrust inputs once and evaluates the unpowered path and forced coast-to-brake continuations on a worker. The airless deceleration end comes from the same V1 target-terrain+200 m policy on that snapshot, independent of update order in the mutable predictor fields. Its legacy-mode numerical endpoint, UT, surface speed and step count match all nine captured forced candidates; the focused candidate-177 C# replay also agrees to sub-metre precision. The actual active path interpolates to the target-height stopping radius, keeping handoff position, UT and velocity at the same event.
- `TargetAwareAirlessPlanner` resolves the first sampled unpowered terrain impact, evaluates nine coarse brake times, refines a signed downrange bracket and logs crossrange independently. A distant coarse candidate may intersect a ridge while still supplying a geometric sign for the search; publication requires the final local bracket and selected powered path to be terrain clear. The selected coast is rechecked before commitment when it enters the terrain envelope. Synthetic ridge tests verify that an unsafe distant candidate cannot conceal a separate clear root, while a ridge on the selected branch or coast fails the transaction. Above-ground handoffs are valid terrain states. The terminal check currently proves only a **necessary, optimistic vertical stopping bound** at V1's actual terrain+205 m transition. It cannot prove attitude response, throttle/hover behaviour, fuel, or touchdown; these remain validation gates.
- A single target-aware transaction owns the published result in targeted airless mode. The ordinary queue and old parallel planner queue cannot publish there, including at the final ordinary publication write. A complete newer transaction passes generation, target/body/terrain, snapshot age, sequence, clearance and terminal gates before swapping the V1-compatible result and incrementing `ResultVersion`. Failure, stale work, cancellation and asynchronous out-of-order completion leave the prior committed result in place for the **same** target and terrain. A changed target/body or changed target terrain invalidates that old commitment; V1 then sees its existing no-prediction state until a new transaction completes. The autopilot phase classes and UI source are unchanged.
- The passive capture now records the **same active snapshot** used by the worker, including target query timing and PQS terrain bounds. It records refresh ownership, worker-stage completions, final clearance/handoff validation and publication lineage separately from legacy `terrainContactConfirmed`. The reader accepts these version-1 optional records while preserving strict validation of older captures. `tools/landing_predictor_lifecycle_harness.py --require-active <capture>` audits the emitted event order, phase observations, committed predecessor, version sequence, ordinary-result exclusion and invalidation. Its synthetic tests cover a phase change, repeated refresh, late abandoned worker, terrain invalidation, braking/hover phase labels and an attempted ordinary overwrite. The original Mun capture has zero new-model publications, so it cannot pass `--require-active`. `LANDED` remains the compatibility outcome for V1 `PredictionReady`, not proof of touchdown.

The captured Mun target terrain was exactly 492.187665 m ASL on all 712 submissions, so the current equality gate has no observed terrain jitter in this case. The 340 measured old-predictor terrain resolutions had median 0.101 ms for 15 queries; target-height queries across the 712 submissions had median 0.0092 ms and maximum 0.0309 ms. On a **flat synthetic terrain oracle** using the captured Mun dynamics, the prototype made 387 ballistic, then 402 coarse, then 29 final-bracket/selected refinement terrain queries (818 total). It found 36.14 m signed downrange error and 164.33 m crossrange error, with a 0.258 s timing interval and an optimistic 172.42 m vertical stopping distance at the 205 m V1 transition. These are geometry and synthetic-terrain results, **not** actual Mun terrain validation. The per-flight-update cap is provisionally 32 queries and 1 ms; even 32 times the slowest captured target-height query is about 0.99 ms, while the explicit timer stops a batch sooner if path queries cost more. The total cap is 1024. Measured active PQS time, warp-dependent snapshot age and final numerical limits still need a live capture before a KSP DLL test.

The Python replay independently reproduces all nine historical forced endpoints to sub-millimetre position and microsecond UT precision. Its spherical signed-root estimate is about −46 m downrange and +164 m crossrange. It deliberately labels the refined path `terrain_unresolved`, because the prior capture contains only sparse terrain samples for candidate 177 and no PQS oracle for the new root; the nearest recorded terrain point to that refined endpoint is about 951 m away. Candidate 177's 41.785 m sampled handoff clearance is accepted as a terrain state; its roughly 58.4 m/s downward endpoint speed is not treated as terminal-safe. The current source prototype rejects any candidate whose **sampled** path collides before handoff and retains the prior committed result when terrain or terminal feasibility is unknown. Samples one second apart cannot establish continuous clearance across an unmeasured sharp ridge; final path spacing and any required clearance margin still need calibration with actual Mun PQS data.

**Pre-flight gates:** obtain actual Mun terrain and controller-state evidence for the refined path and V1's braking-to-hover-to-final-descent transition; verify the target-aware capture/reader on repeated refreshes and DeorbitBurn→CourseCorrection, plus abort, invalidation, terrain changes and late worker completion; measure active PQS and worker time at V1 warp rates, then calibrate downrange, crossrange, timing, clearance, age and performance limits. The source-only focused predictor tests and build passed. The full repository test run had three failures in unrelated static/ascent tests (8,416 pass, three fail); those failures had not been attributed to this repair at the checkpoint.

## 2026-09-28 active Mun flight: stale publication across the deorbit burn

The first installed predictor flight is capture session `7cba28f30a2c459aab48627d2957d974` in `LandingGuidanceV1.capture.jsonl` (lines 3764–5221). The strict reader found 230 target-aware submissions and zero record gaps. Only submission **159** completed and published, at UT **24,603,457.768**, result version **24**, during `DeorbitBurn`. Its input orbit was sampled at UT 24,603,457.428 while the deorbit engine was still firing. The planned virtual brake began at UT **24,603,997.016**, 13,383 m ASL, and ended near the target (about 160 m displayed error) at UT 24,604,189.254. The resolved endpoint terrain was **486.916 m ASL** with only **5.272 m** sampled handoff clearance. The positive clearance and optimistic terminal bound validated that original snapshot; they did not validate the trajectory after the live deorbit burn changed it.

The next submission, **160**, sampled the changed orbit at UT 24,603,462.428 and completed its failed refinement by UT 24,603,462.688, before the controller's `CourseCorrection` step appeared. Its unpowered terrain crossing was at longitude **20.270° E**, roughly **11.2 km short** of target longitude 23.473° E. All nine replayed coarse brake timings ended short; even the latest reached only longitude **22.481° E** at high speed. `NoSignedDownrangeBracket` was therefore a genuine no-solution state for brake-time adjustment on that orbit, not merely missing terrain data. The original unpowered contact for submission 159 had been at longitude **58.102° E**. The ongoing deorbit burn moved the ballistic crossing from far long to short after version 24 was committed. V1 briefly entered `CourseCorrection` and immediately went to `DecelerationBurn` because version 24 still reported a near-target endpoint; the current-orbit miss never became control-facing.

All **71** later refreshes failed: 10 `NoSignedDownrangeBracket`, 39 `SnapshotExpiredBeforePublication` at high warp, 20 `TerrainQueryBudgetExceeded` at the 1,024-query provisional cap, and two `NoBrakingWindow` exceptions near impact. The committed version remained **24** through touchdown, about **623 s** after its input snapshot. The lifecycle harness reports no ownership overwrite, but its current `--require-active` pass is insufficient: it proves that no ordinary result replaced 159 and does not detect that no usable refresh followed an orbit-changing burn or that a committed result stayed authoritative for hundreds of seconds. Add a committed-result freshness/orbit-change and phase-consumption gate to that harness before accepting another flight.

The displayed powered path starts at the predicted brake point because `Result.Trajectory` contains only the powered segment, not the preceding coast. It was also tied to the old orbit: at the planned brake UT the live vessel was **5,409 m ASL**, almost **8 km below** the predicted 13,383 m start. V1 began commanding **full throttle at UT 24,603,992.05**, five seconds before the simulated brake UT, when `DecelerationBurn` left its five-second coast guard; it was still at full throttle until roughly UT 24,604,030. Full throttle is the existing speed-envelope controller's response to being above its target speed, not an independently selected predictor throttle. Both the stale orbital path and the controller's five-second early throttle are model/control mismatches that shorten the flight relative to the published result.

Terrain was queried for the chosen old path. The actual path reached terrain near **1.180° N, 15.289° E**, about **28.6 km short**, where ordinary background results measured local terrain near **1,062 m ASL**. The committed endpoint terrain remained 486.916 m ASL. V1's airless `DecelerationBurn` handoff threshold was therefore about **692 m ASL**, below the actual ground. The last state trace at UT 24,604,080.97 still showed `DecelerationBurn`, altitude 1,073 m ASL, descent about 45.6 m/s and throttle 0.519; ground contact followed near UT 24,604,081. The flight did not reach `KillHorizontalVelocity` or `FinalDescent`. The screenshot's 28.59 km miss and survivable crash agree with this trace.

**Finding before design amendment:** a no-bracket refresh after a trajectory-changing burn cannot silently leave an older near-target result authoritative through a phase decision. A larger query cap or denser terrain sampling alone cannot fix this flight. The proposal to publish a closest-reachable virtual braking result is superseded by the controller-equivalence requirement below: a geometrically complete virtual trajectory is not a valid V1 active prediction unless it represents V1's actual control sequence.

## Controller-equivalence amendment — policy-sharing route approved

**Decision and scope.** Commit `0bce8507` is rejected as an active targeted-airless predictor repair. `AirlessTargetAwareSimulation.Run` and `TargetAwareAirlessPlanner` currently coast to a chosen brake UT, apply their own continuous retrograde speed-envelope braking, and publish that path through `BuildTargetAwareResult`. V1 does not execute those virtual controls. This amendment supersedes the earlier active-model and readiness claims in this review. No new DLL may be installed or KSP test requested until an offline controller-equivalence gate passes. V1's geometric deorbit, course-correction pulses and settling, coast/warp, attitude-gated deceleration throttle, horizontal-velocity kill, and final descent remain the control authority and keep their existing behavior.

### Predictor model and phase ownership

An active `LANDED` result must be a **controller-equivalent forecast**: its trajectory, brake start, endpoint, and terminal handoff are generated from one immutable snapshot and the actual V1 phase/control laws and state, including throttle limits, attitude gate, pulse history, warp transitions, mass/thrust evolution, and phase transitions. The active producer may not select a brake time or virtual throttle law that V1 has not selected. A ballistic impact or an independent virtual-burn endpoint may be recorded as a diagnostic conditional forecast, with its assumed control horizon and signed downrange/crossrange miss, but it is not silently promoted to V1's active landing result. If future V1 commands, attitude response, staging, or terrain are not reconstructible, the forecast ends at that uncertainty boundary and records `controller_equivalence_unresolved`; it must not invent a landing endpoint or publish `LANDED`.

| V1 phase | Active-prediction rule | Work allowed |
| --- | --- | --- |
| `PlaneChange`, `DeorbitBurn` | Do not commit a post-burn landing from an orbit sampled during an unfinished burn. Identify the actual completed-burn state before any result can be authoritative across the phase transition. | Bounded passive snapshot and conditional ballistic diagnostic. |
| `CourseCorrection` | Account for the existing finite pulse, post-burn 0.75 s settling and two-result direction agreement. Pre-pulse, in-pulse and settled snapshots are distinct; a pre-pulse landing endpoint cannot remain authoritative after the orbit changes. | Controller-equivalent forecast only when pulse state and subsequent control are represented; otherwise explicit uncertainty. |
| `CoastToDeceleration` | Prediction brake start and `Trajectory.First().UT` must match the V1 coast/warp and deceleration trigger actually exercised. A five-second discrepancy, as in this flight, fails equivalence. | Bounded current-state forecast and lineage check. |
| `DecelerationBurn`, `KillHorizontalVelocity`, `FinalDescent` | No terrain candidate search or forced-brake refinement. A previously committed result must not become a stale plan merely because work fails. Terminal outcome is checked against actual phase, attitude gate, commanded throttle, local terrain and vessel clearance. | Passive observation and bounded terrain/lineage checks only. |

**Publication and failure contract.** Record target/body/terrain identity, generation, input UT, phase/control-state revision, model provenance, horizon, validation, and result version. A complete newer controller-equivalent result may atomically replace an older result from the same landing and compatible phase lineage. An ordinary immediate-braking result, the `0bce8507` virtual model, and a failed/incomplete/terrain-unresolved candidate have no publication route into this slot. A failed candidate does not overwrite a still-valid commitment. Independently, an orbit-changing V1 burn or phase transition can make a prior commitment invalid; the current code has no safe general invalidation path because `DecelerationBurn` dereferences `Prediction.Trajectory` and `CourseCorrection`/`KillHorizontalVelocity` can retain earlier commands on `!PredictionReady`. With V1 controller behavior frozen, **safe handling of that invalidation is an unresolved design gate**, not permission to keep a demonstrably false result or to publish a substitute virtual path. The active predictor must stay disabled where that gate cannot be met.

**Terrain.** Query current KSP/PQS terrain on the flight thread at bounded, measured cadence. Record target terrain, each queried local terrain sample, minimum path clearance, early coast/braking intersection, handoff clearance, and terminal-descent requirement. Reject an early intersection or a below-terrain handoff. Above-ground handoff is valid when V1 terminal control is demonstrably able to handle its clearance and speed. No mandatory touchdown and no hardwired Mun height are inferred from the simulator's `LANDED` label. An unqueried interval remains uncertain.

### Offline whole-sequence gate

Join the existing capture session `7cba28f30a2c459aab48627d2957d974`, `LandingGuidanceV1.trace.jsonl`, and the matching `KSP.log` state rows by UT and phase; preserve their different sample rates and report missing observations. The harness must consume production submission/publication records and compare each active result's brake UT, first trajectory UT, endpoint, model provenance and age with actual V1 phase, commanded and achieved throttle, attitude error/gate, correction pulse and settling state, warp rate, local terrain and clearance. It must fail this flight on the published 13,383 m versus actual 5,409 m brake-start altitude, the five-second early full-throttle command, 71 failed refreshes, 623 s retained input age, and the actual higher terrain/intersection before terminal handoff. The existing `--require-active` lifecycle pass is not an equivalence pass.

Focused synthetic fixtures must exercise repeated refresh, deorbit-to-correction, pre-/in-/post-pulse states and consensus, coast/warp, braking throttle and attitude gates, hover/final descent, abort, target/terrain changes, failed and late worker completion, and pooled result release. Require zero incompatible-model publications, zero terminal-phase candidate searches, no failed terrain replacement, no stale-lock after orbital change, and a complete controller/terrain lineage for every active result. Compare V1 command outputs before/after any implementation at the same captured states; a changed command or a missing phase observation fails the gate. Calibrate accuracy, age, and time/query thresholds from the replay rather than inventing final values. The present flight supplies no `KillHorizontalVelocity` or `FinalDescent` observation, so their production equivalence cannot be claimed from this capture; synthetic checks must be labelled as such.

### Abort Autoland visibility

The capture records `landing_stopped` and `landing_module_disabled` at UT **24,604,081.065**. `MechJebModuleLandingGuidance` draws **Abort Autoland** only while `Core.Landing.Enabled`; after automatic stop it instead draws disabled landing-start buttons, while the separate **Show landing predictions** toggle can remain on. The screenshot therefore shows an already stopped autopilot with prediction display still visible. The requested UI correction is to keep the Abort Autoland control visible after touchdown. When clicked, it must call the existing stop path if active and release this window's prediction-display user if still enabled, so the visible guidance can be dismissed; it must not introduce a new flight-control response. Test active abort, automatic touchdown, post-touchdown click, and manual prediction use separately.

**Approved implementation route and gate.** The user approved extracting existing V1 controller decisions into shared pure policy functions used by both V1 and the predictor, with captured-state tests proving unchanged V1 command outputs. The policy implementation must include the actual phase gates, pulse state, coast/warp trigger, attitude-gated deceleration throttle, and terminal handoff; a partial policy cannot be promoted as a complete active model. Any future input the shared policy cannot reproduce ends the forecast with explicit uncertainty. Neither the current independent virtual brake model nor ordinary immediate-braking results may publish to the targeted-airless active slot. Remove that publication route first, then implement shared policy and the offline equivalence harness. A safe invalidation rule after an orbit-changing burn remains a required gate: until the controller-equivalent result is complete, do not claim predictor completion or install a new DLL. Add the UI visibility correction, run focused regression tests and the offline harness, build, commit, and push only after the accepted active path is complete. Do not install a DLL or request another KSP flight until those offline checks pass. The currently installed `0bce8507` DLL remains an unaccepted test build; this amendment does not modify game files.

### 2026-09-28 implementation checkpoint (not deployable)

The independent virtual planner's result-construction/publication route has been removed from the working source. A completed run now records `ControllerEquivalenceUnresolved` and is abandoned; ordinary results remain excluded from the targeted-airless slot. Terminal-phase transaction starts are blocked, and an in-flight planner is cancelled on entry to `DecelerationBurn`, `KillHorizontalVelocity`, or `FinalDescent`. Consequently this working tree does **not** yet provide a usable targeted-airless V1 prediction and must not be installed. It is a safety boundary while the controller-equivalent forecast is built.

`V1LandingControlPolicy` now contains the exact current V1 geometric-deorbit trigger/throttle, post-pulse settling, pulse magnitude limit, coast/brake trigger, braking attitude gate and throttle calculation, hover throttle, and final-descent speed bound. The live V1 steps call these extracted formulas. This proves that the source rules can be shared; it does **not** prove a future trajectory. Future position, mass, achieved thrust, attitude, controller pulse state, and dynamic terrain clearance must be propagated or bounded before those rules can predict commands ahead of the current snapshot. The old passive capture does not contain all of these future inputs, and its completed flight never entered the two terminal phases. A future active result must expose its model provenance and uncertainty boundary in capture data; the current virtual planner cannot simply acquire that provenance label.

`Trajectory.First().UT` had two incompatible meanings: it was both the first map-path sample and V1's brake reference, from which `DecelerationBurn` subtracts five seconds. A complete forecast must draw its actual earlier coast/burn path without moving the control trigger. `Result.BrakeReferenceUT` now returns a separate explicit reference when supplied by a future controller-equivalent result and otherwise returns the original first-trajectory UT for every legacy result. Both existing V1 readers use it. The optional capture fields `modelProvenance`, `controllerBrakeReferenceUT`, and `controllerBrakeReferencePosition` preserve that distinction for the offline gate. The virtual diagnostic planner sets no explicit control reference. Exact legacy timing is covered by a focused regression test; no new accepted active path yet exercises the explicit reference.

### 2026-09-28 diagnostic flight exception

For one data-gathering KSP build, Dave explicitly selected the previous experimental predictor as the active model. This temporarily restores `BuildTargetAwareResult` publication with submission provenance `virtual_forced_brake_diagnostic`; it **does not** accept that model as a V1 controller-equivalent repair or relax the offline equivalence gate for the eventual repair. The build label states that it is a virtual-predictor diagnostic. Its landing prediction can be wrong, as the completed Mun flight demonstrated, and the sequence harness must report controller-equivalence failure even if result ownership passes.

The diagnostic retains the ban on new expensive terrain-candidate searches in `DecelerationBurn`, `KillHorizontalVelocity`, and `FinalDescent`. Existing trace sampling additionally records vessel orientation, thrust-controller mode and setpoint, mass, thrust capability, local gravity, terrain-relative altitudes, and one event for each observed phase transition. These are passive reads of existing V1 state, with no new simulation or terrain query. The event record lets replay distinguish a very short `CourseCorrection` phase from no transition; it cannot prove a correction pulse occurred when the phase lasted less than a sample interval. New capture submissions identify the virtual model. The installed DLL, if any, remains a diagnostic instrument until flight data are compared with actual V1 commands and terrain.

### Diagnostic Mun flight, session `a00c65cee6954d7c998d04bac5d940da`

The second flight confirms that the diagnostic model is not a usable V1 landing prediction. Of 158 target-aware submissions, the first 157 failed `NoBallisticTerrainPass`. Submission 158 alone published result version 57 at UT 24,603,460.089 from a DeorbitBurn snapshot at UT 24,603,459.609. Its capture explicitly identifies `virtual_forced_brake_diagnostic`; its virtual trajectory begins at brake UT 24,604,049.954, 4,190.65 m ASL, and ends at a predicted site with 485.50 m local terrain. It contains no controller-equivalent brake reference or preceding coast trajectory. No newer target-aware result was published. V1 entered `CourseCorrection` at UT 24,603,463.37 and `DecelerationBurn` at UT 24,603,463.39. That 0.02-second correction phase had no sampled pulse/settling state. Terminal-phase search suppression then prevented further target-aware submissions, while version 57 remained authoritative through the end of the flight, roughly 605 seconds after its input snapshot.

The vessel was only 2,196.6 m ASL at the predicted brake UT, 1,994.0 m below the virtual trajectory's starting altitude. V1 commanded full throttle at UT 24,604,044.99, 4.96 seconds before the virtual brake time, using its existing five-second trigger and closed-loop speed policy. The deorbit orbit continued changing after the snapshot: logged periapsis altitude went from −13,669 m at UT 24,603,459.01 to −19,806 m at UT 24,603,463.09. The virtual result neither forecasts that remaining controller action nor executes V1's later attitude-gated throttle. Its near-target endpoint was therefore a conditional path that V1 did not fly, yet V1 consumed it as a committed `LANDED` result.

Terrain clearance was valid only along the old virtual path. Near the actual ground, the last guidance sample at UT 24,604,064.55 was still in `DecelerationBurn`: 1,410.75 m ASL, 12.46 m above local terrain, 7.84 m bottom clearance, full throttle, and result version 57. KSP's state log measured approximately 397.87 m/s surface speed, 394.64 m/s horizontal speed and −50.54 m/s vertical speed at that instant. Local ground was therefore about 1,398 m ASL, versus 485.50 m at the old predicted endpoint. V1's handoff threshold based on that endpoint was about 685.5 m ASL, already below the actual ground. Neither `KillHorizontalVelocity` nor `FinalDescent` was reached. The offline sequence harness rejects this flight for non-equivalent model provenance, throttle before the predicted brake time, and terrain above the predicted handoff; it also marks terminal-phase observations and correction pulses unresolved. This evidence reinforces the controller-equivalent prediction requirement and the need to invalidate or explicitly bound the authority of an aging result after an orbit-changing action, without adding a new V1 controller response.

### 2026-09-29 controller-policy predictor candidate

The previous virtual speed limiter no longer has a publication path: production snapshots carry explicit V1 control-model inputs and publication rejects an output without that model. The `RunV1Control` forecast propagates the copied Mun state with gravity and recorded body rotation, coasts to V1's `BrakeReferenceUT − 5 s` release, waits for V1's speed-policy trigger, and uses the same `SafeDescentMaximumSpeed` and `BrakingThrottle` functions as the live controller. It evolves thrust acceleration from captured mass, engine thrust and mass flow. It stops at V1's terminal handoff, not physical touchdown. The previously observed full-throttle command five seconds before the old virtual burn is thus part of this forecast. A near-term attitude gate that is not aligned causes candidate failure; future alignment during a long coast remains an explicit prediction assumption, to be checked by the KSP trace.

Heavy brake-time and terrain candidate searches start in `CourseCorrection` or `CoastToDeceleration`, after geometric deorbit has completed. `DeorbitBurn` no longer publishes a target-aware endpoint that the remaining deorbit burn can invalidate. `CourseCorrection` already waits when there is no valid prediction. A bracketed signed-downrange root is refined as before, with crossrange independent. If no brake-time root exists, the planner validates the nearest reachable controller forecast and publishes its **actual miss** so V1 can make its existing correction pulses; it tries the next candidate if the nearest path is terrain-unsafe. A selected path is checked against KSP PQS. When local terrain differs from the modelled V1 speed-policy terrain, one worker repropagates the same brake reference with the newly resolved controller handoff height, then the result is terrain-checked again. In `DecelerationBurn`, a single live-state forecast refreshes about every two seconds using bounded terrain queries; there is no multi-candidate search. No planning starts in `KillHorizontalVelocity` or `FinalDescent`. Target-aware generations, sequence, atomic replacement and ordinary-result exclusion remain in force. Failed/incomplete work retains the last committed result and records why it failed. The UI's landing/Abort buttons are restored to their original active-state behavior.

**Offline evidence before this candidate's KSP flight:** the independent Python V1-control propagation and the C# worker agree on the captured Mun candidate-177 brake start (reference minus five seconds), handoff UT, endpoint and surface speed. The C# flat-terrain Mun fixture refines to 43.1 m signed downrange and 164.3 m crossrange with 669 measured terrain queries, under the 1,024-query cap; these are not actual PQS terrain values. The no-root fixture publishes a validated nonzero miss, and a raised-terrain fixture repropagates the live braking forecast to a 205 m local-terrain handoff. Focused tests also cover reader completeness, terminal-phase search exclusion, phase sequence, failed/late refresh ownership, and rejected ordinary overwrites. The previous flight still fails the whole-sequence audit, as it should. The first new KSP flight must establish actual post-deorbit snapshot geometry, commanded throttle/attitude, measured terrain clearance, live refresh lineage, and `KillHorizontalVelocity`/`FinalDescent` behavior. The source and offline checks justify a test DLL, not a claim that the in-game target accuracy is already proven.

The optional passive submission snapshot now also records the **current** surface/orbital velocity, orientation vectors, gravity, achieved/available thrust acceleration, speed, altitude, step duration, attitude error, angular speed, commanded throttle, warp setting, RCS setting, touchdown-speed setting, and active result version/input UT. It is sampled only when a predictor submission is captured, and the reader rejects a partial new-format controller snapshot while accepting the old Mun file. This does not supply future engine mass flow, engine/staging changes, attitude response, correction-pulse private state, terrain along an unflown future path, or terminal-phase observations. Those missing data cannot be inferred solely from the controller's source equations; they must be modeled with validated bounds or captured in a later passive run. No new DLL has been installed to gather them.

`tools/landing_predictor_sequence_harness.py` joins the existing capture, JSONL guidance trace and KSP state log by session lineage and UT, and fails the completed Mun flight. It measures a predicted brake start at **13,383.3 m ASL** versus **5,408.7 m ASL** actually observed at that UT, V1's first full-throttle command **4.966 s before** the predicted brake UT, **71** failed post-commit refreshes, and maximum observed input age **623.54 s**. It flags terminal-phase candidate searches and missing controller-equivalent provenance. Terminal ordinary terrain results near actual contact measured **1,062.18–1,062.25 m ASL** versus the committed old endpoint terrain **486.92 m ASL**. V1's predicted terminal threshold was **691.92 m ASL**, below that observed local terrain; the harness flags this conflict. It explicitly reports absent sampled `CourseCorrection` pulse/settling state and absent `KillHorizontalVelocity`/`FinalDescent` flight observations. A malformed trace tail row is also recorded as missing evidence. These are rejection findings, not calibrated accuracy thresholds.

The next active-model step must propagate the V1 controller's own commands and dynamic state through a bounded forecast, then compare those commands at captured states and the forecasted path against the entire observed sequence. It must not publish while a deorbit or correction burn changes the input orbit, after stale lineage, or when attitude/thrust/terrain uncertainty prevents a complete result. Current V1 has no safe general no-result transition after `CourseCorrection`; solving that without changing V1 command behavior remains an explicit design gate. Until that and the missing dynamic inputs are resolved, the working build is a source checkpoint only, and the implementation/push/DLL gates remain closed.

Source alone supplies the decision formulas, not their future operands. `CourseCorrection` calculates each pulse from the currently published endpoint and end UT, then waits for two independent post-pulse predictions; forecasting future pulses is therefore a feedback problem, not a one-pass burn schedule. `DecelerationBurn` uses the vessel's future forward vector, gravity, thrust acceleration and speed policy every update; the thrust and attitude controllers and changing propellant mass determine the state those calculations receive. `FinalDescent` delegates to thrust-controller modes and re-queries terrain near the estimated landing position. The existing Mun capture and 10 Hz logs are enough to reject the old model and check several decision boundaries; they omit full forward/thrust/fuel/pulse state and contain no actual terminal phases, so they cannot certify an active whole-sequence replacement. The added passive snapshot fields will be available only after a later instrumented run. Under the present no-install-before-offline-gate rule, the remaining flight-state evidence is unavailable; a source-only synthetic test can verify shared formulas but cannot be presented as a Mun controller-equivalence pass.

### 2026-09-29 test-build decision and corrected control inputs

Dave subsequently authorized implementation, build, and a single-DLL deployment for another KSP test. This supersedes the prior no-install hold **for this test build only**; it does not turn the synthetic replay into proof of an accurate Mun landing. The old virtual speed-envelope path is excluded at publication. The active forecast starts from the post-deorbit orbit at the same UT as its mass/thrust/policy snapshot, uses V1's speed-trigger and brake-reference-minus-five-seconds guard, shared braking throttle law, evolving engine mass flow, and captured minimum, maximum, and smoothing throttle settings. A live braking refresh is allowed only after V1's burn has actually triggered and only when the observed near-retrograde attitude passes a conservative gate. It uses one continuation, with no terminal-phase brake-time search. A correction-pulse snapshot is rejected if the pulse began after its input UT or its post-burn 0.75-second settling interval has not elapsed. These are predictor ownership checks; V1 commands and UI state are unchanged.

The trajectory includes the coast from the snapshot and the V1-controlled braking segment. The separate brake reference preserves V1's control timing. Dynamic KSP PQS samples validate clearance along the selected path and repropagate the speed policy if the predicted local terrain height changes. The terminal result remains an above-ground handoff with a necessary stopping-distance check, not a simulated touchdown. Ordinary immediate-braking results cannot publish in targeted airless mode, and failed or incomplete refreshes do not replace a committed result. The old result can still age during repeated failures; that age is logged and is an explicit remaining flight-validation issue.

The source-only gates cover: the captured Mun numerical propagation in both C# and independent Python; reachable and unreachable brake-time roots; separated crossrange; raised local terrain revalidation; unresolved live attitude rejection; selected-path terrain failure; ordinary-result exclusion; asynchronous/late results; repeated refresh/phase lineage; and rejection of the two failed historical flights by the whole-sequence audit. The first test flight must compare actual burn trigger, applied throttle, attitude/correction, handoff height/speed, terrain clearance, and successive result input UTs. Future attitude convergence, changing throttle limits or engine configuration, continuous V1 steering corrections, and the actual `KillHorizontalVelocity`/`FinalDescent` response remain measured uncertainties. The predictor should not be described as fully controller-equivalent until the new capture demonstrates those phases and an accurate landing.

The Release build completed with zero warnings and errors. All 43 focused landing tests and 21 Python replay/capture/lifecycle/sequence tests passed; the existing Mun capture parsed as 6,176 records, 1,100 submissions, zero gaps. The full .NET suite had 8,432 passes and three failures in `StaticTests.ToSITest` and two `PSGTests.AscentTests.KerbinTests` cases. Their test and implementation files were not changed by this repair; the failures are recorded and not claimed to have been resolved. The KSP single-DLL install used a closed-game check and a timestamped manifest backup at `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260929-010913-4382190\manifest.json`. Source and installed `MechJeb2.dll` SHA-256 both equal `D775BAE57DE6284EA8D01BF68EB902065D104D04E437B8C1F4B4C4F387833293`; the backed-up previous DLL is `EAF7464594231BE49D8117DA08702EE90694BF95764420616C86237EF43D817D`. No companion DLL was changed.

### 2026-09-29 failed Course Correction flight and repair

Session `f7c7e7e4154c40f9a775bc16fb6c55ab` contains 24 target-aware submissions after the DeorbitBurn to CourseCorrection transition at UT 24,603,463.769. No target-aware endpoint is published during the moving deorbit orbit; the first post-deorbit submission failed `ControllerPolicyTerrainNotConverged`, delaying the reticle. Submission 2 alone published version 9 from input UT 24,603,468.789. It was an explicitly unbracketed V1-policy fallback approximately 32.3 km short of target (endpoint longitude 14.234° E versus target 23.473° E). Its coast began from the vessel snapshot, braking began at UT 24,603,983.664, and its terminal handoff was UT 24,604,087.699. The map drew coast and powered flight in one red path, making the apparent burn much longer than the predicted powered interval.

V1 fired one limited CourseCorrection pulse around UT 24,603,480.609, then waited for two new settled predictions beyond version 9. None published: 13 refreshes failed `ControllerPolicyTerrainNotConverged`, and 10 failed in refinement with `RefinedCandidateTimedOut`. The latest submitted snapshot, number 24 at UT 24,603,579.229, had an unpowered terrain contact near longitude 23.917° E after the pulse, but a near-target delayed brake would have handed over at hundreds of m/s. That is not a safe landing solution on the current orbit. Its capture ends without worker completion when KSP exits; the reader reports this one gap explicitly. The whole-sequence audit flags no incompatible publication in this session, but cannot verify a later burn or terminal landing because the flight was aborted.

Two defects kept Course Correction from advancing. First, live V1's airless speed-policy height came from the previous prediction's local endpoint terrain, while the forecast began at target terrain and repropagated at each newly predicted local height. On rugged Mun terrain, this feedback failed to converge and discarded refreshes. The targeted airless V1 policy now uses the selected target's terrain height consistently; KSP PQS at each candidate remains a separate coast/braking clearance and terminal-handoff check. Other landing modes retain their existing policy. Second, the forecast evaluated the next-step speed envelope below its own terminal handoff radius, producing an invalid square root and false timeout. Clamping only that lookahead query to the handoff radius permits the step to finish and the actual crossing to be interpolated. Unsafe high-speed roots still fail terminal validation. When a root or refinement fails, the planner tries the nearest complete, terrain-clear, terminal-safe coarse forecast and publishes its real miss, allowing another existing V1 correction pulse. Failed or incomplete work never replaces a committed result. The map now draws the coast in cyan and powered segment in red.

The captured submission-24 state is a regression fixture: under a flat target-height terrain oracle the repaired planner completes with a roughly 14.3 km signed miss, 454 bounded terrain queries, and a valid terminal control bound. That oracle cannot certify the actual Mun PQS path; the next flight must test real clearance and subsequent correction publications. All 44 focused C# landing tests and 21 Python tests passed. The full suite had 8,433 passes and the same three unrelated failures (`StaticTests.ToSITest`, two `PSGTests.AscentTests.KerbinTests` cases). The build succeeded with zero warnings or errors. This is a test-build repair, not proof of an accurate KSP landing.

The 2026-09-29 03:17 AEST test DLL was installed only at the authorized `GameData\MechJeb2\Plugins\MechJeb2.dll` path after confirming KSP was closed. The backup and manifest are at `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260929-031742-4400010\manifest.json`. Source and installed DLLs both hash to SHA-256 `2E04C24E3C70356E016E95EEA4D6AA3C2BCF36CC02A9B8E728710B41F84B91BC`; the backed-up prior DLL hashes to `D775BAE57DE6284EA8D01BF68EB902065D104D04E437B8C1F4B4C4F387833293`. No companion DLL was replaced.

### 2026-09-29 no-reticle flight and terrain-policy correction

Session `f60db89c58984f79b50606580d317b46` used the installed `2E04C24E...` DLL. All 28 post-deorbit submissions in `CourseCorrection` failed `NoTerrainClearControllerForecast`; none published. The trace's version 24 has a null prediction input UT, so it is a version counter without a committed prediction. V1 remained in `CourseCorrection` with zero commanded throttle and no correction pulse. Its sampled unpowered terrain intersection was near 20.12° E, roughly 11.7 km before the 23.47° E target. The intersected ground was about 1,420 m ASL; target terrain was 492 m ASL. The capture ended with an incomplete final append and an abandoned final submission; the reader reports those gaps explicitly. The aggregated failure does not identify each candidate's terrain or terminal rejection, so no stronger candidate-by-candidate claim follows from this capture.

The prior target-fixed policy made V1 hand off at target terrain + 205 m, about 697 m ASL, including for uprange paths across higher ground. The earlier flight measured about 836 m ASL at one uprange endpoint; such a forecast could no longer be published as terrain clear. Yet Course Correction requires a real miss before it can push the orbit downrange. To resolve this feedback dependency, a candidate that encounters higher KSP/PQS terrain may raise its V1 speed-policy terrain reference monotonically, with a bounded number of worker repropagations at the same brake reference and bounded flight-thread PQS samples. The result carries the selected policy height, and targeted-airless V1 uses that same height after publication. If the raised policy moves the endpoint, any prior target bracket is discarded and its true signed miss is reported. A high-speed terminal candidate, an early coast intersection, failed terrain query, incomplete worker, or exhausted escalation budget is still not published. Direct braking refreshes do no terrain candidate search or policy escalation.

The first failed-flight input snapshot is now a regression fixture. With a deliberately explicit 835 m uprange terrain corridor and the captured 492 m target height, the planner publishes a terrain-clear, terminal-bound-valid controller forecast about 27.1 km short, after one policy escalation and 365 PQS-oracle queries. A second refresh using the committed policy also completes. The prior post-pulse snapshot passes a separate raised-terrain surrogate. These tests exercise the missing publication route; the surrogate is not a replacement for real Mun PQS. The next KSP run must confirm live candidate clearance, first publication, correction pulses, updated prediction versions, brake timing, and actual terminal descent before landing accuracy is accepted.

The Release build for this correction succeeded with zero warnings and errors. All 45 focused C# landing tests and 22 Python capture/replay/lifecycle/sequence tests passed. The full C# suite had 8,434 passes and the same three unrelated static/ascent failures recorded above. After KSP-closed and exact-path checks, only `MechJeb2.dll` was installed at 05:17 AEST. The source and installed DLL SHA-256 are both `16D861A47F4028D374BF3D3845FFC017D335576F363DDA95A944B9AA7899E98E`; the timestamped backup manifest is `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260929-051748-4371108\manifest.json`. No companion DLL was replaced.

### 2026-09-29 approved baseline forecast contract (supersedes the target-reaching design above)

The active predictor answers one counterfactual question: from the current settled physical state, where will V1's nominal coast, braking, horizontal-velocity kill, and final descent end **without another target-seeking correction**? Course Correction owns comparison with the selected target and any correction pulse. The predictor must not choose a brake time or endpoint by target error, run a brake-time bracket/refinement, simulate future strategic correction pulses, or find a target-reaching fixed point with its own publication. Any earlier text describing those operations as the active result path is superseded. The selected target is retained only as result lineage and for existing V1 control inputs where unavoidable; target error is diagnostic, never a trajectory-selection objective.

**One-way brake timing.** Starting at the current state, propagate an unpowered coast until V1's existing `ShouldStartDeceleration(surfaceSpeed, MaxAllowedSpeed)` first becomes true. The resulting physical trigger determines the brake reference (`trigger UT + V1's five-second coast guard`); an already-triggered DecelerationBurn starts from its observed state and command. Use the live V1 speed policy and handoff height as inputs. The result must not change `DecelerationEndAltitude`, the speed policy, throttle, attitude, or any other V1 control threshold. V1's braking `ComputeCourseCorrection(false)` term is excluded from the baseline (zero vector), recorded as bounded target-steering uncertainty, and never used to move the baseline endpoint toward the target. Existing coast RCS adjustment is likewise excluded as a future target correction; already-applied vessel velocity is part of the snapshot. A changing physical trajectory or completed real pulse requires a new prediction; pulse snapshots remain barred until V1's existing 0.75-second settling rule.

**Forecast states and geometry.** `NoForecast` means no *fresh* usable baseline. `ImpactForecast` is a fully resolved first terrain contact on the nominal path, with contact position, UT, terrain ASL, vertical and horizontal surface speeds, and validated path clearance until that contact; it is available to Course Correction, never labelled `LANDED` or shown as safe touchdown. `LandableForecast` requires a terrain-clear braking handoff with validated vertical and horizontal state *and* continuation of V1's actual KillHorizontalVelocity and FinalDescent controls to a survivable touchdown. Its endpoint is that touchdown, not the handoff. A stopping-distance lower bound alone cannot establish this state. If terminal control cannot be simulated or validated, the result remains `NoForecast` or an `ImpactForecast` only when an actual first contact is known; it cannot become `LandableForecast` by optimism. Terrain comes from bounded KSP/PQS queries, with first-contact interpolation and no hardwired Mun profile.

| V1 phase | NoForecast | ImpactForecast | LandableForecast |
| --- | --- | --- | --- |
| PlaneChange / DeorbitBurn | Existing geometric control; no result-dependent correction | Diagnostic only; finish existing deorbit operation | Diagnostic only; finish existing deorbit operation |
| CourseCorrection | Wait only while a physically possible correction cycle remains; then end autoland as unrecoverable | Use contact position and UT in the existing finite-difference correction and impact-preserving pulse limiter; never transition to coast/braking merely because error is small | Use predicted touchdown in the same correction calculation and current completion/pulse consensus rules; may enter CoastToDeceleration |
| CoastToDeceleration | No landing transition; return to correction if a usable impact result exists and time permits, otherwise end unrecoverable at the deadline | Return to CourseCorrection if a cycle remains; otherwise end unrecoverable; do not start DecelerationBurn or command forecast-based RCS | Existing coast, RCS, warp and speed-trigger behaviour; may start DecelerationBurn |
| DecelerationBurn | Do not start from this state; if a committed result becomes unusable during the burn, end autoland rather than use a stale brake reference | Never use as a braking/landing prediction; end autoland with the recorded reason | Existing braking control and handoff transition; refresh from the live state with the same baseline model |
| KillHorizontalVelocity / FinalDescent | No expensive predictor search or refresh; live terminal controller continues under its existing authority | No new terminal forecast is published | No new terminal forecast is published; existing live terminal controller runs to touchdown |
| Manual abort / landed / target or body change | Release authority or invalidate lineage respectively | Same | Same |

`PredictionReady` continues to mean only a fresh `LandableForecast`. New `CorrectionForecastReady` means a fresh `ImpactForecast` or `LandableForecast`; Course Correction alone uses it. The legacy `ReentrySimulation.Result` object cannot by itself express this distinction, so immutable published metadata must carry forecast kind, contact/handoff/terminal state, source snapshot, model identity, and expiry. Any UI reticle or label must distinguish predicted impact from predicted safe touchdown. A failed, stale, incomplete, or terrain-unresolved transaction cannot overwrite a committed fresh result. A retained result ceases to be decision-ready when its physical snapshot expires; retention for diagnostics cannot extend controller authority.

**Unrecoverable deadline.** There is no retry-count timeout. For a fresh contact forecast, compare time remaining until predicted impact with the time needed to orient for one *existing* correction pulse, execute its impact-preserving Δv with current available acceleration, allow the existing 0.75-second post-burn settling interval, and obtain the required fresh result(s) under the measured predictor refresh/worker latency. Attitude time must come from observed attitude response or a conservative vessel-specific bound; unknown response is not zero. If the latest usable forecast is still impact-only after that physical deadline, or no impact-preserving pulse can be produced, enter explicit `UnrecoverableForecast`: set throttle to zero, minimum warp, release V1 guidance authority, and display/log the reason. Do not invent an abort manoeuvre, staging, target change, or new descent control law. An explicit user abort retains its existing behaviour.

**Publication and replay gates.** A transaction publishes atomically only after one snapshot, one baseline model, complete terrain resolution, and terminal-state classification. Newer compatible complete transactions may replace earlier ones; ordinary immediate-braking, target-optimised, failed, incomplete, late, or out-of-order work cannot. The offline sequence harness must prove that a long baseline miss stays long, a target cannot affect brake reference or endpoint selection, an impact result feeds a correction but cannot authorize coast/braking, a completed real pulse invalidates its predecessor until settled, fresh compatible work replaces atomically, failed work does not, terminal searches never run, and an unrecoverable impact cannot wait forever. Compare predicted phase transitions, throttle, attitude gate, RCS/warp, terrain clearance, and final outcome against the captured whole sequence. No KSP DLL test is justified by a handoff-only or ideal stopping-distance result.

**Latest completed-flight terminal evidence and implementation gate (29 September).** The newest captured session is `10ee86d75bca457eb20073c71cbd52b5`. The capture has 32 complete publications, but its final appended line is truncated; result version 38 exists in the JSONL guidance trace only, so the harness reports that lineage gap. The trace continues after the last predictor submission into `KillHorizontalVelocity`. At UT 24,604,105.67 it had 1,319.72 m terrain clearance, 113.26 m/s horizontal speed, and the projected vessel forward vector aligned almost exactly opposite horizontal travel. At UT 24,604,126.07, that projected vector aligned **with** travel (cosine 0.868); at the last complete state, UT 24,604,196.45, the craft still had 92.63 m/s horizontal speed at 39.30 m terrain clearance. `FinalDescent` was never entered. Source computed the horizontal thrust component from the current `VesselState.Forward` projection, not from negative horizontal velocity, so the commanded direction could switch sides as attitude passed vertical. These observations reject any terminal-handoff-only `LandableForecast` for this flight. Dave explicitly approved correcting that direction to oppose horizontal travel. The shared `HorizontalKillThrustDirection` policy now supplies the live step's attitude target; its 0.2 lateral-to-vertical tilt, hover throttle, and transition speed remain unchanged. This fixes the observed direction reversal, but its landing effect still needs a controller-equivalent terminal forecast and a new flight observation.

**Current implementation status.** The baseline worker now uses the live V1 speed trigger without target brake-time search and can classify a resolved first terrain contact as `ImpactForecast`. `PredictionReady` does not admit that result; Course Correction has a separate impact-ready gate and a physical correction deadline. The worker still stops at the braking handoff when no prior terrain contact occurs. No validated continuation through the actual horizontal-kill and final-descent controllers exists, so **no `LandableForecast` is currently publishable**. The focused source tests and reader/lifecycle/sequence tests exercise the state boundaries, and the sequence harness correctly rejects the latest captured flight, but those passes are not a controller-equivalence or landing-success gate. This source must not be built for KSP deployment as a working landing predictor until a terrain-resolved terminal continuation passes the captured whole-sequence replay and the horizontal-kill behaviour is resolved within approved scope.

**Approved terminal controller corrections and diagnostic replay.** Dave approved the narrowly scoped horizontal-kill direction correction and then approved a targeted-airless final-descent speed-envelope correction; other landing modes retain their previous speed factor. The horizontal-kill step now asks for thrust opposite the measured horizontal surface velocity rather than using the craft's moving forward direction. The targeted-airless final 300 m envelope uses a 0.40 braking-distance factor instead of the legacy 0.90. In a source-policy replay of captured Mun submission 177, nominal braking reaches a handoff at UT 24,604,188.58, 697.2 m ASL, with 3.61 m/s horizontal and −7.93 m/s vertical speed. An **ideal instantaneous-attitude** horizontal-kill continuation takes 3.4 s and travels 37.8 m before reaching 0.99 m/s horizontal. A fixed-height final-descent replay with the cumulative V1 throttle PID reaches the target terrain at approximately −0.20 m/s vertical speed for 0, 3, and 10 m vessel-bottom offsets. Source resets the PID when `Tmode` changes, but retains the preceding cumulative throttle value; the replay starts with the captured throttle value. The focused tests assert that the ideal replay meets the requested 0.5 m/s setting at captured and several synthetic vertical handoffs. At a separate 100 m/s horizontal handoff, the same ideal horizontal-kill policy requires about 337 s and 15.6 km of travel; a positive 200 m handoff clearance therefore cannot certify a landing. These calculations use shared V1 policy equations, but do not reproduce finite attitude response or dynamic terrain along the terminal path. `RunNominalTerminal` remains a diagnostic worker-side continuation and has **no active publication route**. The present Mun flight never entered FinalDescent, so an in-game terminal-equivalence comparison remains unavailable. A terminal result cannot become `LandableForecast` until the full controller and KSP/PQS path have been validated offline; the new envelope factor alone is not evidence of a safe touchdown.

The sequence harness now records the ideal horizontal stop time and distance beside each observed horizontal-kill endpoint. On the failed flight's first KHV sample (113.26 m/s lateral speed), even ideal opposite-travel hover thrust needed about 354 s and 20.05 km. At its last sample, 92.63 m/s lateral speed and 39.30 m terrain clearance still implied about 289 s and 13.40 km in the ideal bound. These figures explain why correcting the KHV direction alone cannot rescue the already late, near-ground phase entry. They are lower-bound diagnostics, not a real-vessel trajectory or a safe-terrain proof.

The same trace gives one vessel-specific attitude-response observation: KHV entered at 62.25° attitude error and first fell below 5° about 5.10 s later. The old and corrected horizontal-kill direction agree at this initial instant because the craft was pointing against lateral travel, but the old direction subsequently turned with the craft. The initial settling time is useful input to a bounded terminal forecast; assuming instant attitude throughout KHV would omit an observed five-second transient. One observed turn is not a certified bound for every later attitude change.

The new `TargetAwareTerminalTerrainProbe` is a **synchronous offline terrain-oracle diagnostic**, never called with KSP/PQS on the flight thread. It interpolates the precomputed path at no more than 25 m or 0.25 s between queries, reports first contact and minimum sampled clearance, and returns unresolved when its explicit query budget is exhausted or the path is invalid. An offline flat-terrain case reaches the simulated terminal endpoint; an elevated corridor yields earlier contact; a one-query budget is explicitly unresolved. A production KSP/PQS resolver must instead spread bounded terrain queries across flight updates. The probe is **not connected to active publication**: a completed ideal-attitude terrain probe still cannot establish that the real vessel's attitude and throttle will follow that path, nor can sparse samples prove every point between queries clear. The active worker still stops at the braking handoff and retains the current no-`LandableForecast` gate.

**29 September continuation check.** The latest completed Mun capture's final braking submission (ID 33, UT 24,604,103.748) used a live policy radius of 201,807.553 m, hence a handoff altitude of 1,807.553 m ASL, while the craft still had 131.30 m/s horizontal speed. The captured DLL's policy is not the target-fixed policy in the current uncommitted source, so its subsequent flight is a negative regression case, not a positive controller-equivalence replay for that source. The capture contains no `FinalDescent` observation, and its active-session tail is incomplete. The live `DecelerationBurn` attitude gate accepts 0.75 forward alignment; the worker had rejected alignment below 0.95 and assumed perfect retrograde thrust on the current tick. The worker now uses the shared 0.75 gate and the copied forward vector for that tick, with a focused source test. Future attitude response, the dynamic terrain along horizontal kill, and terminal touchdown still lack a validated active continuation. An ideal-attitude terminal path must not be promoted to `LandableForecast` or used to steer CourseCorrection. No Release build, commit, or installation is justified by this change alone.

**Continuation evidence from the last braking input.** Replaying captured submission 33 from its actual UT, mass, thrust, orbit, and forward vector under the *current source* target-terrain-plus-200-m policy reaches the 697.2 m ASL handoff at UT 24,604,138.52 with 4.33 m/s horizontal and −7.37 m/s vertical speed. The ideal shared-policy horizontal-kill continuation then takes 5.8 s and travels 65.2 m; the low-altitude final-descent replay reaches flat target-height terrain at −0.20 m/s vertical and near-zero horizontal speed. A sensitivity replay coasts without thrust for the 5.1 s attitude-settling interval observed in the old KHV trace, then runs the same ideal controller policies. It also reaches flat terrain at about −0.21 m/s vertical, with an endpoint 78.4 m from the instantaneous-attitude result. This **does not** model V1's actual thrust while turning or establish a maximum settling delay. These are controlled counterfactuals, not observed flights of the changed source. They demonstrate that the old DLL's 1,807.6 m handoff, which left 113 m/s horizontal at phase transition, materially altered the outcome; they do **not** certify either terminal endpoint or the Mun terrain along the later path. The final-descent diagnostic now refuses to apply its low-altitude controller law above V1's 300 m branch boundary. A new incremental terrain resolver reproduces the offline flat and rising-terrain contact results under four queries per update, and reports unresolved on query exhaustion or PQS failure. It is not yet connected to active publication. These checks pass in the focused C# suite; the full-sequence flight audit remains negative because the only available flight used the earlier controller policy and never entered `FinalDescent`.

**29 September test build and remaining equivalence limit.** The incremental KSP/PQS terminal resolver is now connected to the single active target-aware transaction. A clear braking handoff leads to an asynchronous nominal terminal replay and a second replay with the observed 5.1 s unpowered attitude-delay sensitivity. Each path is resolved incrementally against dynamic terrain, including the captured vessel-bottom offset. A first unsafe contact is classified as `ImpactForecast`; a contact in final descent with both replayed speeds within the configured touchdown and horizontal limits and endpoint spread within targeting tolerance is classified as `LandableForecast`. No handoff is labelled `LANDED` without this terminal continuation. Ordinary immediate-braking results remain barred from the target-aware slot, and failed/incomplete terminal work retains the prior committed result subject to its freshness gate. This is a **test build**, not a completed controller-equivalence proof: the two replay paths do not model V1's finite attitude and actual thrust while turning, and a single measured 5.1 s delay is not a certified upper bound. Terrain samples constrain the predicted paths but cannot prove every interval between samples clear. In particular, a `LandableForecast` from this build is conditional on those documented model assumptions; KSP feedback must determine whether it is accurate. The 64 focused C# tests and 27 Python capture/lifecycle/sequence tests pass. The full C# run has 8,453 passes and the same three unrelated static/ascent failures previously documented. The previous DLL's whole-flight audit still fails with 31 issues and four unknowns, as expected; it is a negative fixture and cannot validate this build. At Dave's explicit request to build and install for KSP testing, the Release build completed with zero warnings and errors and only `MechJeb2.dll` was installed after a closed-game and exact-path check. Source and installed SHA-256 both equal `B6A4E9B387CCE6D9872CC54C32A953BAED5BD0917EF9D35F124A5301575A86E9`; the timestamped backup manifest is `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260929-233542-0408192\manifest.json`. No companion DLL was changed.

### 30 September correction: safe brake timing precedes target correction

This amendment supersedes the **one-way physical-trigger timing** and **no brake-time search** clauses above. They caused an immediate V1 brake reference in the 29 September no-reticle flight: 18 complete refreshes, all classified `ImpactForecast`, with an endpoint about 126 km short of target. A throttle-capable vessel can still crash when braking is scheduled at the wrong point. V1's `DecelerationBurn` reads the published brake reference and will coast until that time; therefore timing is part of the predictor/guidance contract. The target-reaching root used in an older experiment made the opposite mistake by choosing a brake time to erase target error. The active predictor must choose a **safe, later brake time independent of target error** and report the resulting touchdown location. V1's existing `CourseCorrection` uses that location to move the real trajectory toward the selected target. `DeorbitBurn` is geometric and does not read the prediction; this repair does not claim otherwise. A refresh after each settled real correction uses the changed physical state and repeats the same safety timing selection.

For a settled targeted airless snapshot, resolve the unpowered first terrain contact against KSP terrain. On a worker, sample later V1 brake references before that contact using V1's actual coast guard, speed envelope, throttle law, engine inputs and handoff height. Resolve candidate handoffs against bounded KSP/PQS queries on the flight thread. A candidate is ineligible if it intersects terrain before handoff, lacks vertical stopping distance or a 20% ideal thrust-acceleration reserve, or reaches handoff above V1's allowed surface speed by more than 10%. The surface-speed check is essential: a captured Mun fixture produced a late handoff with about 538 m/s horizontal motion despite adequate vertical clearance. Refine between the latest eligible coarse candidate and the next later ineligible candidate; choose the latest eligible refined candidate, then recheck its full coast and powered trajectory against terrain. The target coordinates are used only to measure and publish signed downrange and crossrange miss, never to rank candidates. A candidate is labelled `LandableForecast` only after the existing terminal continuation and dynamic terrain checks produce a valid touchdown; the handoff is never the published landing location. This test-build classification remains conditional on the recorded terminal attitude-response model, so the KSP flight is the performance test.

If no candidate meets that safety screen, the already resolved **ballistic first contact** may be published as an `ImpactForecast` for Course Correction. It has no brake reference, cannot authorize coast or braking, and must not appear as a safe blue landing reticle. A terrain change that invalidates the stored contact instead fails the transaction. Failed, stale, unresolved, or out-of-order work never replaces a committed result. Once V1 enters `DecelerationBurn`, only a live direct continuation is refreshed; `KillHorizontalVelocity` and `FinalDescent` do no candidate search. Target-aware ownership continues across DeorbitBurn to CourseCorrection and ordinary immediate-braking workers cannot replace it.

The flat-terrain Mun fixture needs 1,033 terrain-oracle queries for the selected full path and two terminal continuations. The transaction cap is 1,536 queries, spread across updates with at most 32 queries and 1 ms of query work per flight-thread tick. Terminal terrain interpolation remains capped at 25 m of path travel and now at 1 s elapsed, avoiding fourfold redundant hover samples. The in-game test must record the chosen brake reference, full first-contact/terminal endpoint, actual V1 burn start, throttle and attitude, terrain clearance, correction pulses, result lineage, and touchdown or impact. A safe classification is not a promise of landing accuracy until that flight validates the unmodelled actuator and terrain intervals.

**Test-build evidence.** The 64 focused C# landing tests and 22 Python capture/lifecycle/sequence tests pass. The flat Mun replay selects a later brake reference, validates terminal contact, and produces the same reference and contact state with a different target longitude; its large target miss is retained for guidance to correct. The raised-terrain captured fixtures yield a ballistic `ImpactForecast` without changing V1's live speed policy. The Release build completed with zero warnings and errors. With KSP closed and the exact `KSPDIR` verified, only `MechJeb2.dll` was installed; source and installed SHA-256 are both `89FEF92486E6194A55F9ECD5DB8C9AC92283DFB7AA969CE2C0CF716C29798940`. The prior DLL and timestamped manifest are at `C:\Users\Dave\Documents\KSP Backups\MechJeb2 Explicit Install\20260930-010501-6364319\`. No companion DLL was changed. This is ready for Dave's controlled Mun test, not an assertion that the landing will succeed in KSP.
