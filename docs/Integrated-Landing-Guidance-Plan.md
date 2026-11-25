# Integrated landing guidance — implementation plan

**Date:** 2026-11-26 (Australia/Sydney)  
**Status:** Proposed implementation plan for review; documentation only. No runtime change or installation authorised by this document.  
**Source baseline:** `03120ee2f6248ce3a7399e4ad1f4afd99cbe3f53` (`2.16.0-landing.20261126.duna-dual-window`).  
**Scope:** KSP 1.12.5, targeted landings, powered guidance with atmospheric forces; vacuum is the zero-atmosphere special case. Preserve proven airless V1 until replacement meets independent regression gates.

## 1. Problem and decision

Suspend incremental tuning of the Duna dual-window candidate. Retain its captures and source as experimental evidence, not as the next architecture. The atmospheric predictor, correction law and emergency braking supervisor presently disagree about the intended executable trajectory. The geometric deorbit, fixed speed envelope, impulsive orbital sensitivity, permanent rejection of correction after one failed pulse search, conservative no-drag braking screen and altitude-only terminal transition are not a joint landing solution.

Observed evidence: M1 failed after prolonged correction, first actual braking after transition by 6.08 s, and terminal approach at excessive speed; pilot observed full throttle, so logged NaN requested throttle is not established as the crash cause. The dual-window screenshots showed a large short prediction with RCS enabled (new path ineligible), then with eligible settings an early DecelerationBurn at approximately 85.6 km terrain clearance and 243.3 km predicted target miss. The exact latest commitment reason and achievable range are unverified without the flight trace. Do not infer that delayed braking alone makes that target reachable.

The review of the refined airless path corrected an earlier assumption: TargetAwareAirlessPlanner.FinishCoarse / PrepareFallbackFromCoarse use latest safe reference and continuity for HasV1ControlModel, whereas the alternate TargetAwareBrakingPlan path supports target-error ranking and downrange bracketing. Do not simply enable the airless planner for atmosphere or reuse its ranking unchanged.

Adopt **plan-led, target-aware, constrained powered descent guidance (PDG)** with bounded receding-horizon replanning. Prediction and execution shall share the same dynamics, phase events and commanded trajectory; independent safety supervision may override, but every override invalidates the nominal plan and changes the forecast's status. Optimise coasting and braking timing before residual targeting corrections, then permit coupled optimisation where demonstrably better. Compare candidate algorithms empirically; no claim of global optimality.

## 2. Requirements and exclusions

- Primary goal: physically feasible safe touchdown on the selected surface; minimise landing error, then propellant subject to accuracy tolerance and protected reserve. Do not silently move the selected target. If unreachable, report best feasible miss and reason; before irreversible action reject an infeasible plan. In flight, prioritise achievable safe landing over target capture.
- Model rotating body, nonuniform gravity, atmospheric drag and passive lift, actual vessel geometry, thrust/propellant capability, attitude slew and settling, engine lag, and terrain. No assumption of lift authority, wings, controllable bank or gliding; ordinary arbitrary KSP vessels must work. Active aerodynamic steering is a future optional extension, not a prerequisite.
- Initial flight-validation envelope: powered Duna, no chutes, RCS initially disabled to isolate actuator evidence. These are **test case choices, not global eligibility switches**. RCS option must not disable planner/safety. Parachutes are an optional modelled actuator, introduced after powered validation; until supported explicitly declare configuration unsupported for the new plan, keep legacy path available before engagement. Do not claim a chute-enabled plan is validated when deployment is unmodelled. Minimum throttle limiter is excluded by user decision; no requirement to support imposed nonzero throttle. Actual engine inability to throttle/restart is a capability limitation, not a reason to pretend authority exists.
- Independent entry and braking readiness constraints: finish separate correction, including tail-off, and acquire/settle entry attitude by atmospheric interface OR prepare thrust vector and engine by earliest required powered braking, whichever limits the manoeuvre first. Entry attitude follows validated vessel configuration/reference; retrograde baseline where appropriate, not an asserted universal aerodynamic stability solution. If already inside atmosphere, establish attainable readiness with no fictitious crossing. Remain compatible with entry attitude and thrust constraints during subsequent targeting; no re-entry into separate correction after commitment.
- Retain existing landing surface support where validated (PQS, loaded deck and bottom clearance); distinguish water, steep/obstructed surfaces and unsupported configurations. Do not assume an arbitrary landing target is safe.
- Preserve manual abort and normal MechJeb actuator interfaces. Do not change airless V1, untargeted landings, other autopilots or public defaults during atmospheric development. Staging, complex propulsion and active aero are not first milestones; represent capability conservatively and explicitly flag unsupported transitions. No new mandatory settings except proven incompatibilities. Display exact test prerequisites and rejection reasons at delivery.

## 3. Architecture and contracts

Use a new opt-in atmospheric plan path initially. Decide exact class names/namespace after M0 inspection of existing V2 interfaces; the following are conceptual boundaries, not existing APIs:

| Component | Responsibilities |
|---|---|
| Snapshot provider | Immutable physical input and capability manifest, body and target identifiers, UT, frame, resources, attitude/actuator response, environment, terrain handles; copy mutable KSP data on main thread |
| Dynamics / propagation | Deterministic coast and finite-burn propagation, aerodynamic force adapter, event location, mass and actuator state, terminal contact; callable by planning and validation |
| Planner | Candidate generation, feasible range, constrained PDG, residual correction, deorbit and ignition decisions, fuel accounting; no direct actuation |
| Independent validator | Nonlinear forward replay of candidate including actual command model, terrain corridor, reserve and terminal state; validate uncertainty cases and inter-node bounds |
| Plan manager | Atomic versioned publication, continuity, deadline-aware replacement, ownership, stale/invalid result semantics, cancellation, warm start and time budgets |
| Tracker | Convert reference state and feed-forward thrust to attitude/throttle requests with feedback, capability-aware limits, anti-windup and measured response; single actuator owner |
| Supervisor | Independently track recoverability, safe fallback and imminent hazards; no waits for optimiser; override invalidates current plan |
| Warp scheduler | Sole warp owner for new path; exit early enough for fresh snapshot, actual settling and ignition; prohibit warp during unsupported physical dynamics |
| UI / telemetry | Existing target vs prediction semantics, status, expected miss, plan age, validity, remaining reserve, preparation deadlines, requested/actual thrust, rejection and fallback reasons |

Data contracts:
- GuidanceSnapshot includes snapshot/generation, state UT, body, target revision, position/velocity in explicit frame, mass/usable propellant, engine force/flow maps, attained orientation/angular rate, thrust state and limits, atmosphere and configuration revision, gear/chute states, terrain/collider offsets and uncertainty. Never use previous predicted endpoint as a physical input. A previously validated solution may seed the new solve only.
- LandingPlan contains the full time-ordered reference state/control, deorbit/correction/entry/preparation/first-required-thrust/terminal/contact events as separate concepts, expected fuel/clearance/terminal error, reachable range, validation envelope, generation and expiry. A plan is not merely a touchdown coordinate.
- Forecast contract distinguishes feasible targeted, feasible miss, impact/unsafe, unresolved terrain, unsupported capability, no encounter, incomplete/timeout and timing-only stale information. Solver failure or timeout does not prove infeasibility. Geometric contact is not a landable outcome. Keep old valid forecast visibly aged only where current reachability and lineage still permit; never allow stale timing to postpone a safety action.
- Every command has plan/version ownership and validity interval; invalidation on target, vessel configuration, stage, warp, capability or major deviation. No concurrent controllers writing contradictory throttle. No opportunistic switch back to legacy while powered unless an explicitly validated handover exists.
- Keep requested target and predicted touchdown separate. Do not relabel a handoff point as touchdown. Status must differentiate planned, executing, replanning, degraded and emergency, and identify why targeting cannot continue.

## 4. Dynamics and mathematical problem

Use SI internally (explicit conversion of KSP units including tonnes/kN where applicable). Body-centred inertial position r, velocity v, mass m; target and terrain rotate with body. Air-relative velocity w = v - omega cross r for stock co-rotating atmosphere; use actual environment adapter where supported. Do not confuse orbital, surface and vessel-frame velocities.

    dr/dt = v
    dv/dt = -mu*r/|r|^3 + (T_actual + D + L)/m
    dm/dt = -sum(engine and enabled RCS propellant mass flows)

Model attained orientation, angular response and engine response state (measured/identified, bounded where uncertain). For one ideal engine, flow = thrust/(Isp*g0); for heterogeneous engines use actual per-engine flow and pressure/capability, not a single assumed Isp. Include gravity and thrust loss during finite burns and coast; do not apply a fictitious instantaneous impulse at a different integration interval. Aerodynamic attitude and drag/lift use actual or commanded/attainable orientation; verify coordinate transformations with live measurements, including roll/lift direction. No aerodynamic steering variable in initial solver. Passive lift is disturbance/modelled force, not an independently commandable force.

Control is piecewise throttle and thrust direction (including coast/off), constrained by attitude response, available engines, fuel and enabled actuation. Position and velocity constraints must hold throughout the trajectory, not only at mesh nodes. Event-resolved integration for entry, thrust onset, terrain/gear contact, fuel exhaustion and phase boundaries. Terrain path clearance accounts for vessel extent and uncertainty; query candidate corridors, not global maximum terrain as universal nominal ground. Global/no-drag conservative bounds can screen or protect, not rank nominal solutions; no-drag is not universally conservative with adverse lift or varying engines.

Terminal conditions at actual surface: horizontal position within declared tolerance; horizontal velocity near zero; vertical speed within configured safe contact envelope; suitable attitude and angular rate; no earlier collision; remaining usable resource reserve. For a terminal capture/handoff, validate subsequent real terminal controller to touchdown. Do not require hovering where a direct feasible trajectory exists. With insufficient hover/landing thrust, assess actual touchdown feasibility rather than equate TWR alone with success.

Lexicographic objective:
1. Satisfy hard dynamic, clearance, actuator, entry, fuel and touchdown constraints under stated uncertainty scenarios.
2. Minimise miss if target tolerance cannot be reached. For reachable targets, enforce tolerance instead of paying to chase zero error.
3. Maximise terminal usable propellant (equivalently minimise total mass consumed for fixed configuration) with protected reserve; account for all manoeuvres.
4. Tie-break on continuity, unnecessary slew/hover and time. Avoid arbitrary cross-unit weights that trade safety for accuracy.

Solve in a rotating target-local frame for terminal residuals or transform inertial states at each UT; handle polar/dateline/longitude wrapping and changing local axes. Do not assume touchdown range is monotone in ignition time; search disconnected feasible intervals and use sign bracketing only on verified continuous branches.

## 5. Solver selection: mandatory benchmark, not a foregone choice

SpaceX publicly describes onboard convex optimisation; G-FOLD and published successive convexification are design references, not proprietary SpaceX code or evidence of identical Super Heavy algorithms. A simplified 3-DoF PDG model with bounded attitude/engine dynamics is appropriate initially. Full 6-DoF and aero steering are unnecessary for the first objective.

A. **Reference bounded search:** propagate fixed coast/brake profiles, coarse ignition-time samples over executable interval, refine safe intervals, optimise duration and small vector-control parameters, revalidate terminal path. Always provide an honest best feasible miss. This is the transparent benchmark and possible low-cost fallback, not final optimum.
B. **Direct nonlinear optimisation:** inspect existing managed MechJebLib.PSG.Optimizer, phase transcription, ALGLIB minnlc/SQP and derivative/cancellation infrastructure. It currently transcribes ascent problems and is not a ready-made descent solver. Test independent landing formulation with free final time, variable throttle/coast, terminal constraints and bounded solve time. Reuse maths, not ascent-specific assumptions. Avoid changing existing ascent optimiser.
C. **Convex PDG / successive convexification:** prototype fixed-time local translational convex subproblem (thrust norm, pointing, velocity, terminal constraints), outer search over final/ignition times; nonlinear gravity/drag/mass/attitude linearised about reference for successive updates, with trust regions, scaled variables and explicit virtual-control/slack residual checks. Never accept relaxation slack as physical authority or extrapolate lossless-convexification results beyond their hypotheses. Forward-propagate actual nonlinear model independently before publication. Investigate managed QP/conic capability and licence/deployment cost; a QP approximation is not an exact SOCP. No new dependency until benchmark establishes need and compatibility with net48/C#8/KSP Mono.
D. Use receding horizon: replan remaining trajectory on fresh state, warm start from prior valid plan, allow independent safety action during solve. Terminal tracking at physics rate; planner slower and cancellable. No continuous unbounded nonlinear solve on game main thread.

Benchmark A/B/C on identical snapshots, target tolerance, force model and validator. Record feasible solution rate, false feasible/false infeasible against stronger offline search, terminal miss, propellant, earliest actual thrust, robustness, p50/p95/p99 solve and main-thread cost, allocations, timeout and recovery. Vary mesh/time resolution and initial guesses, include no-target and low-authority cases. Select simplest algorithm meeting all gates; hybrid A + best PDG may be appropriate. An offline high-resolution nonlinear solution is reference, not proof of global optimum. Document selection with data before choosing production solver.

## 6. Guidance lifecycle

| Stage | Decision and exit |
|---|---|
| Preflight / orbit | Determine usable engines, surface, accessible target and fuel; search deorbit epoch/vector and resulting entry corridor if deorbit needed; no burn on unknown or infeasible plan |
| Deorbit execution | Track counted finite burn, measure actual delivery, update plan on fresh state; do not blindly aim for fixed 0.9R periapsis; do not repeat deorbit if already on feasible descent |
| Approach / coast | Search braking commencement and feasible landing range on existing trajectory first; avoid correction when timing/steering alone meets target; choose candidate with acceptable fuel and margins |
| Residual correction | Generate direction/delivery from full trajectory response or geometric seed, simulate finite burn + tail + reoptimised braking/terminal path; execute only useful feasible improvement; do not permanently abandon correction solely because one candidate fails |
| Entry preparation / entry | Latch separate-correction commitment at readiness deadline, command attainable validated attitude, observe actual angle/rate and thrust tail; keep independent safety checks; no separate correction re-entry |
| Powered descent | Execute actual planned ignition/attitude/force profile; jointly manage downrange, crossrange and energy using available thrust vector. Continue safe steering during braking, including above interface if compatible; never allow steering to consume required braking authority |
| Terminal approach | Track combined position and velocity with continuous attitude transition. Handoff only after physically recoverable state verified; use existing terminal laws initially if accurately modelled, later compare integrated terminal tracking |
| Contact / stop | Validate touchdown, stop landing commands on actual contact/landed state, release authority; log final coordinates, contact velocity, reserve and result |

The outer strategic optimiser selects orbit/entry geometry; the approach optimiser selects executable coast, finite burns and trajectory; inner tracking handles deviations. Do not spend correction propellant to compensate for an avoidably short braking solution. If no safe solution is found, distinguish search failure from infeasibility; replan when time permits. On loss of plan before irreversible action, hold/abort safely; during descent, execute an independently validated reachable fallback if available, otherwise explicit best-effort hazard response. No guarantee of recovery from physically unrecoverable state.

## 7. Reuse map and integration boundaries

| Existing source | Reuse / action |
|---|---|
| MechJeb2/LandingAutopilot/AirlessTargetAwareSimulation.cs | Reference for controller-consistent finite burns, mass, state transfer and terminal continuation; extract or adapt with vacuum regression, not direct atmosphere enable |
| TargetAwareAirlessPlanner.cs; TargetAwareBrakingPlan.cs | Bounded coarse/refine/warm-start candidate search, coverage and continuity; separate selection objective; no assumption all atmospheric endpoints monotone |
| CorrectionBurnPlanner.cs; BrakingEndpointResponse.cs | Finite-burn backtracking, measured endpoint sensitivity; recompute using atmospheric dynamics and reoptimised remaining trajectory |
| TargetAwareTerminalHandoff.cs; TargetAwareTouchdownSafety.cs | Extend necessary bounds and actual terminal path checks; distinguish optimistic bound from sufficiency |
| TargetAwareTerrainCache.cs; TargetAwareTerrainContext.cs; TargetAwareTerminalTerrainProbe.cs; LandingSurfaceProvider.cs | Candidate corridor queries, bounded main-thread terrain work, surface and contact; validate cached spatial resolution and uncertainty |
| LandingForecastContract.cs; TargetAwarePublicationGate.cs; PublishedForecastHold.cs; LandingPredictionConsensus.cs | Ownership, lifecycle and freshness; do not require indefinite consensus or let stale endpoint delay readiness; compare snapshot-matched residuals |
| V1WarpPreparation.cs; V1LandingControlPolicy.cs | Measured slew/settling/warp response and pure controller laws where applicable; do not transfer fixed scalar braking envelope as nominal optimiser |
| DunaM1Capture.cs; LandingPredictorCapture.cs; LandingPredictorTrace.cs | Correlated replay evidence; extend to plan, command, achieved state, solver and override lineage |
| DunaBrakingHandoff.cs | Retain as candidate safety-screen research; distinguish unsafe vs unknown/numerical failure; recalibrate on actual approach corridor; no unconditional failed-probe nominal ignition |
| ReentrySimulation.cs; SimulatedVessel/drag cube infrastructure | Atmosphere/force inputs; inspect frame convention, prior endpoint terrain feedback, accepted vs next dt, static mass/thrust and geometric contact. New path avoids legacy inconsistency without unreviewed global changes |
| MechJebLib/PSG/Optimizer.cs; alglib project | Existing managed nonlinear solver precedent; net48/C#8 compatibility; benchmark before reuse or new dependency |
| MechJebModuleLandingAutopilot.cs; MechJebModuleLandingPredictions.cs | Adapter/dispatch, opt-in isolation, generation and actuator ownership; do not replace existing airless or untargeted dispatch |
| DeorbitBurn, PlaneChange, LowDeorbitBurn, CourseCorrection, CoastToDeceleration, DecelerationBurn, KillHorizontalVelocity, FinalDescent | Existing control reference and rollback. Replace decision ownership in new path incrementally; model exact retained terminal law; no altitude-only safety handoff |

Related prior designs: [V2 proposed architecture](LandingGuidanceV2.md), [V1 baseline](LandingGuidanceV1-Proven-Baseline.md), [V1 predictor review](LandingGuidanceV1-TargetAwarePredictor-Design-Review-2026-09-27.md), [M1 capture](Duna-M1.md), [dual-window history](Duna-Braking-Handoff.md). Preserve their historical acceptance records. This plan supersedes incremental Duna development direction only; it does not automatically approve or replace every historical V2 requirement.

## 8. Implementation milestones and gates

No milestone automatically authorises installation. Each implementation increment records source/fixture identities, changed files, test outcomes, measured limits and unresolved decisions in DEVELOPMENT.md; update SPEC/README/CHANGELOG as appropriate when actual behaviour changes. Version increments only for substantive functional changes, not packaging.

| Milestone | Deliverable | Exit gate |
|---|---|---|
| M0 — evidence and harness | Pin proven airless source and exact binaries; obtain matched Duna initial save/craft, failed M1 and latest dual-window trace, actual KSP build/parts and settings, source version; inspect existing V2, PSG and test runners; repair reference-assembly regression environment | Reproduce named baseline outcomes with correct references, record airless successes and failures, parse flight trace with units and time lineage; no solver tuning to screenshots alone |
| M1 — dynamics parity | Isolated deterministic atmosphere/finite-burn simulator, actual actuator and terminal models, terrain events, numerical domain/step fix; no live commands | Analytical vacuum/constant-accel/rotation/mass cases; timestep convergence; drag/lift frame checks vs flight; reconstruct observed thrust/trajectory using recorded actual commands; explain residuals and uncertainty; separate contact from safe touchdown |
| M2 — feasible-range planner | Bounded ignition-time sweep/refine + terminal validation on fixed approach, offline plots and candidate table | Recover reachable interval(s) and best safe miss for Duna snapshots; show whether target can be reached by timing alone; reject unsafe/unknown paths, detect disconnected branches; reproduce current short forecast under legacy policy as a comparison |
| M3 — solver benchmark | Implement A/B/C prototypes and common scoring/validator; document decision | Benchmark success, accuracy, fuel and timing on same fixtures; choose production solver and fallback; no unbounded iteration, virtual-control or constraint residual accepted; validate selected algorithm with nonlinear replay |
| M4 — shadow integrated plan | Immutable snapshots, plan publication, freshness/ownership and replanning; entry and braking deadlines; no actuation | Actual game flight shadow mode shows plan age, reachable range, predicted trajectory, readiness; cancellations/warp/target changes do not publish stale results; compare actual known control trajectory separately from alternative plan |
| M5 — powered execution | Opt-in plan tracker, exclusive ownership, supervisor and warp; direct coast/brake and terminal execution, no strategic deorbit change yet | Matched Duna powered/no-chute flight from same save: safe touchdown, planned vs actual thrust, entry/terminal states, miss and fuel recorded; explicit fail/abort/fallback tests; no regression to uncontrolled late/early braking |
| M6 — integrated correction and steering | Finite correction + reoptimised braking; powered position/velocity control; feasible terminal transition | Cases reachable by timing alone burn no separate correction; correctable cross/downrange cases converge; no useful pulse rejected solely by one candidate; no post-commit separate correction; demonstrate improved error/fuel and safe touchdown |
| M7 — strategic targeting | Joint deorbit epoch/vector and approach optimiser, finite burn, resource budget, plane-change selection | Full orbit-to-touchdown runs on Duna, varied inclination/altitude/TWR; compare with starting descent tests; no irreversible action on infeasible preflight; show reduced total fuel and correction cost where achievable |
| M8 — expand capability | Enable RCS allocation with independent safety, model optional chute phases, additional atmosphere regimes and supported configurations | RCS on/off does not silently disable guidance; chute enabled/deployed model validated; Kerbin/Eve/Duna stock tests where physically feasible; explicit unsupported conditions, no global body-name gate for validated model |
| M9 — release readiness | Cross-body regression, UI, performance, dependency/deployment, rollback documentation | Full relevant automated suite green with real references; matched live airless regressions unchanged, atmosphere flight matrix passes; package and exact-path install only after separate approval |

M0–M4 are non-actuating. M5–M7 are opt-in powered tests; not a general release. M8 RCS work may be brought forward if needed to accurately represent enabled assistance; this sequencing does not justify using the RCS checkbox as a global planner/safety switch. Active aerodynamic steering explicitly deferred beyond these milestones.

## 9. Acceptance matrix and quantitative measures

Agree final engineering thresholds from M0/M1 measurements before tuning; following are **proposed initial test targets, not existing requirements or achieved results**. All candidate comparisons use same vessel/save/target, resource state, game version and configuration, with repeated runs and seeds where applicable.

- Nominal targeted reachable cases: initial target <=10 m surface miss, horizontal touchdown speed <=1 m/s, vertical contact within configured TouchdownSpeed plus measured physics-tick tolerance (propose 0.2 m/s), upright/appropriate landing attitude (propose <=5 degrees), angular rate (propose <=1 degree/s), no earlier terrain/collider contact; actual limits depend on vessel and surface. Preserve airless proven 1.64 m Mun landing record as reference, not a universal atmospheric guarantee.
- Feasible-miss cases: report miss and reason before burn if possible; compare best feasible error against offline reference and verify safety; never require target accuracy for intentionally unreachable test.
- Model parity: compare time series position, surface velocity, attitude, actual thrust/flow, drag/lift, first thrust, phase events and contact; proposed controlled replay integration convergence <1 m endpoint and <0.1 m/s velocity when halving dt in smooth analytic cases; calibrate flight discrepancy envelope separately. No false declaration of safe touchdown on known impact traces.
- Safety: zero accepted unsafe candidates in tested matrix; test late/early braking, excessive speed at handoff, negative discriminant, TWR/capability changes, unknown terrain, NaN/Infinity, zero/negative mass and stale predictor, poor alignment and actuator lag. Treat positive Infinity sentinel separately from invalid command. No nonfinite throttle/vector reaches actuators.
- Entry/braking readiness: record planned and measured angle, angular rate, actual throttle/engine tail, entry crossing and first required/actual thrust; measured readiness thresholds calibrated per vessel, not arbitrary fixed margins asserted universal. Exercise both deadline orderings, already-inside and no-intersection cases.
- Efficiency: log total propellant and phase delta-V, correction impulse, hover time and reserve. Prefer zero separate correction where timing/steering reaches tolerance; no efficiency claim based on unmatched trajectories. Compare constrained equivalent miss/safety to legacy and bounded reference.
- Performance: propose <=1 ms p95 main-thread incremental planning work per physics tick (terrain work budgeted), asynchronous solve p95 <=0.5 s wall-clock for local refresh at 1x, initial full search <=2 s target; report p99, deadline misses, CPU, allocations and actual physics/warp latency. These targets are provisional and must be adjusted only with measurements; no acceptance of stale solve solely to meet timing. Actual deadline-relative freshness takes precedence over fixed wall time.
- Regression: recorded successful Mun/Minmus and previously failing approach replays; exact preserved path dispatch and no actuator/UI changes outside opt-in. Latest baseline and candidate CI both 37 pass/54 fail in broad stub-reference environment, so passing new focused tests is insufficient. Re-run with real compatible KSP/Unity assemblies; classify any remaining failures with evidence; do not mark release-ready with unresolved relevant regressions.

Matrix: vacuum zero atmosphere; thin/thick atmosphere; high/low available thrust and varying mass/Isp; ascending/descending and horizontal/steep approach; crossrange and downrange overshoot/undershoot; different terrain elevation and slopes, poles/dateline; aero asymmetry and attitude lag; no atmosphere intersection, entry already passed; fuel shortfall and unreachable targets; solver timeout/cancellation, warp exit, target change, engine loss, abort, vessel reload, RCS option toggles; later chute configurations. Use deterministic synthetic tests, recorded flight replay, offline optimisation comparison, shadow game tests and matched closed-loop flight. Record denominators, not just best successes.

## 10. Decision checkpoints and risks

- M0: determine authoritative working baseline and real test references; latest trace/save needed to establish cause of premature commitment and actual achievable Duna range. Cannot derive them from screenshot.
- M1: establish aerodynamic orientation, drag cube fidelity, pressure-dependent engines, mass/flow, torque/response and supported surface geometry. Unknown model uncertainty must remain visible, not treated as zero.
- M3: select solver by actual benchmark; existing PSG uses ALGLIB and defaults suited to ascent, not a landing latency guarantee. Assess dependency licences, architecture and package compatibility before introducing native/conic libraries. Avoid rewriting proven airless components.
- M4–M5: validate single actuator ownership and fault policy before enabling commands. Optimisation and safety are distinct; emergency intervention may sacrifice targeting but not fabricate a safe plan. Solver timeout cannot force premature braking unless independently justified by shrinking recoverability reserve.
- M8: agree validated capabilities and compatibility boundaries before enabling other bodies/configurations. No unsupported chute action, no automatic active aerodynamic steering, no minimum-throttle extension.
- M9: deployment is separate: verify exact commit/build, artifact hash, assembly manifest and companion compatibility, closed KSP and rollback backup. A changed MechJebLib/ALGLIB build may require companion assembly deployment; do not assume previous single-DLL packaging remains valid.

## 11. References

- Lars Blackmore, *Autonomous Precision Landing of Space Rockets* (2016 symposium): https://www.nationalacademies.org/read/23659/chapter/10 — public SpaceX convex optimisation and precision landing context.
- NASA G-FOLD programme: https://techport.nasa.gov/projects/12272 ; flight testing: https://www.nasa.gov/centers-and-facilities/armstrong/jpl-masten-testing-spacecraft-precision-landing-software/ — fuel-optimal large divert, real-time flight evidence.
- Szmuk et al., *Successive Convexification for Real-Time 6-DoF Powered Descent Guidance with State-Triggered Constraints*: https://arxiv.org/abs/1811.10803 — conditional constraints and nonlinear trajectory optimisation reference; not a requirement for full 6-DoF implementation.
- NASA *Smart-Divert Powered Descent Guidance*: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20130012660.pdf — minimum-miss feasible diversion and constraints.
- Repository source baseline and linked historical documents above. No proprietary SpaceX guidance code is claimed or required.

## 12. Immediate next work

Begin M0 only after implementation authorisation. Secure the matched Duna save/craft and latest trace from the user or an explicitly authorised environment; identify physical inputs missing from M1 capture. Reproduce baseline and build replay tests before implementing or flight-testing another controller candidate. M1 and M2 are the first engineering deliverables; do not begin live actuator changes until the predictive model and solver benchmark pass. This plan is the development reference for the new approach; prior builds remain available as historical experiments and rollback candidates.
