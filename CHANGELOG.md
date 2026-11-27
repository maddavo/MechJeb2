# Changelog

## 2.16.0-landing.20261126.duna-brake-handoff

- Added bounded Duna-only powered braking commitment independent of ordinary prediction readiness, including pulse admission, preparation reserve, commitment latch, finite conservative thrust override and trace fields.
- Preserve nominal braking steering when reserve permits. Airless path, parachutes, ordinary predictor, correction consensus/caps and terminal controllers unchanged. Experimental; flight validation required.

## 2.16.0-landing.20261125.duna-m1

- Added explicitly armed, bounded Duna targeted powered landing observations, correlated live and ordinary predictor records, numerical interval/virtual impulse accounting, and geometric contact semantics.
- Added focused diagnostic tests and capture/build instructions. No guidance, chute or publication-law changes. Not built or tested in KSP in this environment.

Earlier landing development history: [V1 performance record](docs/LandingGuidanceV1-Commit-Performance-Record.md).

## Dual-window candidate (2026-11-26)

Version: `2.16.0-landing.20261126.duna-dual-window`. Duna targeted powered/no-chute only. Independently screen the descending atmospheric interface intersection using runtime orbit and RealMaxAtmosphereAltitude, reserving correction tail, measured/fallback slew and settling, warp overrun and margin. The existing no-drag-credit braking delay probe reserves correction tail separately and retains engine preparation. Entry commitment coasts surface-retrograde without forecast, blocks re-entry to CourseCorrection, and keeps independent braking guard active. Braking commitment bypasses forecast timing; fresh optional braking steering remains available below the entry interface when reserve allows. Larger correction commands (initial cap 4 m/s, up to 8 m/s after measured scalar error improvement beyond estimated coast drift; near-target and reversal limits) are screened against geometric response and both current and instantaneously perturbed entry/braking projections. Retain two accepted post-tail directions; require new pulse forecast age <=5 s. Trace predicted entry/cutoff, commitment reason/miss (unknown when stale), actual entry alignment/residual with bracketing samples, pulse request/delivery/trust. No claim of verified entry readiness or convergence until matched flight. Screening is not a flight guarantee: impulsive geometry, no aerodynamic perturbation validation, no calibrated worst-case attitude/engine margin or terrain-path guarantee. No deorbit redesign absent evidence, no changes to airless algorithms, chutes, general NaN handling or terminal controllers. Focused CI tests cover cutoff arithmetic, pulse selection and response trust; they cannot replace live phase/replay verification.

## 2.16.0-landing.20261126.duna-dynamics-m1

- Independent atmospheric dynamics model, bounded candidate screen and readiness classification; synthetic tests and CI.
- Preserve existing airless and atmospheric command paths. Test-only build; no flight-validation claim.

## 2.16.0-landing.20261126.duna-dynamics-m1.1

- Explicit correction preparation fit for both deadlines, fail-closed capture, optional passive lift adapter.
- Three further synthetic tests; no runtime activation.

## 2.16.0-landing.20261126.duna-dynamics-m1.2

- Added explicit SI force decomposition, deterministic atmosphere/aerodynamic adapters and strict scalar altitude lookup to the independent dynamics model.
- Added continuous bounded command-history replay with separate state and measured-force residuals, explicit invalid/unknown data, contact and cancellation handling; ten new synthetic tests.
- No live guidance change, no installation or flight-validation claim. Continue original M0/M1 gates.

## 2.16.0-landing.20261126.duna-dynamics-m1.3

- Add explicit terrain/support geometry and corridor lower-bound adapter with ordered contact/reserve events and unknown rejection.
- Add non-actuating KHV/FinalDescent reference replay, distinct PID/authority/engine states and bounded integration, twelve synthetic tests.
- Extend existing local evidence report to identify missing actual model inputs without repeating established evidence. No live guidance change, installation or plan gate waiver.

## 2.16.0-landing.20261126.duna-dynamics-m1.4

- Extend explicitly armed capture with bounded raw engine/nozzle/frame and initial curve observations. Correct neutral thrust provenance and fail closed for actual-vector closure in schema 1. No new guidance commands. Verification pending.

## 2.16.0-landing.20261126.duna-dynamics-m1.5

- Add bounded raw loaded aero/part/resource/controller observations and engine accepted-request percentage, session-local part identity, and candidate frame audit; reject mismatched live capture contexts. Preserve distinction between asynchronous raw fields and integrated force. No guidance change; CI verification pending.

## 2.16.0-landing.20261126.duna-dynamics-m1.6

- Add separately armed short Harmony observer for stock/MFI Integrate callbacks, pending force holder snapshots, route and Drive sequence observations, explicit caps, cleanup and offline probe audit. No MFI override, force mutation or solver-completion claim; verification pending.

## 2.16.0-landing.20261126.duna-dynamics-m1.7

- Replace environment-variable diagnostic arming with a persistent one-shot request file for normal CKAN launches; explicit readiness/rejection logs, eligible landing retry, persistent receipt and immediate header flush. Correct probe state callback label; no change to guidance. Add request tests and MVID to CI manifest.

## 2.16.0-landing.20261126.duna-planner-m2.0

- Implement bounded nonlinear feasible-range reference search for finite corrections, alternative braking times and terminal contact replay; rotating target miss, lexicographic fuel/accuracy ranking, generated correction seeds, sampled outcome and budget reporting. No changes to live landing control.

## 2.16.0-landing.20261126.duna-integrated-m3.0

- Implement nonlinear finite-time vector-guidance candidate search, clearance-dependent terminal velocity feedback, complete command history and independent finer-step replay.
- Add opt-in Duna snapshot solver preview to Landing Guidance, comparing A/B results without commanding the vessel. Approximate model only; existing live controllers unchanged.

## 2.16.0-landing.20261126.duna-integrated-m4.0

- Implement trajectory interpolation, live position/velocity tracking and explicit reserve/authority/clearance/expiry guard.
- Publish replayed model candidate with bounded sampled corridor checks and coast preparation; add isolated experimental Duna powered-descent engagement, single MechJeb ownership, direct attitude/throttle requests and explicit manual-abort/lifecycle release. No default guidance change; no automatic replanning or strategic deorbit.

## 2.16.0-landing.20261126.duna-integrated-m4.1

- Add measured-state prefix projection, warm-start bounded remaining-trajectory search, finer-step replay, generation and scheduled-epoch admission, shared incremental terrain screen and original reserve preservation.
- Refresh the active experimental trajectory asynchronously; keep tracking during solve and reject stale/unsafe replacements. Distinguish degraded off-target model candidate and cancel pending work on abort/teardown.

## 2.16.0-landing.20261126.duna-integrated-m4.2

- Add independent alternative local-site search, per-site terrain model, reserve-preserving ranking, finer-step replay, sampled corridor and footprint screen, bounded precomputation and strict current-state fallback admission. Preserve original target separately; no legacy fallback or claimed guaranteed recovery.

## 2.16.0-landing.20261126.duna-integrated-m5.0

- Add bounded orbital epoch, retrograde/normal finite burn and powered descent search with continuous fuel accounting, rotating target handoff and whole-history finer-step replay.
- Add explicit experimental orbital preview/execution, preburn alignment and deadline checks, phase-specific tracking, and altitude-scaled terrain mismatch screening. No default landing dispatch change.

## 2.16.0-landing.20261126.duna-integrated-m5.1

- Add independent radial stopping/delay/fuel screening, distinguish high-altitude warning from imminent hazard, latch explicit best-effort non-targeting response after loss of admitted trajectory.
- Add four closed-loop uncertainty scenarios, require sampled contact within limits before initial/replacement/contingency publication; share bounded terminal extension with flight control.

## 2.16.0-landing.20261126.duna-integrated-m5.2

- Implement constrained local fixed-time convex acceleration solver with scaled endpoint constraints, thrust ball/pointing cone projection, residual and cancellation gates.
- Add common A/B/C nonlinear replay and closed-loop uncertainty benchmark; publish per-fixture net8/net48 outcome, timing, miss, fuel and validation results in CI. No live solver switch.

## 2.16.0-landing.20261126.duna-integrated-m5.3

- Integrate exponential engine response analytically for impulse and mass flow, estimate local numerical errors with adaptive step doubling, and resolve contact before undefined atmosphere evaluation. Use endpoint rather than interval-averaged pointing rate. Preserve contact and reserve gates.

## 2.16.0-landing.20261126.duna-integrated-m5.4

- Adopt engineering-scale contact brackets (100mm / 10ms), refine only where safety classification or reserve order requires it, and trial tenfold coarser adaptive error budgets. Preserve physical acceptance and strict atmosphere-domain checks.

## 2.16.0-landing.20261127.duna-integrated-m5.5

- Try ranked alternatives under shared validation budgets before admitting a plan, including replan, fallback and whole orbital trajectory.
- Allow explicitly bounded terminal replay continuation inside the engineering contact band; preserve all physical and trajectory consistency checks.

## 2.16.0-landing.20261127.duna-integrated-m5.6

- Use ordinary Land at target to automatically select and engage integrated Duna orbital/descent planning; no separate preview/execute interaction; explicit cancellation and refusal.
- Obtain prospective drag from stock drag-cube geometry rather than instantaneous vacuum drag, and admit default parachute settings on craft without parachutes.
- Share numerical admission and lifecycle decisions with representative offline tests and record capture inputs in ordinary KSP logs.

## 2.16.0-landing.20261127.duna-integrated-m5.7

- Establish capture-epoch solver frame using body-fixed basis; transform live vectors, attitude commands, terrain/forecast/fallback locations and replacement corridors. Add recorded Eagle rotating-world regression.
- Sample atmospheric temperature using the existing world-position FlightGlobals API and log temperature knots; runtime density parity remains unverified.
- Document vessel-independent runtime capability acquisition and high-crossrange targeting requirements; actual Eagle end-to-end feasibility remains open.
- Add a geometric large-plane-change/deorbit seed with variable-mass burn duration, expanded bounded strategic burn/wait domain, and captured Eagle orbital search diagnostic. Retain all replay, uncertainty and target gates.

## 2.16.0-landing.20261127.duna-integrated-m5.8

- Acquire effective KSP base temperature, pressure, solar/latitude/axial curves and gas density coefficient; evaluate spatial temperature/density in fixed inertial coordinates, including uncertainty and contingency propagation. Sampled interpolation and fixed capture solar position remain approximations requiring reference parity verification.
- Add radial/great-circle finite-time endpoint feedback candidate with explicit spherical curvature and rotating-frame acceleration terms. Search alongside Cartesian candidate in high-crossrange orbital and direct descent; preserve candidate identity through replay, join and replanning.
- Add stock Duna reference fixture from KittopiaTech export, validate captured pressure keys, require safe targeted contact; add analytic target-periapsis transfer seeds and binary-search curve interpolation.
- Stabilise low-energy terminal descent and share its state-feedback law with the live/closed-loop tracker; reserve mixed-family search capacity for near-feasible trajectory refinement, preserving contact and replay gates.
- Preserve lateral tracking bounds during state-based terminal timing recovery; require targeted contact and protected reserve in every recorded Eagle robustness scenario.
- Exercise existing live orbital alignment, preburn and radial supervisor decisions during strategic closed-loop candidate validation; enforce all targeted strategic scenario misses.

## 2.16.0-landing.20261127.duna-integrated-m5.9

- Replace target-elevation sphere in production planning with a copied body-fixed terrain grid and fine local destination patches. Propagate and replay terrain-dependent clearance and contact, including scenario replay and fallback environment copies.
- Distinguish physically clear higher ground from collision or terminal terrain conflict; use actual clearance in live stopping/terminal control, automatically sample conflict terrain and replan from current orbital state. Preserve genuine terrain and reserve safety constraints.
- Use coarse orbital routing and demand-loaded approach terrain, fast high-clearance propagation, local terminal interpolation bounds and warm-started terrain-aware planning; record terrain samples for replay and add uneven-terrain acceptance.

## 2.16.0-landing.20261127.duna-integrated-m5.10

- Preserve the best fully validated targeted orbital trajectory when the optimisation deadline expires; explicit cancellation invalidates the result. Interactive planning stops at the first fully validated target rather than spending the remaining budget on fuel optimisation.
- Report deadline exhaustion without a validated result distinctly; retain terrain and live admission checks. Add deterministic completion-boundary regressions.

## 2.16.0-landing.20261127.duna-integrated-m5.11

- Honour existing Auto-warp checkbox for validated unpowered vacuum coast, using regular warp controller with bounded rate, atmosphere and next-thrust deadlines, and explicit unpacked 1x revalidation.
- Invalidate and recalculate a changed preburn orbit rather than skipping a burn; retain normal approach recovery, cancel owned warp on termination, and test analytic orbit crossing and actual post-warp state continuity.

## 2.16.0-landing.20261127.duna-integrated-m5.12

- Display planned deorbit countdown and duration, remaining burn time, next scheduled descent burn and touchdown countdown from the current accepted trajectory, updated using game UT during physics and warp. Display warp exit countdown during owned warp.
- Restore the existing Auto-warp checkbox during integrated planning and descent, removing the obsolete 1x-only label. Test merged powered intervals, countdown boundaries and game-time jumps.
- Preserve pending auto-warp during stock KSP alarm braking/message acknowledgement; resume only after the alarm closes, fresh-state revalidation and a remaining safe preparation window. Explicit user cancellation, loss of context and deadline expiry override alarm suspension.

## 2.16.0-landing.20261127.duna-integrated-m5.13

- Replace single-tick nominal burn alignment abort with actual feedback-command alignment interlock and attitude recovery. Replan from actual settled vessel state when sufficient vacuum coast remains; preserve reserve and body-fixed terrain. Committed short-coast paths suspend misaligned thrust and retain independent hazard control.
- Use the same actual-command throttle interlock in uncertainty replay. Log observed alignment and rate on recovery.
