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
