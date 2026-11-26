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
