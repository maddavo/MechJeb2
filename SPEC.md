# Landing guidance specification addendum

Existing refined V1 behavior is authoritative at the pinned source baseline; this addendum defines only Duna M1. See [Duna-M1](docs/Duna-M1.md) for the capture contract and constraints. Diagnostic-only; no new guidance authorization, no safe-touchdown claim, no airless or chute behavior change.

## Duna braking handoff (2.16.0-landing.20261126.duna-brake-handoff)

Experimental atmospheric-only change; M1 diagnostic constraints still apply to capture. See [Duna braking handoff](docs/Duna-Braking-Handoff.md) for the new scope, decision, trace contract, limits and acceptance criteria. No airless, chute, targeting or general NaN repair in this milestone.

## Dual-window candidate (2026-11-26)

Version: `2.16.0-landing.20261126.duna-dual-window`. Duna targeted powered/no-chute only. Independently screen the descending atmospheric interface intersection using runtime orbit and RealMaxAtmosphereAltitude, reserving correction tail, measured/fallback slew and settling, warp overrun and margin. The existing no-drag-credit braking delay probe reserves correction tail separately and retains engine preparation. Entry commitment coasts surface-retrograde without forecast, blocks re-entry to CourseCorrection, and keeps independent braking guard active. Braking commitment bypasses forecast timing; fresh optional braking steering remains available below the entry interface when reserve allows. Larger correction commands (initial cap 4 m/s, up to 8 m/s after measured scalar error improvement beyond estimated coast drift; near-target and reversal limits) are screened against geometric response and both current and instantaneously perturbed entry/braking projections. Retain two accepted post-tail directions; require new pulse forecast age <=5 s. Trace predicted entry/cutoff, commitment reason/miss (unknown when stale), actual entry alignment/residual with bracketing samples, pulse request/delivery/trust. No claim of verified entry readiness or convergence until matched flight. Screening is not a flight guarantee: impulsive geometry, no aerodynamic perturbation validation, no calibrated worst-case attitude/engine margin or terrain-path guarantee. No deorbit redesign absent evidence, no changes to airless algorithms, chutes, general NaN handling or terminal controllers. Focused CI tests cover cutoff arithmetic, pulse selection and response trust; they cannot replace live phase/replay verification.

## Proposed integrated landing guidance (2026-11-26)

[Implementation plan](docs/Integrated-Landing-Guidance-Plan.md) is the reference for the proposed next development direction: constrained target-aware powered trajectory optimisation, consistent prediction/execution, atmospheric drag/passive lift without required aerodynamic steering, independent readiness and safety constraints, and capability-based compatibility. It supersedes incremental Duna tuning as a proposed development direction, not the current runtime contract. Preserve prior airless behavior. Minimum throttle limiter support excluded. Implementation and deployment are separate authorisations.

## Independent M1 atmospheric dynamics (2026-11-26)

Authorized non-actuating reference implementation: [M1 scope and contracts](docs/Integrated-Landing-Dynamics-M1.md). SI finite-burn propagation, pressure/Isp and mass, restricted verified torque-balanced engines, bounded pointing and lag, synthetic spherical contact, finite correction and independent entry deadline classification. No static hover pre-rejection; two failed candidates do not prove an ignition deadline. Capture is not touchdown. Unsupported actual data remains unknown. No live activation or changes to refined airless/legacy guidance. M0 evidence gaps remain; no flight validation.

M1.1 adds explicit tested braking-preparation fit, an optional passive lift input adapter, and fail-closed low-speed capture. These remain synthetic-model checks, not a live safety authorization.

M1.2 (`2.16.0-landing.20261126.duna-dynamics-m1.2`): non-actuating shared force decomposition, explicit time/position/attained-direction atmosphere/aerodynamic adapters and bounded continuous command replay with independent observed-state residuals. Missing or invalid capability remains unknown; no invented data, no observation resets or implicit coasts; modeled geometric contact, completed history and verified real touchdown remain distinct. See [M1 contracts](docs/Integrated-Landing-Dynamics-M1.md). Binding original plan and existing gates remain unchanged. No runtime dispatch change.

M1.3 (`2.16.0-landing.20261126.duna-dynamics-m1.3`): offline explicit surface/contact model with independent physical/control clearance, conservative interval bound and unknown terrain rejection; source-derived KHV/FinalDescent replay including distinct control/PID/actual-throttle states, bounded steps and explicit contact. Caller-supplied model fidelity remains unvalidated; no live controller dispatch change or flight safety claim. See [M1 contracts](docs/Integrated-Landing-Dynamics-M1.md).

M1.4 adds explicitly armed bounded raw runtime observations and rejects neutral thrust as actual measured force. No new actuation, frame/force epoch parity or flight validation. See [M1 contract](docs/Integrated-Landing-Dynamics-M1.md).

M1.5 extends explicitly armed passive capture of raw loaded model/controller and resource fields; validates candidate frame consistency only. No actuation or verified force-epoch parity. See [M1.5 contract](docs/Integrated-Landing-Dynamics-M1.md).

M1.6: optional bounded passive callback calibration probe; no MFI override or force mutation, no certified solver completion, no actual-vector force closure. Existing non-actuating plan and separate deployment gates retained. See [M1.6 contract](docs/Integrated-Landing-Dynamics-M1.md).
