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
