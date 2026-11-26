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

M1.7 diagnostic activation uses an explicit one-shot bounded request file in existing PluginData, fail-closed parsing and persistent consumption before capture. No environment dependency; initial ineligibility retries only during the requested targeted landing; readiness and failure reasons logged. Existing guidance and acceptance gates unchanged.

M2.0 adds offline reference finite correction/ignition/terminal candidate search with sampled feasible outcomes, rotating target residual, budget/cancellation and explicit model/capability inputs. No claim of actual reachability or new live controller. Original integrated plan remains authoritative; M2 validation and later milestones open.

M3.0 adds finite-time vector-guided shooting and clearance-dependent terminal feedback, command history and finer replay, plus explicit reduced-order KSP snapshot preview comparing reference timing search with direct trajectories. No actuation; snapshot fidelity and solver-selection acceptance remain open.

M4.0 introduces an explicit experimental Duna descent step and trajectory tracker. No ordinary landing dispatch changes; use only a finer-replayed target-tolerance model plan with sampled terrain, current-state/ownership checks and explicit user engagement. Failures release control rather than silently fall back. Reduced-order model and sampled terrain are not physical safety validation; no automatic replanning or strategic deorbit yet. Original acceptance gates remain.

M4.1 adds asynchronous receding-horizon replacement of the experimental active trajectory. Preserve the original absolute reserve, project the measured state to a scheduled handoff under current control, warm-start search, finer-step replay and sampled terrain check, admit only matching current context and close handoff state. Off-target candidate admitted only on loss of old tracking at handoff. This is not a guaranteed emergency fallback; no default guidance changes.

M4.2 adds independently sampled local-site contingency generation for the experimental descent, with own terrain radius, original reserve, full nonlinear and finer-step replay and sampled corridor/footprint review. Only an already-screened, current-state-compatible contingency can replace a failed nominal tracker. No guaranteed coverage or automatic best-effort hazard control when neither plan is valid. Original target remains distinct from contingency site.

M5.0 adds bounded strategic orbit/finite vector deorbit and subsequent powered descent search using one continuous mass/state/command history and finer whole-history replay. Explicit orbital preview and experimental execution only; low Duna bound orbit, 1x, no irreversible burn on unadmitted plan. Original target and reserve preserved. Strategic geometry/solver and actual flight parity remain experimental; ordinary Land at target unchanged.

M5.1 adds independent radial recoverability warning/hazard screening and explicit latched best-effort response after plan loss in experimental descent. Require four sampled closed-loop uncertainty contacts for candidate publication in addition to existing finer nominal replay. These are reduced-order screens, not certified recoverability or measured uncertainty. Ordinary landing unchanged.

M5.2 adds a non-actuating local convex minimum-energy acceleration prototype and common A/B/C benchmark with recorded command histories and shared replay/scenario validator. C is not a full SOCP/SCvx implementation. B remains finite-time nonlinear shooting. No production solver selection or live control changes.

M5.3 corrects shared atmospheric integration: analytical first-order throttle moments, adaptive local-error step doubling, chronological surface/reserve event subdivision, endpoint angular rate and strict model domain; touchdown limits unchanged. Numerical validation remains mandatory.

M5.4 numerical precision policy: use the coarsest validated practical resolution for gameplay; refine only when material to feasibility, safety or objective. Surface-event brackets target 100 mm and 10 ms, require a real propagated crossing and preserve contact limits and reserve/unknown-data gates. Adaptive error budgets are subject to replay and runtime verification. Do not quantise the actual geometry or exact lineage identifiers.

M5.5: rank sampled feasible candidates, publish only after finer replay and sampled uncertainty validation, try alternatives within shared bounded budgets/deadline. Explicit final terminal-history replay continuation <=0.5s within 100mm contact band requires physically propagated contact with unchanged safety, reserve and consistency checks. Generic replay has no implicit extension.
