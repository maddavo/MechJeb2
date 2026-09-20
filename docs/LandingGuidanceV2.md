# Landing Guidance v2

**Status:** Proposed design for review  
**Scope:** A new, opt-in landing-guidance path.  It is not an incremental retuning of the existing landing controller.

## Purpose

Landing Guidance v2 provides efficient, safe, and understandable targeted landings for arbitrary KSP vessels.  It replaces continuous "predict-and-chase" behaviour with a plan-led, phase-based controller.

Its objectives, in order, are:

1. Preserve a safe touchdown state and sufficient landing reserve.
2. Reject an infeasible landing before altering the vessel trajectory.
3. Meet the selected target or an approved local divert target.
4. Minimise propellant use subject to the preceding constraints.
5. Support auto-warp during safe coast phases without issuing stale control commands.
6. Let the player visually inspect and adjust the landing site during final approach.

This design is informed by Apollo's separate braking, approach, and landing guidance phases, and the publicly documented SpaceX recovery sequence of boostback, entry/aerodynamic guidance, and landing burns.  It deliberately does not claim to reproduce proprietary flight software.

## Design principles

- The predictor is an estimator, never a direct steering command.
- Every estimate is derived from one immutable vessel/body snapshot.  A prior prediction must never become input to the next prediction.
- Each phase has an explicit objective, owner, entry gate, exit gate, and allowed control authority.
- A strategic deorbit burn is a finite, counted, feed-forward burn.  It must stop in a defined long-side corridor and must never deliberately cross to the short side of the target.
- Auto-warp is a phase-manager service, not a side effect invoked by individual controllers.
- The player-facing target remains semantically stable: red is always the desired target and blue is always the predicted touchdown.
- User retargeting is local, safety-checked, and paid for from a protected terminal-divert reserve.

## Architecture

```
Immutable guidance snapshot
        |
        v
Deterministic estimator ----> estimate, confidence, convergence data
        |
        v
Landing planner -----------> feasible plan, delta-V budget, phase events
        |
        +--------------------> warp scheduler
        |
        v
Phase manager -------------> trajectory tracker and authority allocator
        |
        v
Attitude, throttle, RCS, and staging interfaces
```

### Core data objects

`GuidanceSnapshot`

- UT and snapshot version.
- Vessel position, velocity, mass, available engines, propellant, thrust, minimum throttle, attitude state, and control authority.
- Body gravity, rotation, atmosphere, terrain/query context, and target position.
- Current rails/physics and warp state.

`LandingEstimate`

- Snapshot version used to create it.
- Outcome: no impact, impact, atmospheric entry, invalid, or non-converged.
- Predicted endpoint, event UT, velocity, terrain result, uncertainty, and residual/convergence information.
- No previous estimate may affect its calculation.

`LandingPlan`

- Current phase, planned phase-event UTs, burn vectors, and planned delta-V.
- Long-side/cross-range corridor.
- Braking-entry state and terminal trajectory reference.
- Usable delta-V, allocated trim delta-V, terminal-divert reserve, and landing margin.
- Feasibility state and explicit rejection reason.

`TargetState`

- Original preflight target, retained as reference.
- Active red target.
- Blue predicted touchdown from the current estimate.
- Whether the one-time visual target rebase has occurred.

## Estimation and planning

### Airless bodies

Airless-body targeting uses a deterministic conic/rotating-body impact calculation as the primary estimator.  It uses terrain only as a bounded, repeatable refinement.  It must return `NoImpact` explicitly rather than placeholder coordinates.

The planner searches feasible candidates for burn epoch, burn vector, alignment, and terminal-braking state.  It selects the lowest-total-delta-V candidate that satisfies all constraints.

The strategic-deorbit acceptance condition includes a signed downrange constraint:

```
predicted impact must be on the long side of the active target
and within the permitted cross-range and long-side corridor
```

If no candidate satisfies that condition, the system waits, replans at a defined event, or reports no plan.  It must not enter correction without a valid impact trajectory.

### Atmospheric bodies

Atmospheric prediction remains a model-based estimator with explicit uncertainty.  The planner targets a robust entry corridor and retains sufficient powered reserve for the terminal phase.  It must not present atmospheric estimates as exact touchdown promises.

## Delta-V feasibility and optimisation

Before ignition, guidance calculates:

```
available usable delta-V
- alignment delta-V
- strategic deorbit delta-V
- allocated trim delta-V
- braking and terminal-descent delta-V
- terminal-divert reserve
- engine/settling contingency
= landing margin
```

Usable delta-V is based on actually available engines, propellants, current mass, engine performance in the current environment, minimum-throttle/restart limitations, and any authorised staging path.

The reserve is calculated from estimator uncertainty, gravity losses, engine/attitude response, local divert capability, and touchdown safety.  It is not a fixed percentage.

The default policy is fail closed: if the landing margin is negative, the command is rejected before the vehicle burns.  The user receives the calculated shortfall and cause.

Course corrections may consume only the allocated trim budget.  A correction is accepted only when it improves the objective, remains long-side, preserves the protected reserve, and does not reverse the prior planned correction direction without an explicit re-plan.

## Guidance phases and auto-warp

| Phase | Objective | Warp policy | Control policy |
|---|---|---|---|
| Preflight | Build and validate the plan | Never | No vessel command |
| Alignment | Achieve planned geometry | Warp to burn gate only | Counted alignment burn |
| Strategic deorbit | Establish long-side impact corridor | Warp to ignition only | Counted feed-forward burn |
| Coast / energy management | Preserve plan and approach next event | Airless coast only | No continuous target chasing |
| Bounded trim | Remove a validated residual | Never | Limited validated burn |
| Braking approach | Reach visual-assessment state | Never | Track reference profile |
| Visual assessment | Human inspection and local target adjustment | Never | Hold or slow approach if feasible |
| Terminal divert | Reach active red target | Never | Bounded local trajectory |
| Velocity null / touchdown | Safe landing | Never | Velocity, attitude, clearance priority |

The warp scheduler owns all warp transitions:

```
warp to ignition minus attitude/engine-settle margin
-> settle at 1x
-> execute counted burn
-> warp to the next phase boundary minus revalidation margin
-> take a fresh snapshot and revalidate
```

On every warp exit, all estimates derived from a pre-warp snapshot are discarded.  No estimator output, RCS command, or burn command may cross a warp boundary without a fresh snapshot and phase revalidation.

Warp is prohibited during burns, meaningful atmospheric flight/drag, parachute operation, visual assessment, terminal divert, and touchdown.

## Target semantics and visual landing adjustment

The visual model is constant throughout the landing:

```
Red  = active desired landing target
Blue = predicted touchdown from the current trajectory
N/S/E/W controls always modify Red
```

The system may enter visual assessment only after auto-warp has ended, the craft is within visual range of the terrain, and blue is already within the configured pre-visual accuracy corridor of the original target.

At entry, guidance performs one target rebase:

```
active red target = current blue predicted touchdown
```

The original target is preserved as a subdued reference/reset marker.  The target must not continuously follow blue after this event.

N/S/E/W inputs now make a local, player-requested movement of red.  Blue remains an independent prediction.  The terminal-divert planner changes attitude and thrust to move blue toward red, subject to reserve and feasibility gates.

If blue is not close enough to the original target at the visual gate, guidance must continue earlier targeting or report a failure.  It must not silently rebase a materially inaccurate approach.

### Local terrain assessment

Long-range terrain data may reject obviously impossible targets but cannot certify a safe site.  Meaningful assessment occurs only in the local visual phase, within several hundred metres.

For an active red target and nearby user-selected alternatives, assessment checks conservative vessel footprint slope, local roughness, clearance, water/collision exclusion, and reachable divert cost.  It may suggest candidate sites but never silently changes the active target.

A high-thrust craft may offer **Hold for site selection** only if it has a demonstrated safe hold and sufficient reserve.  Other vessels use a controlled slow approach with a calculated redesignation window.

Each requested target movement is accepted only if the local divert trajectory is feasible and leaves positive protected reserve.  Otherwise the UI reports the reason and leaves red unchanged.

## User interface

Before activation, the player sees a Landing Plan panel containing feasibility, usable delta-V, phase budgets, reserve, margin, and any rejection reason.

Example:

```
LANDING PLAN: FEASIBLE
Available usable delta-V: 1042 m/s
Planned landing delta-V:   816 m/s
Protected reserve:         126 m/s
Landing margin:            100 m/s
```

During coast the panel identifies the next event and warp state.  During visual assessment it identifies the rebase, reachable divert footprint, remaining divert reserve, and local terrain assessment.  N/S/E/W retain their familiar purpose at every phase.

## Safety invariants

- No command may use an estimate from a stale pre-warp, pre-burn, or superseded snapshot.
- No correction phase may run without a valid impact estimate.
- No strategic deorbit plan may intentionally end short of the long-side corridor.
- No trim may consume protected terminal-divert reserve or reverse direction without an explicit re-plan.
- No visual rebase occurs outside the pre-visual accuracy corridor.
- No user divert is accepted unless its landing margin remains positive.
- Any infeasible preflight plan is rejected before engines are commanded.

## Diagnostics and validation

V2 uses a structured trace separate from general `KSP.log`.  It records snapshots, estimates, confidence/residuals, candidate plans, rejected-plan reasons, delta-V budgets, warp transitions, target-rebase/divert events, commanded and delivered delta-V, and phase transitions.

Validation must demonstrate:

- repeatable estimator results for identical input snapshots;
- no stale command across warp transitions;
- long-side strategic deorbit behaviour;
- bounded trim expenditure and no target-crossing correction;
- equivalent safe outcomes with auto-warp enabled and disabled;
- correct target rebase/NSEW behaviour in visual mode;
- feasibility rejection for insufficient delta-V;
- airless-body coverage for Mun, Minmus, and Tylo, followed by atmospheric coverage for Kerbin and Duna.

## Delivery strategy

1. Preserve the current experimental landing work as evidence and rollback material.
2. Implement V2 in a separate branch/worktree from a known baseline.
3. Add the data model, structured trace, deterministic airless estimator, and preflight panel first.
4. Implement unified airless strategic deorbit, coast/warp, braking, and terminal tracking.
5. Add visual target rebase and local terminal-divert capability.
6. Implement and validate atmospheric energy-management guidance.
7. Keep V2 opt-in until the validation matrix passes; only then consider it as the default landing path.

## References

- NASA, *Apollo 11 Mission Report*, lunar descent trajectory and guidance phases.
  https://www.nasa.gov/wp-content/uploads/static/apollo50th/pdf/A11_MissionReport.pdf
- NASA, *Apollo Lunar Descent and Ascent Trajectories*, P63/P64/P65/P66 phase and target behaviour.
  https://www.nasa.gov/wp-content/uploads/static/history/alsj/nasa58040.pdf
- SpaceX, *Falcon User's Guide*, publicly documented boostback, entry, aerodynamic-guidance, and landing sequence.
  https://www.spacex.com/assets/media/falcon-users-guide-2025-05-09.pdf
- NASA/JPL, *JPL, Masten Testing New Precision Landing Software*, G-FOLD context.
  https://www.jpl.nasa.gov/news/jpl-masten-testing-new-precision-landing-software/
