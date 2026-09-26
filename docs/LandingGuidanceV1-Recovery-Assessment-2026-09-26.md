# Landing Guidance V1 recovery assessment — 2026-09-26

## Decision

Do not install another V1 DLL until the last accepted V1 controller has been restored as the reference and each proposed correction has been checked against captured flight states. Do not rewrite V1’s strategic-deorbit method or continue adding gates, tolerance changes, or course-correction safeguards to the present angle-derived deorbit calculation.

`v1-proven-c47337ff` remains the last user-confirmed complete V1 landing. The current installed controller is `e102ff6f` plus documentation commit `a8097529`. Its live run completed Plane Change but safely refused deorbit because every candidate was more than 121 km from the target. No source work after `a8097529` has been installed in KSP.

## Control recovery recorded

`v1-control-recovery-20260926` restores `DeorbitBurn.cs` exactly to the source shared by the user-confirmed baseline and the successful terrain-profile landing build. The three strategic-deorbit commits are absent. The V1 predictor improvements remain: sea-level vacuum propagation, first real terrain contact, displaced-branch stability, effect-scaled course corrections, and fine-pulse completion. The focused information/control checks passed 27 of 27. This is a source recovery point only: no release DLL was built or installed, and no new live result is claimed.

## V1 Alpha plus narrowly scoped predictor repairs

`v1-alpha-terrain-repairs-20260926` is the next V1 validation candidate. It is V1 Alpha in full except for two predictor-information repairs: correct local indexing into the final terrain-profile slice and require three compatible observations before a materially displaced terrain branch can replace the published endpoint. It deliberately excludes later strategic-deorbit, course-correction policy, and terminal-control work. The source composition was checked by file hash, its terrain predictor test suite passed 11 of 11, and its release build completed with no warnings or errors.

## What changed after the confirmed landing

The confirmed landing was followed by 757 added lines and 75 removed lines in 11 V1 controller/predictor files. Most changes were response to real traces and several fixed their immediate defect. The problem was the integration method: a local repair was installed after focused policy tests, while the complete Plane Change → Deorbit → Correction → Coast → Braking → Terminal sequence had no captured-state replay harness.

| Version(s) | Change and result | Lesson |
|---|---|---|
| `1cb795d0` | Preserved the Plane Change → Deorbit handoff. No independent landing result was recorded. | Phase handoffs need a regression sequence, not source inspection alone. |
| `4543bc39`, `301e6326` | Added predictor acceptance and consensus. Both were partial; terrain branch switching remained. | A numerical acceptance rule cannot make incompatible terrain contacts represent one trajectory. |
| `09221f16`, `3a76e596` | Tried terrain-height convergence and scalar terrain bracketing. Both failed and could hold Course Correction at zero throttle. | Terrain must not be fed back as a scalar root into the vacuum trajectory. |
| `d8ac3dd5`, `067ca75e`, `a8ff3530` | Used the first actual terrain contact along the sea-level trajectory and reverted an unsuitable profile acceptance rule. A later landing succeeded. | This is the strongest predictor repair: solve the vacuum path once, then sample real terrain contact along it. Keep it isolated. |
| `a1370de0`, `d4a76c87`, `a244dd1a` | Replaced fixed correction pulse limits with predicted-surface-effect scaling and measured response adaptation. Partial results only. | Course correction must be assessed by both endpoint improvement and retained impact/terminal feasibility. Endpoint error alone is insufficient. |
| `c2165a54`, `353b0535`, `92c36f04` | Added terrain-branch persistence and fixed the terrain-profile local index exception. The exception and reticle publication were fixed; no complete landing followed. | Predictor publication needs captured terrain cases through every landing phase, especially after braking begins. |
| `48a8dc97`, `eaf2cc26`, `84d309d4` | Repaired Minmus terminal hover, horizontal-drift direction, and the user-selected minimum-throttle control. Each had focused evidence; no complete regression landing was recorded. | Terminal control is a distinct subsystem. Its tests must remain independent from strategic targeting changes. |
| `a608ef1e`, `b6c4e4da` | Fixed correction consensus deadlock and premature small-pulse completion. | A wait state needs a bounded exit and physical-delivery verification. |
| `76a2b80f` | Added a deorbit phase gate and correction safety. It incorrectly treated target-normal angle as a plane-alignment measure and prevented ignition. | Do not turn a diagnostic geometry value into authority without a recorded state proving its meaning. |
| `6d55071c` | Removed the incorrect normal-angle veto. It fired, but the first impact was about 185 km from target and later entered braking with a 129 km miss. | The deorbit burn itself was never validated against its predicted landing endpoint. |
| `e102ff6f` | Added a self-consistency and endpoint gate. It refused ignition because the constructed candidate had a 121.5 km miss and an almost 180° velocity-direction conflict. | Refusing an unsafe burn is necessary, but an angle-derived formula with an acceptance gate is not a planner. It cannot recover by itself. |

## Why the recent changes made behaviour appear worse

The early V1 controller would often continue after a bad internal estimate. That can look more active even when its targeting is invalid. The newer checks exposed bad states by refusing ignition or leaving correction before it removed all impact margin. Those refusals are safer than firing the known-wrong burn, but are not acceptable landing behaviour.

The recent regression is confined to the strategic-deorbit changes. The original V1 method demonstrably performed a complete landing at `v1-proven-c47337ff`; the later additions caused it to fire far too early or refuse ignition. Replacing the established V1 method with a new planner would exchange a known, usable controller for an unproven controller. Subsequent Course Correction should not be asked to repair an initial error of 100–185 km, but the first task is to restore the prior deorbit behaviour, then make only a measured correction to the identified error.

## Required development path

1. **Restore and freeze the accepted reference.** Use the exact controller source at `v1-proven-c47337ff` as the V1 reference. Exclude the uncommitted `DeorbitBurn.cs` experiment and revert the post-reference strategic-deorbit changes (`76a2b80f`, `6d55071c`, `e102ff6f`) from any future test candidate. Preserve the V1 UI unchanged.

2. **Separate baseline defects from regressions.** Reproduce the reported minor predictor behaviour using the reference version and captured flight states. A defect first seen only after a later commit is a regression to remove, not an invitation to replace V1’s landing method.

3. **Add a small replay harness around the existing deorbit calculation.** Capture its start state, commanded burn vector and magnitude, predicted first impact, and phase transition. The harness is observational: it proves the established calculation is preserved before any tuning is permitted.

4. **Make one narrowly scoped correction at a time.** For a confirmed baseline defect, alter only the term linked to the measured error. The test must show that the correction improves the original captured case while preserving the accepted reference case. No new phase, planner, candidate-search system, or authority model is in scope for V1.

5. **Use existing V1 behaviour as the integration target.** Check the complete existing sequence—Plane Change, Deorbit, Course Correction, Coast, Braking, Terminal—on captured states. Assert that the change does not alter V1 controls, phase ownership, warp rules, UI layout, or user settings outside the measured defect.

6. **Run KSP validation last.** A future DLL is appropriate only after the reference replay and the one-defect regression case both pass. The live run then verifies interface and environmental integration; it must not be the first evidence that the controller sequence works.

## Acceptance criteria for the next DLL

- No V1 UI or V2 source diff.
- The executable source is based on `v1-proven-c47337ff`, with only the explicitly reviewed, scoped change applied.
- The accepted reference replay retains its original deorbit timing, burn direction, and phase transition.
- The recorded 147 km, 185 km, and 121.5 km cases demonstrate that the post-reference strategic-deorbit regressions are absent; they do not introduce a replacement V1 planner.
- The reference replay and the specific defect regression test pass through the complete existing V1 phase sequence.
- Release build passes without warnings.
- The installed DLL is backed up and hash-verified only after KSP is closed.
