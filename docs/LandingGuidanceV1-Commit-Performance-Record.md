# Landing Guidance V1 commit performance record

## Maintenance requirement

**Update this record for every commit that changes V1 Landing Guidance, its predictor, its landing-autopilot phases, or its associated V1 tests.** Add the commit tag, the intended issue, the evidenced runtime result, whether that exact version landed, and every subsequently identified issue. Do not infer a landing result from a successful build or automated test.

`v1-proven-c47337ff` is the last known good V1 landing version. Dave confirmed that it performed plane change, deorbit, course correction, and landing.

| Commit tag | Issue the version tries to fix | Did it fix the issue? | Did it land? | What other issues were identified or introduced after performance analysis? |
|---|---|---|---|---|
| `v1-proven-c47337ff` / `c47337ff` | Restored the known good Solver Basis V1 controller and UI. | **Yes**, baseline restored. | **Yes** — user confirmed. | Terrain-sensitive prediction instability later identified. |
| `1cb795d0` | Preserve the restored PlaneChange → DeorbitBurn handoff. | **Likely yes** in source; no separate runtime proof. | Not recorded. | Predictor and terrain issues remained. |
| `4543bc39` | Stabilize V1 prediction acceptance. | **Partial.** It did not solve terrain branch switching. | Not recorded. | Alternating flat/mountain terrain contacts still drove opposing corrections. |
| `b620c834` | Restore and lock the proven V1 baseline after experimental drift. | **Yes**, restoration/protection objective met. | Not separately recorded. | V1 still needed authorised predictor repairs. |
| `301e6326` | Stabilize V1 prediction consensus. | **Partial.** It did not resolve terrain-contact feedback. | Not recorded. | Airless terrain bisection later stalled Course Correction. |
| `09221f16` | Converge airless terrain predictions. | **No.** The terrain iteration remained unstable. | No. | Terrain-height bisection could hold Course Correction at zero throttle. |
| `3a76e596` | Bracket terrain contact. | **No.** The scalar terrain-root approach was unsuitable. | No. | Terrain ridges and lowlands produced incompatible solution branches. |
| `d8ac3dd5` | Replace terrain iteration with first real terrain contact along the sea-level trajectory. | **Yes.** Removed terrain feedback loop. | **Yes, later confirmed** by the terrain-profile landing record. | Public endpoint could still switch between stable terrain branches. |
| `f0411120` | Accept stable terrain profiles. | **No.** Reverted. | No. | The acceptance change was not suitable for the observed branch behaviour. |
| `067ca75e` | Revert `f0411120`. | **Yes.** Removed the unsuitable acceptance logic. | **Yes, later confirmed** by the terrain-profile landing record. | Course-correction pulse policy then became the main issue. |
| `a1370de0` | Scale course-correction pulses instead of fixed Δv limits. | **Partial.** Established effect-based scaling. | Not recorded. | Needed closer-to-target scaling and response adaptation. |
| `d4a76c87` | Limit correction by predicted target effect. | **Partial.** Better than a fixed Δv cap. | Not recorded. | Remote corrections still needed measured-response adaptation. |
| `a244dd1a` | Adapt remote correction gain from observed endpoint movement. | **Partial.** Added measured-response reduction/increase. | Not recorded. | Terrain branches could still make braking predictions oscillate. |
| `c2165a54` | Require a persistent terrain branch before publishing it. | **Partial.** Improved coast-phase branch stability. | Not recorded. | Smaller braking-phase branch changes still oscillated. |
| `48a8dc97` | Prevent low-gravity terminal hover climb. | **Partial.** Corrected the low-throttle calculation. | No recorded landing. | The initial implementation bypassed the user’s minimum-throttle control and horizontal drift remained. |
| `a608ef1e` | Release Course Correction after noisy post-burn solver samples. | **Partial.** Removed the direction-agreement deadlock. | Not recorded. | Very small effect-scaled pulses were immediately treated as complete. |
| `b6c4e4da` | Let small correction pulses actually deliver their Δv. | **Yes** for the premature-completion defect. | Not recorded. | Braking prediction still switched between nearby terrain branches. |
| `353b0535` | Stabilize braking-phase terrain consensus. | **Yes in focused tests**; no complete live landing recorded afterward. | Not recorded. | Terminal translation then showed lateral-drift behaviour. |
| `eaf2cc26` | Point terminal translation opposite measured horizontal velocity. | **Yes in focused tests.** | Not recorded. | Minimum-throttle handling had to be corrected to honour the user control. |
| `84d309d4` | Honour **Keep limited throttle over X%** during terminal translation. | **Yes** in source and focused tests. | Not recorded. | Terrain-profile contact then threw an index exception before predictor publication. |
| `92c36f04` | Fix absolute-versus-local terrain-profile index use. | **Yes** for the exception and reticle publication. | No. | Latest live run showed a separate deorbit-phase defect: burn started far too early, giving an approximately 147 km initial miss. |
| `76a2b80f` | Require the deorbit phase corridor; stop unsafe correction behaviour. | **Partial.** The first live test skipped deorbit because it incorrectly treated the target-normal diagnostic angle as a plane-alignment gate. | **No.** It remained in DeorbitBurn without firing. | The target-normal angle is near 90° for an equatorial target in an equatorial orbit; it must not veto ignition. |
| `6d55071c` | Restore the original phase-and-velocity ignition gate while retaining the early-burn shortcut removal. | **Passed offline tests and build; live result pending.** | Not yet tested. | The correction restores ignition for a 90° target-normal diagnostic angle. |

The successful terrain-profile landing was documented at `a8ff3530`; its code was the post-revert terrain-profile path ending at `067ca75e`. Each table row states the level of evidence available for that exact version.