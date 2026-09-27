# Landing Guidance V1 baselines

## V1 Minmus target-landing baseline (authoritative)

The authoritative source baseline for the Minmus landing that Dave described as
one of the most accurate he had experienced is Git commit
`d8ac3dd5` (`v1-terrain-profile-contact-20260925`).  It was the exact source
of the installed DLL with SHA-256
`D817AC5E5C0B97A30C4E7AF5597B0A8516385E116452113B27D0AEF39E145A63`.
Dave confirmed that this installed build landed at the selected Minmus target on
2026-09-25.  The same binary remains preserved at:

`C:\Users\Dave\Documents\KSP Backups\MechJeb2-LandingGuidanceV2\V1-effect-gain-20260925-173652\MechJeb2.dll`

The documentation commit `a8ff3530` records that result.  Its executable
parent `067ca75e` is the revert of an uninstalled experiment and has the same
runtime source as `d8ac3dd5` for this landing path.

Later "V1 Alpha recovery" builds are not substitutes for this exact baseline:
they retained a terrain-profile local-index change and a three-sample terrain
branch publication rule added after the successful flight.  Those changes may
be valid repairs, but they were not part of the proven Minmus artifact and must
be evaluated separately.

## Earlier restored V1 baseline

Git tag `v1-proven-c47337ff` (commit
`c47337fff7f655d8d9394f3b1ffdf00038d1089b`) remains the earlier controller
recovery that Dave confirmed performed plane change, geometric deorbit, course
correction, and landing.

V2 work must not modify the V1 controller, predictor, landing-autopilot steps,
throttle behavior, or reentry simulator. The V2 controls belong only in the
separate panel appended after the complete V1 Landing Guidance window.

On 2026-09-25 Dave separately authorised a V1 predictor correction after a
Minmus trace showed the predictor alternating between flat and mountain terrain
branches. That correction is recorded in
[LandingGuidanceV1-Predictor-Investigation-2026-09-25.md](LandingGuidanceV1-Predictor-Investigation-2026-09-25.md).
It does not change V1 landing phase ownership, actuator behaviour, or UI.
The earlier `c47337ff` artifact remains the preserved rollback baseline until
the corrected predictor has passed a fresh V1 runtime landing test.

Run `tools/Verify-V1Baseline.ps1` before every V2 commit and release build.
It compares the protected source files to the immutable V1 tag.

The matching user-confirmed compiled artifact is archived at:
`C:\Users\Dave\Documents\KSP Baselines\MechJeb2-Landing-Guidance-V1-Proven-c47337ff`

Its `MechJeb2.dll` SHA-256 is
`034921CF46F90494EDF25570D1EAAF6BC79C26FE39CE20FEDE13BEADCCA13F54`.
