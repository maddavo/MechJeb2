# Proven Landing Guidance V1 baseline

The V1 landing controller is fixed at Git tag `v1-proven-c47337ff`
(commit `c47337fff7f655d8d9394f3b1ffdf00038d1089b`). Dave confirmed that
build performs plane change, geometric deorbit, course correction, and landing.

V2 work must not modify the V1 controller, predictor, landing-autopilot steps,
throttle behavior, or reentry simulator. The V2 controls belong only in the
separate panel appended after the complete V1 Landing Guidance window.

Run `tools/Verify-V1Baseline.ps1` before every V2 commit and release build.
It compares the protected source files to the immutable V1 tag.

The matching user-confirmed compiled artifact is archived at:
`C:\Users\Dave\Documents\KSP Baselines\MechJeb2-Landing-Guidance-V1-Proven-c47337ff`

Its `MechJeb2.dll` SHA-256 is
`034921CF46F90494EDF25570D1EAAF6BC79C26FE39CE20FEDE13BEADCCA13F54`.
