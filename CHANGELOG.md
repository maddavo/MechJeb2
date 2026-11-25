# Changelog

## 2.16.0-landing.20261126.duna-brake-handoff

- Added bounded Duna-only powered braking commitment independent of ordinary prediction readiness, including pulse admission, preparation reserve, commitment latch, finite conservative thrust override and trace fields.
- Preserve nominal braking steering when reserve permits. Airless path, parachutes, ordinary predictor, correction consensus/caps and terminal controllers unchanged. Experimental; flight validation required.

## 2.16.0-landing.20261125.duna-m1

- Added explicitly armed, bounded Duna targeted powered landing observations, correlated live and ordinary predictor records, numerical interval/virtual impulse accounting, and geometric contact semantics.
- Added focused diagnostic tests and capture/build instructions. No guidance, chute or publication-law changes. Not built or tested in KSP in this environment.

Earlier landing development history: [V1 performance record](docs/LandingGuidanceV1-Commit-Performance-Record.md).
