# V1 Gamma — successful Mun flight baseline

Dave designated the existing tested build **V1 Gamma** on 30 September 2026
after confirming a successful landing. Git tag: `v1-gamma`. This designation
preserves the tested runtime source and DLL; it does not change runtime code,
build another DLL, or reinstall anything.

## Exact build and evidence

- Installed and archived DLL SHA-256:
  `7DC051FED8F66960508012E2AF4841300D70B6E7505E8627FB4DCB559D04A3CD`.
- Installed file:
  `C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\MechJeb2.dll`.
- Local preserved DLL, session capture and manifest:
  `MechJeb2/bin/Release/V1-Gamma/` (ignored build output).
- Capture session: `fa4c55a4fbbc48f79bf406bbbaebce53`.
- KSP logged `SUB_ORBITAL -> LANDED` at UT 24604309.0017258.
  The saved `Orbiter 7 ps` vessel is `LANDED`. Dave confirmed success.
- The installed build's existing acceptance records show 79 focused C# tests
  and 32 Python tests passed, and Release build zero warnings/errors. Those
  records predate this flight; no rebuild was needed to designate Gamma.

## Landing distance

| Position | Latitude, degrees | Longitude, degrees east |
|---|---:|---:|
| Captured target | 0.6741666666666666 | 23.473055555555554 |
| Actual landed vessel from `saves/ARSE Career 2/persistent.sfs` | 0.67503100292952223 | 23.218479038306732 |

Great-circle distance on the captured Mun radius of 200,000 m is
**888.583 m**, primarily west of the target. The last published forecast,
submission 132/version 42, was **8.33 m from the actual landed location**.
Successful landing does not establish target accuracy or the requested
0.5 m/s touchdown speed; exact contact velocity was not captured.

## Prediction cadence and deorbit overshoot

- 31 results published: 10 during LowDeorbitBurn, 21 during DecelerationBurn.
  No CourseCorrection phase occurred. No planning refresh occurred during
  KillHorizontalVelocity or FinalDescent.
- Deorbit publication interval: median **4.96 s**, maximum **9.98 s**.
  The configured planner refresh interval is 5 s. First publication latency
  was 2.96 s; later deorbit snapshot-to-publication ages were mostly 0.7–0.9 s.
- Submission 110 published at UT 24604081.079 with a 1,197 m miss beyond the
  target. Submission 111 published at UT 24604086.039 with an 859 m miss short
  of the target: a **2,056 m endpoint jump** between publications.
- Dave observed the deorbit burn overshoot because of cadence, and no course
  correction. The captured endpoint crossing and direct transition to braking
  corroborate those observations. The capture does not isolate cadence as the
  sole possible cause of the final target miss.
- The next predictor improvement should address timely deorbit refresh and
  endpoint delivery while preserving this successful baseline. No performance
  changes are implemented by this designation.

## Capture audit limitation

The reader correlated all 132 submissions without gaps. The existing lifecycle
audit reports 42 violations for the 21 direct braking forecasts because its
terminal-phase provenance allow-list omits the current safe-brake model string.
Those validation records carry `directForecast=true`; their worker stages are
DirectForecast and Terminal, not brake-time candidate searches. This is a
harness classification limitation, not evidence that the complete audit passed.
The raw audit report is preserved in the local Gamma manifest. Its classification
needs correcting before claiming a clean whole-session lifecycle audit.

The successful flight establishes forecast availability and landing for this
test. Remaining evidenced issues are coarse deorbit cadence, target miss, and
the audit classification above; it does not prove mountain-case performance.
