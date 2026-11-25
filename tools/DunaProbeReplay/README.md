# Duna probe replay (M0)

Test-only standalone .NET 8 console project. Compiles the original DunaBrakingHandoff.cs directly into the executable; no game binaries, no Unity stubs, no live commands. Must not be deployed as a plugin.

## Inputs and command

From a clean worktree containing this project, verify the linked source blob against runtime commit `03120ee2f6248ce3a7399e4ad1f4afd99cbe3f53`:

    git hash-object --path=MechJeb2/LandingAutopilot/DunaBrakingHandoff.cs MechJeb2/LandingAutopilot/DunaBrakingHandoff.cs

Expected: `5be6c9c76e12c7fdbcc00d3efa0e5e0a42dff1ec`. Stop if different. The source hash check is manual, not enforced by the console application. Use genuine .NET 8 SDK (check dotnet --list-sdks). No dependency on KspDir.

First input: raw guidance_transition JSON object or {"sourceLine":123,"record":{...}} wrapper. If existing duna-probe.json uses a different envelope, export the unchanged raw record to this shape locally. Do not edit controllerDetail. Transition must equal dunaProbeUT within 1 microsecond; otherwise get same-epoch evidence before using this particular replay. The tool intentionally accepts only this observed pre-integration commitment case.

Second input: JSON object with required measured/supplied body parameters, with optional positionBCI_m at root:

    {
      "body": {
        "radius_m": null,
        "mu_m3_s2": null,
        "angularVelocity_rad_s": [null, null, null]
      },
      "positionBCI_m": null
    }

Nulls are placeholders, not usable inputs. Body values must be supplied explicitly; no nominal defaults. Rotation vector must be in same KSP world axes as the raw record at probe epoch. If unavailable, leave fixture incomplete; a separately labelled inferred-input run is useful but not independent runtime verification. positionBCI_m may be omitted/null: code reconstructs from unit Up, supplied radius and actual ASL, checks against logged clearance and reports reconstruction. Do not treat raw world CoM as BCI. If supplying BCI, use measured same-epoch state and provenance.

    dotnet run --project tools/DunaProbeReplay/DunaProbeReplay.csproj -c Release -- "$probeJson" "$bodyJson"

Capture stdout to a local replay-results.json (build diagnostics may precede JSON; run compiled DLL directly after dotnet build for clean JSON). Exit 0 matches both original calls and consistency checks, 1 mismatch, 2 input/build-time-independent application error. No interpretation as safe touchdown or validation of actual feasible braking timing. Tolerances 0.05 m, 1e-4 m/s, 1e-5 m/s² are provisional serialization/epoch checks; do not loosen to force a pass. Tool not compiled/run in chat; local execution required.

## Body and saved-vessel discovery

- Actual runtime body: use a read-only flight-thread diagnostic in an authorized local runtime, with active vessel mainBody or matching Duna from FlightGlobals.Bodies. Record `Radius`, `gravParameter`, `GeeASL`, `rotationPeriod`, `angularVelocity`, `atmosphereDepth`, `pqsController.radiusMax` and UT. Read `CoMD - mainBody.position`, `obt_velocity`, `getRFrmVel(CoMD)`, orientation and `FlightGlobals.getGeeForceAtPosition(CoMD)` from same tick if available. No flight or installation is authorized by this documentation. Stock body values may be baked in game data; managed assembly metadata alone is not authoritative body config. Current runtime body vector from a different epoch must be checked against historical velocity difference. For this trace the difference is consistent with omega along negative world Y, approximately -9.5900344839e-5 rad/s if that axis is independently established; not a measured vector.
- Inspect local GameData/ModuleManager.ConfigCache if present, and relevant GameData/**/*.cfg for Kopernicus Body/Properties/Atmosphere nodes or patches. Check installed physics mods and Physics.cfg (if present); absent cache or no matching patch does not establish stock numerical data. Do not publish private files. KSP code references are in MechJeb2/MechJebModuleLandingAutopilot.cs:177–185, VesselState.cs:499–544, CelestialBodyExtensions.cs:49. ReentrySimulation.cs:166–174 reads same mu and omega; SimCurves.cs:58–63 copies actual pressure and temperature curves including solar/latitude/axial terms. For M1 collect curve keyframes (time,value,inTangent,outTangent plus normalization and atmosphere flags) and force-model inputs from MechJeb2/ReentrySimulation/SimCurves.cs:36–63, ReentrySimulation.cs:639–715; no need for curves to replay this static rejection.
- Saved vessel instance in QS226 is authoritative. Do not substitute its referenced craft file. Extract ORBIT fields (including epoch and reference body), saved situation, mass/resources and part/module state, including saved per-engine limits/deployment/activation, crossfeed topology, action groups, staging, body/target and vessel attitude. Installed part definitions and runtime PartLoader/Part, PartResource/PartResourceLibrary, ModuleEngines/ModuleEnginesFX, drag cubes, shielding, actual mass/engine capability supplement but never replace saved instance overrides. Relevant MechJeb paths: VesselState.cs:419–432, 499–544 and engine-information update; FlyingSim/SimulatedVessel.cs:35–67 and FlyingSim/SimulatedPart.cs:63–77. Match saved orbit to trace at a common epoch using KSP orbit propagation/frame convention; a named vessel or confirmed save alone is not numerical match. No other vessel or save contents needed. Geometry/force/flow properties not recorded in legacy trace remain unknown until locally resolved.

## Additional read-only trace analysis

`tools/duna_m0_trace.py` reads the local contiguous {sourceLine,record} JSONL extract (no full save, no game installation, Python standard library only). Run:

    python tools/duna_m0_trace.py --self-test
    python tools/duna_m0_trace.py "$flightJsonl" > "$summaryJson"

Optional `--radius-m` takes independently established body radius. With no radius override it derives body radius from same-epoch recorded clearance + terrain radius - ASL. It fits omega from multiple directions via omega cross r = orbital - surface velocity, and reports residuals and rank failures. This is a trace-derived consistency test, not independent runtime body discovery; do not silently feed its inferred values to the exact-source replay and claim independent verification. For independent parameters use actual Duna runtime properties as documented above; search local effective configs first, report unknowns if no authorized read-only runtime access. No need to load a modified DLL. If an existing authorized local runtime inspection can read FlightGlobals.Bodies, use the actual body object. No assumption that stock body config is a .cfg file.

The summary retains the full phase transitions and computes forward-axis angular displacement per UT interval at 1x, rejecting duplicate/regressed UT, gaps >2 s and warp endpoints. This estimates sampled pointing motion, not actual yaw rate, roll or full angular velocity. Compare raw and limited forward equivalent force (mass times acceleration), actual throttle cap, attitude and slew before/after commitment. The stored dunaAuthority is not necessarily current state authority; summarize it only at same-epoch probe samples. First observed positive scalar forward thrust is not exact ignition time and generic VesselState uses neutralized gimbal directions. No per-engine allocation or torque demand in legacy trace. Synthetic self-test included; not executed in chat. Return summary JSON and exact-source replay results separately.
