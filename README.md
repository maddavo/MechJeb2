# MechJeb2

Anatid Robotics and Multiversal Mechatronics proudly presents the first flight assistant autopilot: MechJeb

MechJeb2 is a mod for the game Kerbal Space Program. To learn how to use it, [visit the wiki][wiki]. For more
info, [visit this KSP forum post][post].

[wiki]: https://github.com/MuMech/MechJeb2/wiki

[post]: http://forum.kerbalspaceprogram.com/index.php?/topic/154834-122-anatid-robotics-mumech-mechjeb-autopilot-260-12-dec-2016/

## Table of Contents

- [MechJeb2](#mechjeb2)
    - [Table of Contents](#table-of-contents)
    - [Install](#install)
        - [Manual install](#manual-install)
            - [Download](#download)
            - [Unpack](#unpack)
        - [Via CKAN](#via-ckan)
            - [Development version of Mechjeb](#development-version-of-mechjeb)
    - [Common Issues](#common-issues)
    - [Development](#development)
        - [Maintainers](#maintainers)
        - [Code Standards](#code-standards)
        - [Third-party libraries](#third-party-libraries)
        - [Build](#build)
            - [Linux](#linux)
            - [Windows](#windows)
    - [License](#license)

## Install

### Manual install

#### Download

Download from Jenkins:
<https://ksp.sarbian.com/jenkins/job/MechJeb2-Release/>

#### Unpack

Unzip the zip in KSP GameData directory. You should have something that looks like that :

    Kerbal Space Program
    -- GameData
       -- MechJeb2
          -- Bundles
          -- Icons
          -- Localization
          -- Parts
          -- Plugins

### Via CKAN

CKAN has all the release of MechJeb, just install it as usual.

#### Development version of Mechjeb

If you want the unstable dev version of MechJeb then :

1. Open CKAN settings (Settings => CKAN Settings)
2. Press the New button
3. Select the MechJeb-dev line, click OK and exit the options.
4. Refresh
5. Select "Mechjeb2 - DEV RELEASE" in the list
6. Then "Go to Change" to install

## Common Issues

1. Why is the Mechjeb menu not showing?

   Make sure you have the part on your ship (AR202 case in the Control section).

2. (Windows) I cannot find Mechjeb anywhere, there aren't even parts in the R&D facility!

   Some Windows protection and anti-virus software can sometimes block KSP from loading MechJeb.
   You should install KSP outside the `C:\Program Files (x86)\`
   directory. [Steam has an option to change the install directory](https://support.steampowered.com/kb_article.php?ref=7710-tdlc-0426)
   of a game or you can just copy the directory somewhere else.

3. Why is some Mechjeb function not available?

   Science and career mode requires you to unlock some specific node in the Research and Development tree.
   You also may need to upgrade the tracking station to level 2 (game code restriction we can't do much about).

4. How do I report a bug?

   Check if your problem has already been reported: <https://github.com/MuMech/MechJeb2/issues>  
   If you found a problem which is similar to yours, feel free to add more information to the existing issue.

   **If you cannot find the problem**, get
   a [log](https://forum.kerbalspaceprogram.com/index.php?/topic/83212-how-to-get-support-read-first/#Logs) and create a
   new issue with a descriptive title of the problem.

## Development

### Maintainers

- [@sarbian](https://github.com/sarbian)
- [@lamont-granquist](https://github.com/lamont-granquist)

### Code Standards

1. [No var](https://docs.microsoft.com/en-us/visualstudio/ide/reference/convert-var-to-explicit-type): use explicit
   types.
2. Prefer single lines, when possible: especially if-else blocks!
3. [No null-conditional operators](https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/operators/member-access-operators#null-conditional-operators--and-):
   Unity 4.x has
   a [custom == for checking object nulls](https://blog.unity.com/technology/custom-operator-should-we-keep-it).
4. Assembly version needs to remain at 2.5.1.0; file version can be incremented.

### Third-party libraries

- [ALGLIB](https://www.alglib.net/)
- [NSubstitute](https://nsubstitute.github.io/)
- [xunit](https://xunit.net/)

### Build

#### Linux

The project uses Mono and Make to build the addon, make sure you have both installed. You need Nuget to download the external dependencies.

You can also use the [flake.nix](./flake.nix) with direnv or by runnning `nix develop` to set up the development environment.

1. (optional) Set your KSP directory

```sh
export KSPDIR="${XDG_DATA_HOME}/Steam/SteamApps/common/Kerbal Space Program"
```

2. Fetch external packages

```
nuget restore
```

3. Build the mod

```sh
make build
```

`make install` is disabled in this branch. The only authorised installation is the
Windows DLL operation described below.

#### Windows

1. Install the version of Unity that KSP uses ( Currently 2019.2.2f1 )

2. Configure your system environment variables and add:

- KSPDIR set to exactly **C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program**
- MONO set to the path of Unity current mono.exe ( usually C:\Program
  Files\Unity\Hub\Editor\2019.2.2f1\Editor\Data\MonoBleedingEdge\bin\mono.exe )
- PDB2MDB set to the path of pdb2mdb.exe ( usually **C:\Program
  Files\Unity\Hub\Editor\2019.2.2f1\Editor\Data\MonoBleedingEdge\lib\mono\4.5\pdb2mdb.exe** )

3. Load MechJeb2.sln and open the properties of the MechJeb2 project (Right-Click=>properties). In the "Reference Path"
   section add the KSP libs folder to the list ( usually **C:\Program Files (x86)\Steam\SteamApps\Common\Kerbal Space
   Program\KSP_x64_Data\Managed** )

4. Repeat step 3 for the MechJebLib, MechJebLibBindings, and MechJebLibTest projects.

5. Perform `nuget restore` to get external dependencies such as JetBrains.Annotations.

Building writes to repository build-output directories only. It does not install
files into KSP. After a Release build, close `KSP_x64` and run an explicit
read-only validation, then install only `MechJeb2.dll`:

```powershell
$env:KSPDIR = 'C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program'
./tools/Install-MechJeb2.ps1 -ValidateOnly
./tools/Install-MechJeb2.ps1
```

The script refuses a missing or different `KSPDIR`, verifies KSP is closed,
backs up the existing DLL with a timestamped manifest, and verifies the installed
DLL by SHA-256. Companion DLLs are never included. Replacing any companion DLL
requires a separate review of its identity, installed and source versions, and
specific dependency need, followed by explicit approval.

## License

Licensed under the [GNU General Public License, Version 3](LICENSE.md).

Portions (in the "MechJebLib" directory) are placed in the public domain and are documented in
the affected source code headers.


### Duna M1 diagnostic build

`2.16.0-landing.20261125.duna-m1`: opt-in bounded Duna targeted powered landing diagnostics. See [Duna M1 capture, build and test instructions](docs/Duna-M1.md). No change to guidance commands or installation policy.

### Duna braking handoff test candidate

`2.16.0-landing.20261126.duna-brake-handoff`: see [Duna braking handoff](docs/Duna-Braking-Handoff.md) for the narrowly scoped experimental guidance change, configuration, diagnostics, limits and matched-flight validation. Build/test output is supplied only through branch-scoped GitHub Actions artifacts; no release or installation implied.

## Dual-window candidate (2026-11-26)

Version: `2.16.0-landing.20261126.duna-dual-window`. Duna targeted powered/no-chute only. Independently screen the descending atmospheric interface intersection using runtime orbit and RealMaxAtmosphereAltitude, reserving correction tail, measured/fallback slew and settling, warp overrun and margin. The existing no-drag-credit braking delay probe reserves correction tail separately and retains engine preparation. Entry commitment coasts surface-retrograde without forecast, blocks re-entry to CourseCorrection, and keeps independent braking guard active. Braking commitment bypasses forecast timing; fresh optional braking steering remains available below the entry interface when reserve allows. Larger correction commands (initial cap 4 m/s, up to 8 m/s after measured scalar error improvement beyond estimated coast drift; near-target and reversal limits) are screened against geometric response and both current and instantaneously perturbed entry/braking projections. Retain two accepted post-tail directions; require new pulse forecast age <=5 s. Trace predicted entry/cutoff, commitment reason/miss (unknown when stale), actual entry alignment/residual with bracketing samples, pulse request/delivery/trust. No claim of verified entry readiness or convergence until matched flight. Screening is not a flight guarantee: impulsive geometry, no aerodynamic perturbation validation, no calibrated worst-case attitude/engine margin or terrain-path guarantee. No deorbit redesign absent evidence, no changes to airless algorithms, chutes, general NaN handling or terminal controllers. Focused CI tests cover cutoff arithmetic, pulse selection and response trust; they cannot replace live phase/replay verification.

## Landing guidance development (fork)

See the [integrated landing guidance implementation plan](docs/Integrated-Landing-Guidance-Plan.md) for the proposed next architecture and staged validation. This is a planning document, not an implemented or released feature; current executable behavior remains unchanged.

### Independent Duna dynamics M1 test build

`2.16.0-landing.20261126.duna-dynamics-m1.1` adds a non-actuating atmospheric reference model and synthetic tests. See [M1 scope and limits](docs/Integrated-Landing-Dynamics-M1.md). Not a replacement live landing controller or flight-validated release.

## Integrated landing guidance development

Current branch experimental version: `2.16.0-landing.20261126.duna-dynamics-m1.3`. This is a non-actuating engineering increment, not a new landing controller or a deployable flight fix. See the [binding implementation plan](docs/Integrated-Landing-Guidance-Plan.md) and [M1 dynamics/replay contract](docs/Integrated-Landing-Dynamics-M1.md). Run the independent model tests with `dotnet test tools/DunaDynamicsTests/DunaDynamicsTests.csproj -c Release`; real KSP-reference and flight validation remain required.

Integrated landing research branch: `2.16.0-landing.20261126.duna-dynamics-m1.4` — passive diagnostic research, not an authorized landing fix or release. See [M1 contracts](docs/Integrated-Landing-Dynamics-M1.md).

Current integrated landing research increment: `2.16.0-landing.20261126.duna-dynamics-m1.5` (passive raw observations, no flight validation or install authorization).

Current integrated landing research increment: `2.16.0-landing.20261126.duna-dynamics-m1.6` (optional bounded integrator callback calibration, not flight validated or authorized for installation).

Current integrated landing research increment: `2.16.0-landing.20261126.duna-dynamics-m1.7` (normal CKAN launch, one-shot diagnostic request file; see [M1.7 contract](docs/Integrated-Landing-Dynamics-M1.md)).

Current integrated landing research increment: `2.16.0-landing.20261126.duna-planner-m2.0` — reference joint finite correction and braking timing planner, offline and non-actuating. See DEVELOPMENT.md for implementation and limitations.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m3.0`. Duna Landing Guidance: **Evaluate integrated descent (preview)** on an unpacked descending vessel at 1× with a selected target. Preview only; normal CKAN launch; no new live guidance enabled.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m4.0`. Experimental Duna descent prototype: evaluate a descending unpowered snapshot at 1×, then explicitly execute the model plan only if finer-step replay, model miss and sampled terrain/current-state admission pass. Existing Land at target remains legacy. Actual model/flight parity, safety and full orbit-to-surface integration unverified.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m4.1`. Explicit experimental descent now performs bounded background replanning and guarded trajectory replacement; normal Land at target remains unchanged.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m4.2`. Experimental descent now prepares bounded local-site contingencies; adoption requires a failed nominal tracker and current replayed/corridor-screened contingency. Coverage and real-game parity remain unverified; ordinary landing unchanged.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m5.0`. The explicit Duna orbital preview searches finite vector deorbit and approach trajectories jointly; only a replayed, target-tolerance, sampled-corridor model plan can enter experimental execution. No auto warp; actual orbit-to-contact acceptance remains open.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m5.1`. Experimental plan admission includes closed-loop model uncertainty screening; descent supervisor reports radial margins and invalidates targeting on best-effort hazard response. Actual flight validation remains outstanding.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m5.2`. Non-actuating convex prototype and common solver benchmark; compare synthetic fixtures using actual model contact and shared validation. Experimental live controller remains unchanged.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m5.3`. Shared integrator and contact-event corrections; no change to ordinary landing dispatch.

Current integrated landing increment: `2.16.0-landing.20261126.duna-integrated-m5.4`. Engineering-scale event and adaptive integration resolution; validation requirements unchanged.

Current integrated landing increment: `2.16.0-landing.20261127.duna-integrated-m5.5`. Bounded validation-aware selection and terminal replay completion.

Current integrated landing increment: `2.16.0-landing.20261127.duna-integrated-m5.6`. Normal Land at target workflow on Duna; automatic admission and planning, prospective drag-cube model. Installation readiness not established.

Current integrated landing increment: `2.16.0-landing.20261127.duna-integrated-m5.7` — fixed-inertial/world-frame boundary conversion and rotating-world regression, atmospheric temperature acquisition investigation. Vessel-independent and high-crossrange requirements documented in SPEC and the binding plan. Not installation validated.
