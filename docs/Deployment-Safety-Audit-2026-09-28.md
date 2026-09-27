# Deployment safety audit — 2026-09-28

## Authorised boundary

The sole authorised game target is
`C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\MechJeb2.dll`.
Building produces repository outputs only. Installation is a separate, explicit
operation after `KSP_x64` is closed. The installer requires `KSPDIR` to resolve
to `C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program`, rejects
linked paths, backs up the existing DLL with a timestamped manifest, and
SHA-256 verifies the one installed DLL. It offers no companion-DLL option.

The former `MechJeb2` and `MechJebKos` after-build copy targets and the `make
install` copy recipe have been disabled. The broad `make uninstall` deletion
recipe is also disabled. Both legacy `copy_build` scripts now
fail without copying. The installer is `tools/Install-MechJeb2.ps1`; use
`-ValidateOnly` for a read-only preflight.

## Current installation and companion DLLs

The earlier installation is documented in
`C:\Users\Dave\Documents\KSP Backups\MechJeb2 V1 Passive Predictor Capture\20260928-010355\manifest.json`.
All three companion DLLs below are in
`C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\GameData\MechJeb2\Plugins\`.
Their prior copies, with the same filenames, are in that manifest's directory.
The current repository Release outputs for these three DLLs have the installed
hashes shown below and also report assembly/file version `1.0.0.0`.

| DLL | Installed SHA-256 | Prior SHA-256 | Assembly/file version, installed and prior |
| --- | --- | --- | --- |
| `MechJebLib.dll` | `9B34071DAFE9FE3E77F2D86D58ECDC65BD0FE1DCB5D268C656E8C9C59B634A2C` | `7A0087D1F974D1DC4CA625A43DA747B01574263F4DB438ECD55507682655FD16` | `1.0.0.0` / `1.0.0.0` |
| `MechJebLibBindings.dll` | `8B6C7203BE8C6FC0CB66B96E758D2736348C851CDB89726A1B5A71630CC5E52D` | `ECBE87CE580DB4CEAD8E6F84EEAD11BDC5BEBC55EC3E9A3B8E788C207B57C206` | `1.0.0.0` / `1.0.0.0` |
| `alglib.dll` | `61F383E13F672BC69599D65265A36293265DA7164E4A2CB6FEA4CA0C042C46F9` | `0FBC9D910B1CC325478A4DD817AFD8A6067B34ECAE2829DECD38946736938268` | `1.0.0.0` / `1.0.0.0` |

`MechJeb2.csproj` references all three projects; `MechJebLibBindings` references
`MechJebLib`, and `MechJebLib` references `alglib`. Those are build/runtime
relationships, but they do not by themselves justify replacing installed
companions. The passive capture commit `37a75959` changed no files in these
three projects. Their versions are unchanged, while hashes differ. The prior
replacement was based on hash mismatch, without a demonstrated binary
compatibility requirement. The three installed companions must remain untouched
pending individual review and explicit approval for any future replacement.

## Unauthorised-location inventory

The complete read-only inventory, including every file and directory's relative
path, byte count, SHA-256 for files, UTC timestamps, and ACL owner, is in
`docs/Deployment-Inventory-2026-09-28.csv`.

| Tree | Root | Contents | Ownership and provenance evidence |
| --- | --- | --- | --- |
| `C-GameData` | `C:\GameData\MechJeb2` | 66 files, 7 directories including root; 9,552,009 file bytes. Top level: `Bundles`, `Icons`, `Localization`, `Parts`, `Plugins`, `LandingSites.cfg`. `Plugins` has 10 files. | Every inventoried item has ACL owner `DAVE-GT-PC\Dave`. Root creation time is 2026-09-02 13:23:08 UTC, preceding the 2026-09-27 build. The unsafe copy target could have overwritten files here; creation time and ACL owner do not establish who originally wrote each file. |
| `Temp-Staging` | `C:\Users\Dave\AppData\Local\Temp\MechJeb2-test-staging-ed6bc902e3c044dab231cf097b703c38` | 65 files, 9 directories including root; 9,233,741 file bytes. Under `GameData\MechJeb2`: `Bundles`, `Icons`, `Localization`, `Parts`, `Plugins`, `LandingSites.cfg`; `Plugins` has 9 files. | Every inventoried item has ACL owner `DAVE-GT-PC\Dave`. The root was created 2026-09-27 15:04:50 UTC by the previous test-staging operation in this task. |

Neither tree was changed or deleted during this audit. ACL ownership identifies
the current security owner; it does not prove original authorship.

## Verification

- `dotnet build MechJeb2.sln -c Release --no-restore` succeeded with zero warnings
  and errors after the automatic copy targets were removed.
- Read-only installer preflight rejected absent, `C:\GameData`, staging, and a
  nested `GameData` `KSPDIR`; it accepted the exact Steam directory. The legacy
  Windows copy script exits with failure before any copy.
- The 15 focused landing-library tests and five capture-reader tests passed.
  The full library suite had 8,405 passes and three failures: `ToSITest`
  expected `Infinity` but received `∞`, and two `MainSailTinCan` ascent values
  differed from their fixed expectations. No library or test source was changed
  in this deployment-safety correction.
- SHA-256 comparison after the build and tests found zero changes across all
  inventoried files in either extra tree and zero changes to the four installed
  DLLs recorded in the previous installation manifest. The installation branch
  that copies a DLL was not run.
