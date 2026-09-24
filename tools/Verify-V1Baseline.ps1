[CmdletBinding()]
param(
    [string]$Baseline = 'v1-proven-c47337ff'
)

$ErrorActionPreference = 'Stop'
$protected = @(
    'MechJeb2/AutopilotModule.cs',
    'MechJeb2/MechJebModuleLandingAutopilot.cs',
    'MechJeb2/MechJebModuleLandingPredictions.cs',
    'MechJeb2/MechJebModuleThrustController.cs',
    'MechJeb2/ReentrySimulation.cs',
    'MechJeb2/LandingAutopilot/CoastToDeceleration.cs',
    'MechJeb2/LandingAutopilot/CourseCorrection.cs',
    'MechJeb2/LandingAutopilot/DeorbitBurn.cs',
    'MechJeb2/LandingAutopilot/FinalDescent.cs',
    'MechJeb2/LandingAutopilot/KillHorizontalVelocity.cs',
    'MechJeb2/LandingAutopilot/PlaneChange.cs'
)

git rev-parse --verify "$Baseline^{commit}" | Out-Null
& git diff --exit-code $Baseline -- @protected
if ($LASTEXITCODE -ne 0) {
    throw "V1 baseline violation: protected Landing Guidance V1 source differs from $Baseline."
}
Write-Host "V1 baseline verified against $Baseline."
