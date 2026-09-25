[CmdletBinding()]
param(
    [string]$Baseline = 'v1-predictor-terrain-convergence-20260925'
)

$ErrorActionPreference = 'Stop'
$protected = @(
    'MechJeb2/AutopilotModule.cs',
    'MechJeb2/MechJebModuleLandingAutopilot.cs',
    'MechJeb2/MechJebModuleLandingPredictions.cs',
    'MechJeb2/MechJebModuleThrustController.cs',
    'MechJeb2/ReentrySimulation.cs',
    'MechJeb2/LandingAutopilot/CoastToDeceleration.cs',
    'MechJeb2/LandingAutopilot/LandingPredictionConsensus.cs',
    'MechJeb2/LandingAutopilot/LandingPredictionTerrainConvergence.cs',
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

# The V2 panel is appended after the original V1 window controls. Compare the
# original V1 block line-for-line, while permitting the separate V2 panel below
# it to evolve.
$windowPath = 'MechJeb2/MechJebModuleLandingGuidance.cs'
$baselineWindow = @(& git show "${Baseline}:$windowPath")
$currentWindow = @(Get-Content -LiteralPath $windowPath)
$lastV1WindowLine = 176
for ($line = 0; $line -lt $lastV1WindowLine; $line++) {
    if ($line -ge $baselineWindow.Count -or $line -ge $currentWindow.Count -or
        $baselineWindow[$line] -cne $currentWindow[$line]) {
        throw "V1 UI baseline violation at $windowPath line $($line + 1). V2 may only add its separate panel after the V1 controls."
    }
}

Write-Host "V1 baseline verified against $Baseline."
