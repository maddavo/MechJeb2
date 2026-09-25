[CmdletBinding()]
param(
    [string]$Baseline = 'v1-terrain-profile-contact-20260925'
)

$ErrorActionPreference = 'Stop'
$protected = @(
    'MechJeb2/AutopilotModule.cs',
    'MechJeb2/MechJebModuleLandingAutopilot.cs',
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

# The V1 predictor files are intentionally excluded while this authorised predictor repair is in progress.
# The V2 panel is appended after the original V1 window controls. Compare the
# original V1 block line-for-line, while permitting the separate V2 panel below
# it to evolve.
$windowPath = 'MechJeb2/MechJebModuleLandingGuidance.cs'
$baselineWindow = @(& git show "${Baseline}:$windowPath")
$currentWindow = @(Get-Content -LiteralPath $windowPath)
$lastV1WindowLine = 176
for ($index = 0; $index -lt $lastV1WindowLine; $index++) {
    if ($index -ge $baselineWindow.Count -or $index -ge $currentWindow.Count -or
        -not ([string]::Equals($baselineWindow[$index], $currentWindow[$index], [System.StringComparison]::Ordinal))) {
        throw "V1 UI baseline violation at $windowPath line $($index + 1). V2 may only add its separate panel after the V1 controls."
    }
}

Write-Host "V1 baseline verified against $Baseline."