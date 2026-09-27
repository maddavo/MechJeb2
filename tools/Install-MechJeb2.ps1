#Requires -Version 5.1
[CmdletBinding()]
param(
    [switch]$ValidateOnly
)

$ErrorActionPreference = 'Stop'
$expectedKspDir = 'C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program'
$relativeTarget = 'GameData\MechJeb2\Plugins\MechJeb2.dll'

function Assert-NoReparsePoint([string]$Path) {
    $item = Get-Item -LiteralPath $Path -Force
    while ($null -ne $item) {
        if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
            throw "Refusing a linked or redirected path: $($item.FullName)"
        }
        $item = if ($item -is [IO.DirectoryInfo]) { $item.Parent } else { $item.Directory }
    }
}

if ([string]::IsNullOrWhiteSpace($env:KSPDIR)) {
    throw 'KSPDIR must be set to the authorised Kerbal Space Program installation.'
}

$resolvedKspDir = (Resolve-Path -LiteralPath $env:KSPDIR -ErrorAction Stop).ProviderPath
$resolvedKspDir = [IO.Path]::GetFullPath($resolvedKspDir).TrimEnd('\')
if (-not [string]::Equals($resolvedKspDir, $expectedKspDir, [StringComparison]::OrdinalIgnoreCase)) {
    throw "KSPDIR resolves to '$resolvedKspDir'; the only authorised directory is '$expectedKspDir'."
}
Assert-NoReparsePoint $resolvedKspDir

$target = Join-Path $resolvedKspDir $relativeTarget
if (-not (Test-Path -LiteralPath $target -PathType Leaf)) {
    throw "The authorised existing DLL is missing: $target"
}
Assert-NoReparsePoint $target

$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$source = Join-Path $repoRoot 'MechJeb2\bin\Release\MechJeb2.dll'
if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
    throw "The repository Release build is missing: $source"
}
Assert-NoReparsePoint $source

function Assert-KspClosed {
    $running = @(Get-Process -Name 'KSP_x64', 'KSP' -ErrorAction SilentlyContinue)
    if ($running.Count -gt 0) {
        throw "KSP is running ($($running.Name -join ', ')); close it before installation."
    }
}

Assert-KspClosed
$oldHash = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash
$newHash = (Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash

if ($ValidateOnly) {
    [pscustomobject]@{
        Mode = 'read-only validation'
        KSPDIR = $resolvedKspDir
        Source = $source
        SourceSHA256 = $newHash
        Target = $target
        TargetSHA256 = $oldHash
        KspClosed = $true
    }
    return
}

if ($oldHash -eq $newHash) {
    Write-Output "The authorised DLL already matches the Release build ($newHash); no installation was needed."
    return
}

$backupRoot = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'KSP Backups\MechJeb2 Explicit Install'
$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss-fffffff'
$backupDir = Join-Path $backupRoot $timestamp
$null = New-Item -ItemType Directory -Path $backupDir -ErrorAction Stop
$backup = Join-Path $backupDir 'MechJeb2.dll'
$manifestPath = Join-Path $backupDir 'manifest.json'
$copyAttempted = $false
$manifest = [ordered]@{
    createdLocal = (Get-Date).ToString('o')
    status = 'backup pending'
    kspDir = $resolvedKspDir
    target = $target
    source = $source
    backup = $backup
    originalSha256 = $oldHash
    sourceSha256 = $newHash
    installedSha256 = $null
    changedFiles = @('MechJeb2.dll')
}

try {
    Copy-Item -LiteralPath $target -Destination $backup -ErrorAction Stop
    if ((Get-FileHash -LiteralPath $backup -Algorithm SHA256).Hash -ne $oldHash) {
        throw 'The backup hash differs from the installed DLL; installation was stopped.'
    }
    $manifest.status = 'backup verified; install pending'
    $manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $manifestPath -Encoding UTF8

    Assert-KspClosed
    if ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $oldHash) {
        throw 'The installed DLL changed after backup; installation was stopped.'
    }
    if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -ne $newHash) {
        throw 'The Release DLL changed after validation; installation was stopped.'
    }

    $copyAttempted = $true
    Copy-Item -LiteralPath $source -Destination $target -Force -ErrorAction Stop
    $installedHash = (Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash
    if ($installedHash -ne $newHash) {
        throw "Installed DLL hash verification failed: $installedHash"
    }
    $manifest.installedSha256 = $installedHash
    $manifest.status = 'installed and SHA-256 verified'
    $manifest.installedLocal = (Get-Date).ToString('o')
    $manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $manifestPath -Encoding UTF8
    Write-Output "Installed and SHA-256 verified: $target"
    Write-Output "Backup manifest: $manifestPath"
} catch {
    $failure = $_
    if ($copyAttempted) {
        try {
            Copy-Item -LiteralPath $backup -Destination $target -Force -ErrorAction Stop
            if ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -ne $oldHash) {
                throw 'Rollback hash verification failed.'
            }
            $manifest.status = 'installation failed; original DLL restored and SHA-256 verified'
        } catch {
            $manifest.status = 'installation failed; rollback requires manual review'
            $manifest.rollbackError = $_.Exception.Message
        }
    } else {
        $manifest.status = 'installation stopped before target copy'
    }
    $manifest.error = $failure.Exception.Message
    $manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $manifestPath -Encoding UTF8
    throw "$($failure.Exception.Message) Backup manifest: $manifestPath"
}
