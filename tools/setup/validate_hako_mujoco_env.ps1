[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

function Show-Value([string]$Name) {
    $value = [Environment]::GetEnvironmentVariable($Name)
    if ([string]::IsNullOrWhiteSpace($value)) {
        Write-Warning "$Name is not set"
        return $false
    }
    Write-Host "$Name=$value"
    return $true
}

$ok = $true
$ok = (Show-Value 'HAKO_CORE_INC_PATH') -and $ok
$ok = (Show-Value 'HAKO_CORE_LIB_PATH') -and $ok
$ok = (Show-Value 'HAKO_DRONE_INC_PATH') -and $ok
$ok = (Show-Value 'HAKO_DRONE_LIB_PATH') -and $ok
$ok = (Show-Value 'HAKO_BINARY_PATH') -and $ok
$ok = (Show-Value 'MUJOCO_ROOT') -and $ok

if ($env:MUJOCO_ROOT) {
    $header = Join-Path $env:MUJOCO_ROOT 'include\mujoco\mujoco.h'
    if (-not (Test-Path -LiteralPath $header)) {
        Write-Warning "MuJoCo header not found: $header"
        $ok = $false
    }
    $libraries = @(
        (Join-Path $env:MUJOCO_ROOT 'lib\mujoco.lib'),
        (Join-Path $env:MUJOCO_ROOT 'bin\mujoco.dll')
    )
    if (-not ($libraries | Where-Object { Test-Path -LiteralPath $_ })) {
        Write-Warning 'MuJoCo import library or DLL was not found under MUJOCO_ROOT.'
        $ok = $false
    }
}

if (-not $ok) {
    Write-Error 'Build environment is incomplete. Set the variables for the installed Hakoniwa and matching MuJoCo package; this script does not install or modify them.'
}
Write-Output 'Hakoniwa/MuJoCo Windows build environment: valid'
