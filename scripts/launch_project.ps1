param(
    [switch]$NoPause
)

$ErrorActionPreference = "Stop"
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$python = Join-Path $projectRoot "work\.venv\Scripts\python.exe"
$sceneBuilder = Join-Path $projectRoot "coppeliasim\build_scene.py"
$cellConfig = Join-Path $projectRoot "config\cell.yaml"
$urdf = Join-Path $projectRoot "artifacts\ur10e_coppelia.urdf"
$urdfGenerator = Join-Path $projectRoot "scripts\generate_ur10e_urdf.ps1"
$simulationWatchdog = Join-Path $projectRoot "scripts\simulation_watchdog.py"
$watchdogMaintenance = Join-Path $projectRoot "work\watchdog_maintenance.flag"

function Complete-Launch([int]$Code) {
    if ($Code -eq 0) {
        Write-Host "The launcher can close; CoppeliaSim will continue running." -ForegroundColor DarkGray
        Start-Sleep -Seconds 2
    } elseif (-not $NoPause) {
        Write-Host ""
        Read-Host "Press Enter to close this window"
    }
    exit $Code
}

function Test-TcpPort([string]$HostName, [int]$Port, [int]$TimeoutMs = 350) {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $task = $client.ConnectAsync($HostName, $Port)
        if (-not $task.Wait($TimeoutMs)) {
            return $false
        }
        return $client.Connected
    } catch {
        return $false
    } finally {
        $client.Dispose()
    }
}

function Find-CoppeliaSim {
    $command = Get-Command coppeliaSim.exe -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    $roots = @(
        (Join-Path $env:ProgramFiles "CoppeliaRobotics"),
        (Join-Path $env:ProgramFiles "CoppeliaSim"),
        (Join-Path $env:LOCALAPPDATA "Programs\CoppeliaSim")
    )
    foreach ($root in $roots) {
        if (Test-Path -LiteralPath $root) {
            $candidate = Get-ChildItem -LiteralPath $root -Filter coppeliaSim.exe -File -Recurse -ErrorAction SilentlyContinue |
                Select-Object -First 1
            if ($null -ne $candidate) {
                return $candidate.FullName
            }
        }
    }
    return $null
}

Clear-Host
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host " UR10e CARGO SORTING AUTOMATION" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "Project: $projectRoot"
Write-Host ""

if (-not (Test-Path -LiteralPath $python)) {
    Write-Host "ERROR: The Python project environment was not found." -ForegroundColor Red
    Write-Host "Run the setup commands in README.md first."
    Complete-Launch 2
}

Write-Host "[1/4] Running core safety tests..." -ForegroundColor Yellow
Push-Location $projectRoot
try {
    & $python -m pytest -q
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Tests failed; the simulation was not started." -ForegroundColor Red
        Complete-Launch 3
    }
} finally {
    Pop-Location
}

Write-Host "[2/4] Verifying the official UR10e model..." -ForegroundColor Yellow
if (-not (Test-Path -LiteralPath $urdf)) {
    & $urdfGenerator
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $urdf)) {
        Write-Host "ERROR: The UR10e model could not be generated." -ForegroundColor Red
        Complete-Launch 4
    }
}

Write-Host "[3/4] Locating CoppeliaSim..." -ForegroundColor Yellow
$simulatorRunning = Test-TcpPort "127.0.0.1" 23000
if (-not $simulatorRunning) {
    $coppeliaSim = Find-CoppeliaSim
    if ($null -eq $coppeliaSim) {
        Write-Host ""
        Write-Host "CoppeliaSim is not installed." -ForegroundColor Red
        Write-Host "Install CoppeliaSim 4.10 to run the physics scene."
        Write-Host "The Edu/Pro package is not downloaded automatically because license eligibility must be confirmed."
        Complete-Launch 4
    }

    Write-Host "Starting CoppeliaSim: $coppeliaSim"
    Start-Process -FilePath $coppeliaSim -WorkingDirectory (Split-Path $coppeliaSim -Parent) | Out-Null
    $deadline = [DateTime]::UtcNow.AddSeconds(30)
    do {
        Start-Sleep -Milliseconds 500
        $simulatorRunning = Test-TcpPort "127.0.0.1" 23000
    } while (-not $simulatorRunning -and [DateTime]::UtcNow -lt $deadline)

    if (-not $simulatorRunning) {
        Write-Host "ERROR: CoppeliaSim started, but ZeroMQ port 23000 did not become ready." -ForegroundColor Red
        Complete-Launch 5
    }
}

Write-Host "[4/4] Building the MuJoCo sorting cell..." -ForegroundColor Yellow
$arguments = @(
    $sceneBuilder,
    "--config", $cellConfig,
    "--urdf", $urdf,
    "--replace",
    "--start"
)

New-Item -ItemType File -Path $watchdogMaintenance -Force | Out-Null
try {
    & $python @arguments
    $sceneBuilderExitCode = $LASTEXITCODE
} finally {
    Remove-Item -LiteralPath $watchdogMaintenance -Force -ErrorAction SilentlyContinue
}
if ($sceneBuilderExitCode -ne 0) {
    Write-Host "ERROR: The CoppeliaSim scene could not be built." -ForegroundColor Red
    Complete-Launch 6
}

Start-Process `
    -FilePath $python `
    -ArgumentList @($simulationWatchdog) `
    -WorkingDirectory $projectRoot `
    -WindowStyle Hidden | Out-Null

Write-Host "Simulation started: physical infeed, label orientation, and color sorting are active." -ForegroundColor Green
Write-Host "Health watchdog active: pause state, heartbeat, and production progress are monitored." -ForegroundColor Green
Complete-Launch 0
