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
        Write-Host "Baslatici kapanabilir; CoppeliaSim calismaya devam edecek." -ForegroundColor DarkGray
        Start-Sleep -Seconds 2
    } elseif (-not $NoPause) {
        Write-Host ""
        Read-Host "Pencereyi kapatmak icin Enter'a basin"
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
Write-Host " UR10e GERCEKCI KOLI AYIRMA SIMULASYONU" -ForegroundColor Cyan
Write-Host "============================================================" -ForegroundColor Cyan
Write-Host "Proje: $projectRoot"
Write-Host ""

if (-not (Test-Path -LiteralPath $python)) {
    Write-Host "HATA: Python proje ortami bulunamadi." -ForegroundColor Red
    Write-Host "Once README.md icindeki kurulum komutlarini calistirin."
    Complete-Launch 2
}

Write-Host "[1/4] Cekirdek guvenlik testleri calisiyor..." -ForegroundColor Yellow
Push-Location $projectRoot
try {
    & $python -m pytest -q
    if ($LASTEXITCODE -ne 0) {
        Write-Host "HATA: Testler gecmedi; simulasyon guvenli sekilde baslatilmadi." -ForegroundColor Red
        Complete-Launch 3
    }
} finally {
    Pop-Location
}

Write-Host "[2/4] Resmi UR10e modeli dogrulaniyor..." -ForegroundColor Yellow
if (-not (Test-Path -LiteralPath $urdf)) {
    & $urdfGenerator
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $urdf)) {
        Write-Host "HATA: UR10e modeli uretilemedi." -ForegroundColor Red
        Complete-Launch 4
    }
}

Write-Host "[3/4] CoppeliaSim araniyor..." -ForegroundColor Yellow
$simulatorRunning = Test-TcpPort "127.0.0.1" 23000
if (-not $simulatorRunning) {
    $coppeliaSim = Find-CoppeliaSim
    if ($null -eq $coppeliaSim) {
        Write-Host ""
        Write-Host "CoppeliaSim henuz kurulu degil." -ForegroundColor Red
        Write-Host "Kisayol ve proje calisiyor; fizik sahnesini acmak icin CoppeliaSim 4.10 kurulumu gerekiyor."
        Write-Host "Lisans uygunlugu belirlenmeden Edu/Pro paketi otomatik indirilmeyecek."
        Complete-Launch 4
    }

    Write-Host "CoppeliaSim baslatiliyor: $coppeliaSim"
    Start-Process -FilePath $coppeliaSim -WorkingDirectory (Split-Path $coppeliaSim -Parent) | Out-Null
    $deadline = [DateTime]::UtcNow.AddSeconds(30)
    do {
        Start-Sleep -Milliseconds 500
        $simulatorRunning = Test-TcpPort "127.0.0.1" 23000
    } while (-not $simulatorRunning -and [DateTime]::UtcNow -lt $deadline)

    if (-not $simulatorRunning) {
        Write-Host "HATA: CoppeliaSim acildi ancak ZeroMQ 23000 portu hazir olmadi." -ForegroundColor Red
        Complete-Launch 5
    }
}

Write-Host "[4/4] MuJoCo hucre sahnesi kuruluyor..." -ForegroundColor Yellow
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
    Write-Host "HATA: CoppeliaSim sahnesi kurulamadi." -ForegroundColor Red
    Complete-Launch 6
}

Start-Process `
    -FilePath $python `
    -ArgumentList @($simulationWatchdog) `
    -WorkingDirectory $projectRoot `
    -WindowStyle Hidden | Out-Null

Write-Host "Simulasyon baslatildi: uctan uca fiziksel besleme, etiket yonlendirme ve renk ayirma aktif." -ForegroundColor Green
Write-Host "Saglik gozcusu aktif: duraklama, kalp atisi ve uretim ilerlemesi izleniyor." -ForegroundColor Green
Complete-Launch 0
