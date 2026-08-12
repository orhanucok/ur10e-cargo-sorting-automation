$ErrorActionPreference = "Stop"

$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$launcher = Join-Path $projectRoot "scripts\launch_project.ps1"
$desktop = [Environment]::GetFolderPath("Desktop")
$shortcutPath = Join-Path $desktop "UR10e Cargo Sorting Automation.lnk"
$powershell = Join-Path $env:SystemRoot "System32\WindowsPowerShell\v1.0\powershell.exe"

if (-not (Test-Path -LiteralPath $launcher)) {
    throw "Launcher was not found: $launcher"
}

$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $powershell
$shortcut.Arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$launcher`""
$shortcut.WorkingDirectory = $projectRoot
$shortcut.Description = "Launch the UR10e cargo sorting automation simulation"
$coppeliaIcon = "C:\Program Files\CoppeliaRobotics\CoppeliaSimPro\coppeliaSim.exe"
$shortcut.IconLocation = if (Test-Path -LiteralPath $coppeliaIcon) {
    "$coppeliaIcon,0"
} else {
    "$env:SystemRoot\System32\shell32.dll,25"
}
$shortcut.Save()

Write-Output $shortcutPath
