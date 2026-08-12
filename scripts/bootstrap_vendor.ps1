param(
    [string]$Destination = ".\work\vendor\Universal_Robots_ROS2_Description"
)

$ErrorActionPreference = "Stop"
$expectedCommit = "ae333289875f9ba5a9ea6649a54036efb5ccabee"
$resolvedDestination = [System.IO.Path]::GetFullPath($Destination)
$workspace = [System.IO.Path]::GetFullPath((Get-Location).Path)

if (-not $resolvedDestination.StartsWith($workspace, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Vendor destination must remain inside the workspace."
}

if (-not (Test-Path -LiteralPath $resolvedDestination)) {
    git clone --branch 4.3.1 --depth 1 `
        https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git `
        $resolvedDestination
}

$actualCommit = git -C $resolvedDestination rev-parse HEAD
if ($actualCommit.Trim() -ne $expectedCommit) {
    throw "UR description commit mismatch: expected $expectedCommit, got $actualCommit"
}

Write-Output "Verified Universal Robots description at $actualCommit"

