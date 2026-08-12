$ErrorActionPreference = "Stop"

$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$vendor = Join-Path $projectRoot "work\vendor\Universal_Robots_ROS2_Description"
$xacro = Join-Path $projectRoot "work\.venv\Scripts\xacro.exe"
$outputDirectory = Join-Path $projectRoot "artifacts"
$output = Join-Path $outputDirectory "ur10e.urdf"
$coppeliaOutput = Join-Path $outputDirectory "ur10e_coppelia.urdf"
$shim = Join-Path $projectRoot "tools\xacro_shims"
$python = Join-Path $projectRoot "work\.venv\Scripts\python.exe"

if (-not (Test-Path -LiteralPath (Join-Path $vendor "package.xml"))) {
    throw "Pinned Universal Robots description package was not found."
}
if (-not (Test-Path -LiteralPath $xacro)) {
    throw "xacro is not installed in the project virtual environment."
}

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$env:UR_DESCRIPTION_SHARE = $vendor
$previousPythonPath = $env:PYTHONPATH
$env:PYTHONPATH = if ($previousPythonPath) { "$shim;$previousPythonPath" } else { $shim }

try {
    & $xacro `
        (Join-Path $vendor "urdf\ur.urdf.xacro") `
        "ur_type:=ur10e" `
        "name:=ur10e" `
        "tf_prefix:=" `
        "force_abs_paths:=true" `
        -o $output
    if ($LASTEXITCODE -ne 0) {
        throw "xacro failed with exit code $LASTEXITCODE"
    }

    # Xacro emits Windows absolute paths as file://C:\..., while URDF importers
    # require RFC-style file:///C:/... URIs.
    $urdfText = [System.IO.File]::ReadAllText($output)
    $urdfText = [System.Text.RegularExpressions.Regex]::Replace(
        $urdfText,
        'file://([A-Za-z]):\\',
        'file:///$1:/'
    )
    $urdfText = $urdfText.Replace('\', '/')
    [System.IO.File]::WriteAllText(
        $output,
        $urdfText,
        [System.Text.UTF8Encoding]::new($false)
    )
} finally {
    $env:PYTHONPATH = $previousPythonPath
}

& $python `
    (Join-Path $projectRoot "tools\prepare_coppelia_urdf.py") `
    $output `
    $coppeliaOutput
if ($LASTEXITCODE -ne 0) {
    throw "CoppeliaSim URDF preparation failed with exit code $LASTEXITCODE"
}

Write-Output $coppeliaOutput
