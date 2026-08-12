$ErrorActionPreference = "Continue"

Write-Output "GPU"
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader

Write-Output "System"
Get-CimInstance Win32_ComputerSystem |
    Select-Object Manufacturer, Model, @{Name="RAM_GB"; Expression={[math]::Round($_.TotalPhysicalMemory / 1GB, 1)}}

Write-Output "CPU"
Get-CimInstance Win32_Processor |
    Select-Object Name, NumberOfCores, NumberOfLogicalProcessors

Write-Output "WSL distributions"
wsl --list --verbose

Write-Output "CoppeliaSim"
$coppelia = Get-Command coppeliaSim.exe -ErrorAction SilentlyContinue
if ($null -eq $coppelia) {
    Write-Output "NOT_INSTALLED"
} else {
    Write-Output $coppelia.Source
}

