# Flashes build\pico_debug.uf2 onto the Pico without picotool.
#
# The Pico SDK's USB stack exposes a reset interface: opening its CDC port at
# 1200 baud and closing it reboots the board into the UF2 bootloader, which then
# mounts as a removable drive labelled RPI-RP2. Copying a .uf2 there flashes it
# and the board reboots itself. That is the whole mechanism - no extra tools.
#
# Usage:  powershell -ExecutionPolicy Bypass -File firmware\flash.ps1 [uf2path]

param(
    [string]$Uf2 = "$PSScriptRoot\build\pico_debug.uf2"
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $Uf2)) {
    Write-Output "[error] not found: $Uf2"
    Write-Output "        build it first: firmware\build.bat"
    exit 1
}

function Get-RpiDrive {
    # RP2040's bootloader labels its drive RPI-RP2; RP2350 (Pico 2 / Pico 2 W)
    # labels it RP2350. Accept either, or this silently never finds the board.
    Get-Volume -ErrorAction SilentlyContinue |
        Where-Object { $_.DriveType -eq 'Removable' -and
                       ($_.FileSystemLabel -eq 'RPI-RP2' -or $_.FileSystemLabel -eq 'RP2350') } |
        Select-Object -First 1
}

function Get-PicoPort {
    # Raspberry Pi's USB VID. Walk the enum tree for the port name rather than
    # guessing at COM numbers.
    $base = 'HKLM:\SYSTEM\CurrentControlSet\Enum\USB'
    Get-ChildItem $base -ErrorAction SilentlyContinue |
        Where-Object { $_.PSChildName -like 'VID_2E8A*' } |
        ForEach-Object { Get-ChildItem $_.PSPath -ErrorAction SilentlyContinue } |
        ForEach-Object {
            $p = Join-Path $_.PSPath 'Device Parameters'
            if (Test-Path $p) {
                $n = (Get-ItemProperty $p -ErrorAction SilentlyContinue).PortName
                if ($n) { $n }
            }
        } | Where-Object {
            # The Enum hive keeps PortName after the device is unplugged, so a
            # board sitting in BOOTSEL still "has" its old COM port here.
            # SERIALCOMM only lists ports that are actually present.
            $live = (Get-ItemProperty 'HKLM:\HARDWARE\DEVICEMAP\SERIALCOMM' -ErrorAction SilentlyContinue)
            $live -and ($live.PSObject.Properties.Value -contains $_)
        } | Select-Object -First 1
}

# Already in the bootloader? Then skip the touch entirely.
$drive = Get-RpiDrive

if (-not $drive) {
    $port = Get-PicoPort
    if (-not $port) {
        Write-Output "[error] no Pico found: no RPI-RP2 drive and no VID_2E8A serial port."
        Write-Output "        Either plug it in, or hold BOOTSEL while connecting USB."
        exit 1
    }

    Write-Output "[touch] rebooting $port into the bootloader (1200 baud)"
    try {
        $sp = New-Object System.IO.Ports.SerialPort $port, 1200, 'None', 8, 'One'
        $sp.DtrEnable = $true
        $sp.Open()
        Start-Sleep -Milliseconds 120
        $sp.Close()
    } catch {
        # The board frequently yanks the port away mid-close; that is success,
        # not failure, so this is deliberately not fatal.
        Write-Output "[touch] port closed abruptly (expected)"
    }

    Write-Output "[wait ] for RPI-RP2 to appear"
    for ($i = 0; $i -lt 50; $i++) {
        Start-Sleep -Milliseconds 300
        $drive = Get-RpiDrive
        if ($drive) { break }
    }
}

if (-not $drive) {
    Write-Output "[error] RPI-RP2 never appeared."
    Write-Output "        Hold the BOOTSEL button while plugging in USB, then re-run."
    exit 1
}

$dest = "$($drive.DriveLetter):\"
$size = [math]::Round((Get-Item $Uf2).Length / 1KB)
Write-Output "[flash] $([System.IO.Path]::GetFileName($Uf2)) ($size KB) -> $dest"

Copy-Item $Uf2 $dest -Force

# The board reboots as soon as the write completes, so the drive vanishing is
# the success signal.
Write-Output "[wait ] for reboot"
for ($i = 0; $i -lt 40; $i++) {
    Start-Sleep -Milliseconds 250
    if (-not (Get-RpiDrive)) { break }
}

Start-Sleep -Milliseconds 800
$port = Get-PicoPort
if ($port) {
    Write-Output "[ok   ] flashed; board is back on $port"
} else {
    Write-Output "[ok   ] flashed; board rebooted (serial port not up yet)"
}
