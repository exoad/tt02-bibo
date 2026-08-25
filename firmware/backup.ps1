# Reads the Pico's current flash back to a .uf2 so it can be restored later.
#
# Run this BEFORE flashing anything you cannot rebuild from source.
# Restore with:  firmware\flash.bat <the-backup>.uf2
#
# Usage: powershell -ExecutionPolicy Bypass -File firmware\backup.ps1 [out.uf2]

param(
    [string]$Out = "$PSScriptRoot\..\vendor\pico-flash-backup.uf2"
)

$ErrorActionPreference = 'Stop'
$picotool = "$PSScriptRoot\..\vendor\picotool-2.3.0\picotool\picotool.exe"

if (-not (Test-Path $picotool)) {
    Write-Output "[error] picotool not found at $picotool"
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

if (-not (Get-RpiDrive)) {
    $port = Get-PicoPort
    if (-not $port) {
        Write-Output "[error] no Pico found (no RPI-RP2 drive, no VID_2E8A serial port)"
        exit 1
    }

    # picotool's own `reboot -u` did not work here; the 1200-baud touch on the
    # SDK's USB reset interface does, and is the same mechanism flash.ps1 uses.
    Write-Output "[touch] rebooting $port into BOOTSEL (1200 baud)"
    try {
        $sp = New-Object System.IO.Ports.SerialPort $port, 1200, 'None', 8, 'One'
        $sp.DtrEnable = $true
        $sp.Open()
        Start-Sleep -Milliseconds 120
        $sp.DtrEnable = $false      # the DTR-low transition at 1200 is the trigger
        Start-Sleep -Milliseconds 120
        $sp.Close()
    } catch {
        Write-Output "[touch] port closed abruptly (expected)"
    }

    Write-Output "[wait ] for RPI-RP2"
    for ($i = 0; $i -lt 50; $i++) {
        Start-Sleep -Milliseconds 300
        if (Get-RpiDrive) { break }
    }
}

if (-not (Get-RpiDrive)) {
    Write-Output "[error] board never entered BOOTSEL."
    Write-Output "        Hold the BOOTSEL button while replugging USB, then re-run."
    exit 1
}

Write-Output "[save ] reading flash -> $Out"
& $picotool save -a $Out
if ($LASTEXITCODE -ne 0) { Write-Output "[error] picotool save failed ($LASTEXITCODE)"; exit 1 }
if (-not (Test-Path $Out)) { Write-Output "[error] $Out was not created"; exit 1 }

$kb = [math]::Round((Get-Item $Out).Length / 1KB)
Write-Output "[ok   ] $kb KB -> $Out"
Write-Output "        restore with:  firmware\flash.bat `"$Out`""
