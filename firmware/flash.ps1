# The Pico's flash, from Windows, with no BOOTSEL button and no picotool to flash.
#
#   firmware\flash.bat [file.uf2]           flash it; default build\pico_debug.uf2
#   firmware\flash.bat backup [out.uf2]     read the board's flash back to a .uf2;
#                                           default vendor\pico-flash-backup.uf2
#
# Back up BEFORE flashing anything you cannot rebuild from source, and restore
# with firmware\flash.bat <the-backup>.uf2.
#
# Both start the same way. The Pico SDK's USB stack exposes a reset interface:
# opening its CDC port at 1200 baud and dropping DTR reboots the board into the
# UF2 bootloader, which mounts as a removable drive. Copying a .uf2 there flashes
# it and the board reboots itself; picotool reads the flash while it waits there.
#
# Usage:  powershell -ExecutionPolicy Bypass -File firmware\flash.ps1 [backup] [path]

param(
    [string]$Verb = 'flash',
    [string]$Path = ''
)

$ErrorActionPreference = 'Stop'

# "flash.bat <file.uf2>" with no verb is the form firmware\build.bat prints.
if ($Verb -ne 'flash' -and $Verb -ne 'backup') {
    $Path = $Verb
    $Verb = 'flash'
}

$picotool = "$PSScriptRoot\..\vendor\picotool-2.3.0\picotool\picotool.exe"

# Everything that can be checked without the board is checked before touching it.
if ($Verb -eq 'flash') {
    if (-not $Path) { $Path = "$PSScriptRoot\build\pico_debug.uf2" }
    if (-not (Test-Path $Path)) {
        Write-Output "[error] not found: $Path"
        Write-Output "        build it first: firmware\build.bat"
        exit 1
    }
} else {
    if (-not $Path) { $Path = "$PSScriptRoot\..\vendor\pico-flash-backup.uf2" }
    if (-not (Test-Path $picotool)) {
        Write-Output "[error] picotool not found at $picotool"
        exit 1
    }
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

    # picotool's own `reboot -u` did not work here; this touch does.
    Write-Output "[touch] rebooting $port into the bootloader (1200 baud)"
    try {
        $sp = New-Object System.IO.Ports.SerialPort $port, 1200, 'None', 8, 'One'
        $sp.DtrEnable = $true
        $sp.Open()
        Start-Sleep -Milliseconds 120
        $sp.DtrEnable = $false      # the DTR-low transition at 1200 is the trigger
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

if ($Verb -eq 'backup') {
    Write-Output "[save ] reading flash -> $Path"
    & $picotool save -a $Path
    if ($LASTEXITCODE -ne 0) { Write-Output "[error] picotool save failed ($LASTEXITCODE)"; exit 1 }
    if (-not (Test-Path $Path)) { Write-Output "[error] $Path was not created"; exit 1 }

    $kb = [math]::Round((Get-Item $Path).Length / 1KB)
    Write-Output "[ok   ] $kb KB -> $Path"
    Write-Output "        restore with:  firmware\flash.bat `"$Path`""
    exit 0
}

$dest = "$($drive.DriveLetter):\"
$size = [math]::Round((Get-Item $Path).Length / 1KB)
Write-Output "[flash] $([System.IO.Path]::GetFileName($Path)) ($size KB) -> $dest"

Copy-Item $Path $dest -Force

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
