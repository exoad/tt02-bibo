# Screenshots the preview harness.
#   shot.ps1 <out.png> [mouseX mouseY]   - screen coords to park the cursor at
param([string]$Out = "shot.png", [int]$MX = -1, [int]$MY = -1, [string]$ExeArgs = "")

Add-Type -AssemblyName System.Windows.Forms, System.Drawing

$sig = @'
using System;
using System.Runtime.InteropServices;
public class Win {
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool GetClientRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
}
'@
Add-Type -TypeDefinition $sig
[void][Win]::SetProcessDPIAware()

$exe = Join-Path $PSScriptRoot "build\board_preview.exe"
if ($ExeArgs -ne "") { $p = Start-Process -FilePath $exe -ArgumentList $ExeArgs -PassThru }
else                 { $p = Start-Process -FilePath $exe -PassThru }
Start-Sleep -Seconds 3

if ($p.HasExited) { Write-Output "EXITED code=$($p.ExitCode)"; exit 1 }
$p.Refresh()
$h = $p.MainWindowHandle
if ($h -eq [IntPtr]::Zero) { Write-Output "NO WINDOW"; exit 1 }

[void][Win]::SetForegroundWindow($h)
if ($MX -ge 0) {
  [void][Win]::SetCursorPos($MX, $MY)
  Start-Sleep -Milliseconds 1200
}

$r = New-Object Win+RECT
[void][Win]::GetWindowRect($h, [ref]$r)
$w = $r.R - $r.L; $ht = $r.B - $r.T

$bmp = New-Object System.Drawing.Bitmap $w, $ht
$gfx = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $gfx.GetHdc()
[void][Win]::PrintWindow($h, $hdc, 2)
$gfx.ReleaseHdc($hdc)
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$gfx.Dispose(); $bmp.Dispose()

Write-Output "OK window=${w}x${ht} at ($($r.L),$($r.T))"
Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue
