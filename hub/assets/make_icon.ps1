# Generates hub/assets/bibo.ico — the application and window icon.
#
# Drawn rather than sourced, because the app icon has to be the app: the same
# graphite plate, top bevel and cyan LED accents the UI is built from, with a
# radar sweep as the subject. Fugue is 16x16 only and an icon needs 256.
#
# Run from anywhere:  powershell -ExecutionPolicy Bypass -File hub\assets\make_icon.ps1
#
# Entry format matters and the obvious choice is wrong. PNG-compressed entries
# are legal since Vista, but GDI+ (System.Drawing.Icon) cannot decode them, so an
# all-PNG .ico fails to render in a whole class of tooling even though Explorer
# shows it fine. So: uncompressed DIB for every size up to 64, PNG only for 128
# and 256 where the size saving is real and nothing small reads them.

Add-Type -AssemblyName System.Drawing

$sizes  = @(16, 20, 24, 32, 48, 64, 128, 256)
$outDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$icoPath = Join-Path $outDir 'bibo.ico'

function New-IconBitmap([int]$s) {
    $bmp = New-Object System.Drawing.Bitmap $s, $s,
           ([System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.SmoothingMode     = 'AntiAlias'
    $g.InterpolationMode = 'HighQualityBicubic'
    $g.Clear([System.Drawing.Color]::Transparent)

    $inset = [math]::Max(1, [int]($s * 0.03))
    $w     = $s - ($inset * 2)
    $rad   = [math]::Max(2, [int]($s * 0.18))

    # ---- the plate: rounded graphite, lit from above --------------------
    $path = New-Object System.Drawing.Drawing2D.GraphicsPath
    $d = $rad * 2
    $path.AddArc($inset, $inset, $d, $d, 180, 90)
    $path.AddArc($inset + $w - $d, $inset, $d, $d, 270, 90)
    $path.AddArc($inset + $w - $d, $inset + $w - $d, $d, $d, 0, 90)
    $path.AddArc($inset, $inset + $w - $d, $d, $d, 90, 90)
    $path.CloseFigure()

    $rect = New-Object System.Drawing.Rectangle $inset, $inset, $w, $w
    $brush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
        $rect,
        [System.Drawing.Color]::FromArgb(255, 60, 64, 72),
        [System.Drawing.Color]::FromArgb(255, 20, 22, 26),
        90.0)
    $g.FillPath($brush, $path)
    $brush.Dispose()

    # 1px light bevel along the top edge, dark seam around the whole plate —
    # the same tactile treatment ui::bevelRect gives every control.
    $penEdge = New-Object System.Drawing.Pen (
        [System.Drawing.Color]::FromArgb(210, 0, 0, 0)), ([float][math]::Max(1, $s / 96))
    $g.DrawPath($penEdge, $path)
    $penEdge.Dispose()

    if ($s -ge 24) {
        $penTop = New-Object System.Drawing.Pen (
            [System.Drawing.Color]::FromArgb(70, 255, 255, 255)), ([float][math]::Max(1, $s / 96))
        $g.DrawArc($penTop, $inset, $inset, $d, $d, 180, 90)
        $g.DrawLine($penTop, $inset + $rad, $inset, $inset + $w - $rad, $inset)
        $g.DrawArc($penTop, $inset + $w - $d, $inset, $d, $d, 270, 90)
        $penTop.Dispose()
    }

    # ---- the subject: a radar sweep --------------------------------------
    $cx = $s / 2.0
    $cy = $s * 0.52
    $cyan = [System.Drawing.Color]::FromArgb(255, 79, 195, 247)

    # Sweep wedge, brightest at its leading edge.
    if ($s -ge 20) {
        $rSweep = $s * 0.33
        $wedge = New-Object System.Drawing.Drawing2D.GraphicsPath
        $wedge.AddPie(($cx - $rSweep), ($cy - $rSweep), ($rSweep * 2), ($rSweep * 2), -95, 70)
        $sb = New-Object System.Drawing.SolidBrush (
            [System.Drawing.Color]::FromArgb(70, $cyan.R, $cyan.G, $cyan.B))
        $g.FillPath($sb, $wedge)
        $sb.Dispose(); $wedge.Dispose()
    }

    # Range rings.
    $ringPen = New-Object System.Drawing.Pen (
        [System.Drawing.Color]::FromArgb(190, $cyan.R, $cyan.G, $cyan.B)),
        ([float][math]::Max(1, $s / 26))
    foreach ($f in @(0.33, 0.20)) {
        if ($s -lt 24 -and $f -lt 0.30) { continue }   # one ring only when tiny
        $r = $s * $f
        $g.DrawEllipse($ringPen, ($cx - $r), ($cy - $r), ($r * 2), ($r * 2))
    }
    $ringPen.Dispose()

    # The sweep line, from the hub to the leading edge.
    $linePen = New-Object System.Drawing.Pen (
        [System.Drawing.Color]::FromArgb(230, 190, 235, 255)),
        ([float][math]::Max(1, $s / 26))
    $g.DrawLine($linePen, [single]$cx, [single]$cy,
                [single]($cx + $s * 0.33 * [math]::Cos(-25 * [math]::PI / 180)),
                [single]($cy + $s * 0.33 * [math]::Sin(-25 * [math]::PI / 180)))
    $linePen.Dispose()

    # The hub, lit.
    $rHub = [math]::Max(1.0, $s * 0.075)
    $glow = New-Object System.Drawing.SolidBrush (
        [System.Drawing.Color]::FromArgb(90, $cyan.R, $cyan.G, $cyan.B))
    $g.FillEllipse($glow, ($cx - $rHub * 2), ($cy - $rHub * 2), ($rHub * 4), ($rHub * 4))
    $glow.Dispose()
    $hub = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 235, 248, 255))
    $g.FillEllipse($hub, ($cx - $rHub), ($cy - $rHub), ($rHub * 2), ($rHub * 2))
    $hub.Dispose()

    # A green status LED in the corner, only where it can be seen.
    if ($s -ge 32) {
        $rLed = $s * 0.055
        $lx = $inset + $w - $rLed * 3.2
        $ly = $inset + $w - $rLed * 3.2
        $lg = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(80, 109, 224, 76))
        $g.FillEllipse($lg, ($lx - $rLed * 2), ($ly - $rLed * 2), ($rLed * 4), ($rLed * 4))
        $lg.Dispose()
        $ld = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 109, 224, 76))
        $g.FillEllipse($ld, ($lx - $rLed), ($ly - $rLed), ($rLed * 2), ($rLed * 2))
        $ld.Dispose()
    }

    $g.Dispose()
    $path.Dispose()
    return $bmp
}

# ---- render every size ---------------------------------------------------
#
# Sizes <= 64 are packed as uncompressed DIB (a BITMAPINFOHEADER, a bottom-up
# BGRA image, and an AND mask that is all zeroes because the alpha channel does
# the work). 128 and 256 are PNG, where the compression is worth having.

function Get-DibBytes([System.Drawing.Bitmap]$bmp) {
    $w = $bmp.Width; $h = $bmp.Height
    $rect = New-Object System.Drawing.Rectangle 0, 0, $w, $h
    $data = $bmp.LockBits($rect, [System.Drawing.Imaging.ImageLockMode]::ReadOnly,
                          [System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
    $stride = $data.Stride
    $buf = New-Object Byte[] ($stride * $h)
    [System.Runtime.InteropServices.Marshal]::Copy($data.Scan0, $buf, 0, $buf.Length)
    $bmp.UnlockBits($data)

    $ms = New-Object System.IO.MemoryStream
    $bw = New-Object System.IO.BinaryWriter $ms

    # BITMAPINFOHEADER. Height is doubled: the XOR image and the AND mask are
    # stacked, and the header describes both.
    $bw.Write([UInt32]40)
    $bw.Write([Int32]$w)
    $bw.Write([Int32]($h * 2))
    $bw.Write([UInt16]1)
    $bw.Write([UInt16]32)
    $bw.Write([UInt32]0)          # BI_RGB
    $bw.Write([UInt32]0)          # biSizeImage, may be 0 for BI_RGB
    $bw.Write([Int32]0); $bw.Write([Int32]0)
    $bw.Write([UInt32]0); $bw.Write([UInt32]0)

    # XOR image, bottom-up.
    for ($y = $h - 1; $y -ge 0; $y--) {
        $bw.Write($buf, $y * $stride, $w * 4)
    }

    # AND mask: 1bpp, rows padded to 4 bytes, all zero. Ignored for 32bpp icons
    # but the format requires it to be present.
    $maskRow = [int]([math]::Ceiling($w / 32.0) * 4)
    $zero = New-Object Byte[] $maskRow
    for ($y = 0; $y -lt $h; $y++) { $bw.Write($zero, 0, $maskRow) }

    $bw.Flush()
    $out = $ms.ToArray()
    $bw.Dispose(); $ms.Dispose()

    # The comma is load-bearing: PowerShell unrolls an array on return, so a bare
    # `return $out` emits 25,000 individual bytes into the pipeline instead of one
    # byte[], and every consumer downstream then sees object[].
    return ,$out
}

$blobs = @()
foreach ($s in $sizes) {
    $bmp = New-IconBitmap $s
    if ($s -le 64) {
        $blobs += ,@($s, (Get-DibBytes $bmp), $false)
    } else {
        $ms = New-Object System.IO.MemoryStream
        $bmp.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png)
        $blobs += ,@($s, $ms.ToArray(), $true)
        $ms.Dispose()
    }
    $bmp.Dispose()
}

# ---- pack the ICO ------------------------------------------------------
$fs = [System.IO.File]::Create($icoPath)
$bw = New-Object System.IO.BinaryWriter $fs

$bw.Write([UInt16]0)                 # reserved
$bw.Write([UInt16]1)                 # type: icon
$bw.Write([UInt16]$blobs.Count)

$offset = 6 + (16 * $blobs.Count)
foreach ($b in $blobs) {
    $s = [int]$b[0]; $data = $b[1]
    $dim = $(if ($s -ge 256) { 0 } else { $s })   # 0 means 256
    $bw.Write([Byte]$dim)
    $bw.Write([Byte]$dim)
    $bw.Write([Byte]0)               # palette count
    $bw.Write([Byte]0)               # reserved
    $bw.Write([UInt16]1)             # colour planes
    $bw.Write([UInt16]32)            # bits per pixel
    $bw.Write([UInt32]$data.Length)
    $bw.Write([UInt32]$offset)
    $offset += $data.Length
}
foreach ($b in $blobs) { $bw.Write($b[1]) }

$bw.Flush(); $bw.Dispose(); $fs.Dispose()

# ---- and the 256 on its own, for the README ----------------------------
#
# A browser is not required to render an .ico in an <img>, and the repository
# front page is the one place where "usually works" is not good enough. This is
# the SAME byte array already packed into the .ico above, written out a second
# time rather than re-rendered, so the two cannot drift apart - the failure
# this file would otherwise invite is a README icon that quietly stops matching
# the window icon.
$pngPath = Join-Path $outDir 'bibo.png'
foreach ($b in $blobs) {
    if ([int]$b[0] -eq 256) {
        [System.IO.File]::WriteAllBytes($pngPath, $b[1])
    }
}

Write-Output ("wrote {0} ({1} sizes, {2} bytes)" -f $icoPath, $blobs.Count,
              (Get-Item $icoPath).Length)
Write-Output ("wrote {0} ({1} bytes)" -f $pngPath, (Get-Item $pngPath).Length)
