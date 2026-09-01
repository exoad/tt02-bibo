# Generates hub/assets/bibo.ico — the application and window icon.
# Run from anywhere:  powershell -ExecutionPolicy Bypass -File hub\assets\make_icon.ps1
#
# The subject is the car, on the graphite plate the UI is built from, redrawn
# from primitives after Fugue's car.png - Fugue is 16x16, a smudge blown up.
#
# ENTRY FORMAT: uncompressed DIB for every size up to 64, PNG only at 128 and
# 256. PNG-in-ICO is legal since Vista but GDI+ (System.Drawing.Icon) cannot
# decode it, so an all-PNG .ico fails to render in a whole class of tooling while
# Explorer shows it fine, which is what makes that hard to spot.

Add-Type -AssemblyName System.Drawing

# A rounded rectangle - four arcs and a close, factored out because there are
# seven below and an inline copy is where the corners stop matching. The radius
# is CLAMPED to half the shorter side: at 16 pixels a wheel is three pixels tall,
# and a radius larger than the shape makes GDI+ draw a bow tie rather than refuse.
function New-RoundRect([single]$x, [single]$y, [single]$w, [single]$h, [single]$r) {
    $r = [math]::Min($r, [math]::Min($w, $h) / 2.0)
    $p = New-Object System.Drawing.Drawing2D.GraphicsPath
    if ($r -le 0.5) {
        $p.AddRectangle((New-Object System.Drawing.RectangleF $x, $y, $w, $h))
        return $p
    }
    $d = $r * 2
    $p.AddArc($x, $y, $d, $d, 180, 90)
    $p.AddArc(($x + $w - $d), $y, $d, $d, 270, 90)
    $p.AddArc(($x + $w - $d), ($y + $h - $d), $d, $d, 0, 90)
    $p.AddArc($x, ($y + $h - $d), $d, $d, 90, 90)
    $p.CloseFigure()
    return $p
}

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

    # The plate: rounded graphite, lit from above.
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

    # 1px bevel on top, dark seam around the plate - as ui::bevelRect does.
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

    # The subject: the car, head on. Everything is a fraction of $s so the layout
    # survives all eight sizes, and detail DROPS OUT as it shrinks - below 24 the
    # grille and badge are sub-pixel, and drawing them anyway turns the front of
    # the car into grey mud.
    $bodyL = $s * 0.155
    $bodyR = $s * 0.845
    $bodyW = $bodyR - $bodyL

    # Cabin: roof and windscreen, narrower than the body and sitting on it.
    $cabX = $s * 0.275
    $cabW = $s * 0.45
    $cabY = $s * 0.262
    $cabH = $s * 0.235
    $cab = New-RoundRect $cabX $cabY $cabW $cabH ($s * 0.085)
    $cabBrush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
        (New-Object System.Drawing.RectangleF $cabX, $cabY, $cabW, ($cabH * 1.02)),
        [System.Drawing.Color]::FromArgb(255, 226, 232, 238),
        [System.Drawing.Color]::FromArgb(255, 138, 150, 162),
        90.0)
    $g.FillPath($cabBrush, $cab)
    $cabBrush.Dispose()

    # The windscreen: what makes the shape read as the FRONT of a car.
    if ($s -ge 24) {
        $wsX = $cabX + $cabW * 0.10
        $wsW = $cabW * 0.80
        $wsY = $cabY + $cabH * 0.46
        $wsH = $cabH * 0.42
        $ws = New-RoundRect $wsX $wsY $wsW $wsH ($s * 0.03)
        $wsBrush = New-Object System.Drawing.SolidBrush (
            [System.Drawing.Color]::FromArgb(255, 96, 110, 124))
        $g.FillPath($wsBrush, $ws)
        $wsBrush.Dispose(); $ws.Dispose()
    }
    $cab.Dispose()

    # Body: the blue mass, lit from above like the plate under it.
    $bodyY = $s * 0.452
    $bodyH = $s * 0.245
    $body = New-RoundRect $bodyL $bodyY $bodyW $bodyH ($s * 0.075)
    $bodyBrush = New-Object System.Drawing.Drawing2D.LinearGradientBrush(
        (New-Object System.Drawing.RectangleF $bodyL, $bodyY, $bodyW, ($bodyH * 1.02)),
        [System.Drawing.Color]::FromArgb(255, 150, 205, 250),
        [System.Drawing.Color]::FromArgb(255, 46, 96, 150),
        90.0)
    $g.FillPath($bodyBrush, $body)
    $bodyBrush.Dispose()
    $body.Dispose()

    # Headlamps. These carry the icon at small sizes - two warm dots against blue
    # survive to 16 where nothing else does.
    $lampW = $s * 0.145
    $lampH = $s * 0.085
    $lampY = $bodyY + $bodyH * 0.20
    $lampBrush = New-Object System.Drawing.SolidBrush (
        [System.Drawing.Color]::FromArgb(255, 246, 190, 92))
    $l1 = New-RoundRect ($bodyL + $s * 0.022) $lampY $lampW $lampH ($s * 0.03)
    $l2 = New-RoundRect ($bodyR - $s * 0.022 - $lampW) $lampY $lampW $lampH ($s * 0.03)
    $g.FillPath($lampBrush, $l1)
    $g.FillPath($lampBrush, $l2)
    $lampBrush.Dispose(); $l1.Dispose(); $l2.Dispose()

    # Grille, with the badge in it. Both go at 24 and above only.
    if ($s -ge 24) {
        $grX = $s * 0.315
        $grW = $s * 0.37
        $grY = $bodyY + $bodyH * 0.30
        $grH = $bodyH * 0.34
        $gr = New-RoundRect $grX $grY $grW $grH ($s * 0.025)
        $grBrush = New-Object System.Drawing.SolidBrush (
            [System.Drawing.Color]::FromArgb(255, 24, 58, 96))
        $g.FillPath($grBrush, $gr)
        $grBrush.Dispose(); $gr.Dispose()

        $bdW = $s * 0.10
        $bd = New-RoundRect (($s / 2.0) - $bdW / 2) ($grY + $grH * 0.16) $bdW ($grH * 0.68) ($s * 0.02)
        $bdBrush = New-Object System.Drawing.SolidBrush (
            [System.Drawing.Color]::FromArgb(255, 198, 158, 78))
        $g.FillPath($bdBrush, $bd)
        $bdBrush.Dispose(); $bd.Dispose()
    }

    # Tyres, projecting BELOW the body so the car sits on something. Inside the
    # body's own rectangle they hide behind it and read as two notches of damage.
    $tyW = $s * 0.155
    $tyH = $s * 0.105
    $tyY = $bodyY + $bodyH - $s * 0.030
    $tyBrush = New-Object System.Drawing.SolidBrush (
        [System.Drawing.Color]::FromArgb(255, 22, 21, 23))
    $t1 = New-RoundRect ($bodyL + $s * 0.030) $tyY $tyW $tyH ($s * 0.030)
    $t2 = New-RoundRect ($bodyR - $s * 0.030 - $tyW) $tyY $tyW $tyH ($s * 0.030)
    $g.FillPath($tyBrush, $t1)
    $g.FillPath($tyBrush, $t2)
    $tyBrush.Dispose(); $t1.Dispose(); $t2.Dispose()

    $g.Dispose()
    $path.Dispose()
    return $bmp
}

# Render every size. <= 64 is packed as uncompressed DIB (a BITMAPINFOHEADER, a
# bottom-up BGRA image, and an all-zero AND mask because alpha does the work);
# 128 and 256 are PNG, where the compression is worth having.

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

    # BITMAPINFOHEADER. Height is doubled: XOR image and AND mask are stacked.
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

    # AND mask: 1bpp, 4-byte-padded rows, all zero. Unused at 32bpp, required anyway.
    $maskRow = [int]([math]::Ceiling($w / 32.0) * 4)
    $zero = New-Object Byte[] $maskRow
    for ($y = 0; $y -lt $h; $y++) { $bw.Write($zero, 0, $maskRow) }

    $bw.Flush()
    $out = $ms.ToArray()
    $bw.Dispose(); $ms.Dispose()

    # The comma is load-bearing: PowerShell unrolls an array on return, so a bare
    # `return $out` emits 25,000 loose bytes and every consumer sees object[].
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

# Pack the ICO.
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

# And the 256 on its own, for the README - a browser is not required to render an
# .ico in an <img>. Written from the SAME byte array packed above rather than
# re-rendered, so the README icon cannot quietly stop matching the window icon.
$pngPath = Join-Path $outDir 'bibo.png'
foreach ($b in $blobs) {
    if ([int]$b[0] -eq 256) {
        [System.IO.File]::WriteAllBytes($pngPath, $b[1])
    }
}

Write-Output ("wrote {0} ({1} sizes, {2} bytes)" -f $icoPath, $blobs.Count,
              (Get-Item $icoPath).Length)
Write-Output ("wrote {0} ({1} bytes)" -f $pngPath, (Get-Item $pngPath).Length)
