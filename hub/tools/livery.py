"""Repaints the car atlas into the Subaru World Rally Team scheme.

    python tools/livery.py

Kenney's Car Kit textures every model from one 512x512 palette atlas: each
triangle's UV points at a solid swatch rather than at a painted panel. That is
why this is a script and not an image edit - the "body paint" is not a region of
a car-shaped texture, it is whichever swatch the `body` group's triangles happen
to sample, and the only reliable way to find it is to ask the mesh.

So: read car.obj, take the triangles in the `body` group, sample the atlas at
their UVs, and the most common color among them IS the paint. Then recolor
every pixel of that exact color.

The result is written back over assets/models/colormap.png. The kit is CC0, so
modifying it is permitted; the modification is recorded in ATTRIBUTION.md.

WHAT THIS IS NOT: it does not make the model an Impreza. It is a generic Kenney
saloon in the right colors. The shape is still a stand-in - see ATTRIBUTION.md.
"""
import io, os, struct, sys, zlib
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'assets', 'models')
PNG = os.path.join(MODELS, 'colormap.png')
OBJ = os.path.join(MODELS, 'car.obj')

# Subaru World Rally Team, 1999. Deep blue body; the yellow is decal work this
# palette has no room for, so only the paint is set.
LIVERY = (0x14, 0x3C, 0x96)


def read_png(path):
    d = open(path, 'rb').read()
    assert d[:8] == b'\x89PNG\r\n\x1a\n', 'not a PNG'
    idat, w, h = b'', 0, 0
    i = 8
    while i < len(d):
        ln = struct.unpack('>I', d[i:i + 4])[0]
        typ = d[i + 4:i + 8]
        body = d[i + 8:i + 8 + ln]
        if typ == b'IHDR':
            w, h, bd, ct = struct.unpack('>IIBB', body[:10])
            assert bd == 8 and ct == 6, 'expected 8-bit RGBA, got bd=%d ct=%d' % (bd, ct)
        elif typ == b'IDAT':
            idat += body
        elif typ == b'IEND':
            break
        i += 12 + ln

    raw = zlib.decompress(idat)
    stride = w * 4
    out = bytearray(w * h * 4)
    prev = bytearray(stride)
    pos = 0
    for y in range(h):
        f = raw[pos]; pos += 1
        line = bytearray(raw[pos:pos + stride]); pos += stride
        # PNG filters. Paeth is the only one that needs a helper.
        if f == 1:
            for x in range(4, stride):
                line[x] = (line[x] + line[x - 4]) & 0xFF
        elif f == 2:
            for x in range(stride):
                line[x] = (line[x] + prev[x]) & 0xFF
        elif f == 3:
            for x in range(stride):
                a = line[x - 4] if x >= 4 else 0
                line[x] = (line[x] + ((a + prev[x]) >> 1)) & 0xFF
        elif f == 4:
            for x in range(stride):
                a = line[x - 4] if x >= 4 else 0
                b = prev[x]
                c = prev[x - 4] if x >= 4 else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[x] = (line[x] + pr) & 0xFF
        elif f != 0:
            raise ValueError('unknown PNG filter %d' % f)
        out[y * stride:(y + 1) * stride] = line
        prev = line
    return w, h, out


def write_png(path, w, h, px):
    stride = w * 4
    raw = bytearray()
    for y in range(h):
        raw.append(0)                       # filter 0: none
        raw += px[y * stride:(y + 1) * stride]

    def chunk(typ, body):
        return (struct.pack('>I', len(body)) + typ + body
                + struct.pack('>I', zlib.crc32(typ + body) & 0xFFFFFFFF))

    out = b'\x89PNG\r\n\x1a\n'
    out += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0))
    out += chunk(b'IDAT', zlib.compress(bytes(raw), 9))
    out += chunk(b'IEND', b'')
    open(path, 'wb').write(out)


def body_uvs(path):
    """Every UV used by a triangle in the `body` group."""
    uvs, out, grp = [], [], None
    for line in io.open(path, encoding='utf-8', errors='replace'):
        if line.startswith('vt '):
            p = line.split()
            uvs.append((float(p[1]), float(p[2])))
        elif line.startswith('g '):
            grp = line[2:].strip()
        elif line.startswith('f ') and grp == 'body':
            for tok in line.split()[1:]:
                bits = tok.split('/')
                if len(bits) > 1 and bits[1]:
                    k = int(bits[1])
                    k = k - 1 if k > 0 else len(uvs) + k
                    if 0 <= k < len(uvs):
                        out.append(uvs[k])
    return out


def main():
    w, h, px = read_png(PNG)
    uv = body_uvs(OBJ)
    if not uv:
        print('no body UVs found'); return 1

    hist = Counter()
    for u, v in uv:
        x = min(w - 1, max(0, int(u * w)))
        y = min(h - 1, max(0, int((1.0 - v) * h)))
        i = (y * w + x) * 4
        hist[(px[i], px[i + 1], px[i + 2])] += 1

    # The most common body swatch is NOT the paint: the grays that dominate are
    # the underbody, which nobody ever sees. Paint is the most common CHROMATIC
    # swatch - a car's color is a color, and its chassis is not.
    def chroma(c):
        return max(c) - min(c)

    paint = None
    for c, _n in hist.most_common():
        if chroma(c) >= 40:
            paint = c
            break
    if paint is None:
        print('no chromatic swatch among the body UVs; nothing to repaint'); return 1

    print('body samples: %d, paint #%02X%02X%02X (chroma %d, used %d)'
          % (len(uv), paint[0], paint[1], paint[2], chroma(paint), hist[paint]))
    if paint == LIVERY:
        print('already in the livery color; nothing to do'); return 0

    # Remapped by NEARNESS and luminance-preserving, not by exact match. The
    # atlas has several near-identical shades per region - shading baked into
    # the palette - and an exact-match replace would repaint one of them and
    # leave the car in two colors.
    def lum(c):
        return (c[0] * 299 + c[1] * 587 + c[2] * 114) // 1000

    base = lum(paint)
    changed = 0
    for i in range(0, len(px), 4):
        c = (px[i], px[i + 1], px[i + 2])
        d = abs(c[0] - paint[0]) + abs(c[1] - paint[1]) + abs(c[2] - paint[2])
        if d > 90:
            continue
        # Keep this texel's own light level relative to the swatch it came from.
        k = lum(c) - base
        px[i]     = min(255, max(0, LIVERY[0] + k))
        px[i + 1] = min(255, max(0, LIVERY[1] + k))
        px[i + 2] = min(255, max(0, LIVERY[2] + k))
        changed += 1

    write_png(PNG, w, h, px)
    print('repainted %d texel(s) %s -> %s' % (changed, paint, LIVERY))
    return 0


if __name__ == '__main__':
    sys.exit(main())
