"""From the network's two heatmaps to tags: the CPU half of the NPU detector,
written once here in numpy so the C++ on the board has a reference to agree
with, and so the model can be scored before any board is involved.

    python decode.py runs/tagnet/tagnet.onnx [--samples N] [--frames DIR]

Peaks in the centre map are candidate tags; each takes the four corner peaks
that surround it in angle order; the 6x6 code is sampled through the
homography of that quad and matched against the family under all four
rotations, exactly as the library's quick_decode does, allowing hamming 2.
"""
import argparse
import math
import random
from pathlib import Path

import numpy as np

import synth

HERE = Path(__file__).resolve().parent

# The family's codes as 36-bit integers, in the library's bit order, so the
# same rotation trick applies: rotating the 6x6 grid is a permutation of bits.
def family_codes():
    codes = []
    for tag in synth.FAMILY:
        inner = tag[2:8, 2:8]
        codes.append(inner.copy())
    return codes


CODES = family_codes()


def rotate(grid, k):
    return np.rot90(grid, -k)


def match(bits, max_hamming=2):
    """The (id, hamming, rotation) with the fewest bit errors, or None."""
    best = None
    for tag_id, code in enumerate(CODES):
        for k in range(4):
            ham = int(np.count_nonzero(rotate(code, k) != bits))
            if ham <= max_hamming and (best is None or ham < best[1]):
                best = (tag_id, ham, k)
                if ham == 0:
                    return best
    return best


def peaks(heat, threshold, radius=3):
    """Local maxima above threshold, strongest first, none within `radius` of
    a stronger one, each refined to sub-pixel by its 3x3 centroid. Returns
    (x, y, score) in heatmap pixels."""
    h, w = heat.shape
    # A local maximum of its 3x3 first, so the shoulders of a wide peak are
    # never peaks of their own; then strongest first with a keep-out radius.
    padded = np.pad(heat, 1, mode="constant", constant_values=-1.0)
    neighbourhood = np.max(np.stack([padded[dy : dy + h, dx : dx + w] for dy in range(3) for dx in range(3)]), axis=0)
    ys, xs = np.nonzero((heat >= threshold) & (heat >= neighbourhood))
    order = np.argsort(-heat[ys, xs])
    kept = []
    for i in order:
        y, x = int(ys[i]), int(xs[i])
        if any(abs(x - kx) <= radius and abs(y - ky) <= radius for kx, ky, _ in kept):
            continue
        kept.append((x, y, float(heat[y, x])))
    out = []
    for x, y, s in kept:
        y0, y1 = max(0, y - 1), min(h, y + 2)
        x0, x1 = max(0, x - 1), min(w, x + 2)
        patch = heat[y0:y1, x0:x1]
        gy, gx = np.mgrid[y0:y1, x0:x1]
        mass = patch.sum()
        out.append((float((gx * patch).sum() / mass), float((gy * patch).sum() / mass), s))
    return out


def quads(centres, corners, max_span=60.0):
    """For each centre, the four nearest corners that sit one per quadrant of
    angle around it. Returns quads as 4x2 arrays in input pixels, counter-
    clockwise in image coordinates (y down), matching the library's order."""
    out = []
    for cx, cy, _ in centres:
        near = []
        for x, y, s in corners:
            d = math.hypot(x - cx, y - cy)
            if d <= max_span:
                near.append((math.atan2(y - cy, x - cx), d, x, y))
        if len(near) < 4:
            continue
        near.sort()
        # Four corners about 90 degrees apart: walk the sorted angles and pick
        # the best-fitting set by trying each start.
        best = None
        for i in range(len(near)):
            picked = [near[i]]
            for k in range(1, 4):
                want = near[i][0] + k * math.pi / 2
                want = (want + math.pi) % (2 * math.pi) - math.pi
                cand = min(near, key=lambda n: abs((n[0] - want + math.pi) % (2 * math.pi) - math.pi))
                picked.append(cand)
            err = sum(abs(((p[0] - picked[0][0] - k * math.pi / 2) + math.pi) % (2 * math.pi) - math.pi) for k, p in enumerate(picked))
            spread = max(p[1] for p in picked) / max(1e-6, min(p[1] for p in picked))
            if spread > 2.5:
                continue
            if best is None or err < best[0]:
                best = (err, picked)
        if best is None or best[0] > 1.2:
            continue
        pts = np.array([[p[2], p[3]] for p in best[1]]) * synth.STRIDE
        out.append(pts)
    return out


def sample_code(grey, quad):
    """The 6x6 code bits inside the black border whose corners are `quad`,
    thresholded against the border and the white ring around it."""
    src = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    hm = synth.solve_homography(src, quad)
    h, w = grey.shape

    def at(u, v):
        p = hm @ np.array([u, v, 1.0])
        x, y = p[0] / p[2], p[1] / p[2]
        xi, yi = int(round(x)), int(round(y))
        if 0 <= xi < w and 0 <= yi < h:
            return float(grey[yi, xi])
        return None

    # The black border is the unit square's outer cell ring (8 cells across),
    # so the 6x6 code cells are centred at (i + 1.5) / 8.
    blacks = [at((i + 0.5) / 8, 0.5 / 8) for i in range(8)] + [at(0.5 / 8, (i + 0.5) / 8) for i in range(8)]
    whites = [at((i + 0.5) / 8, -0.5 / 8) for i in range(8)] + [at(-0.5 / 8, (i + 0.5) / 8) for i in range(8)]
    blacks = [b for b in blacks if b is not None]
    whites = [b for b in whites if b is not None]
    if len(blacks) < 6 or len(whites) < 6:
        return None
    threshold = (np.median(blacks) + np.median(whites)) / 2
    bits = np.zeros((6, 6), dtype=np.uint8)
    for r in range(6):
        for c in range(6):
            v = at((c + 1.5) / 8, (r + 1.5) / 8)
            if v is None:
                return None
            bits[r, c] = 1 if v > threshold else 0
    return bits


def detect(grey, heat, centre_threshold=0.35, corner_threshold=0.3):
    """grey: HxW uint8 at the input size. heat: 2xhxw probabilities.
    Returns a list of (id, hamming, quad 4x2 px)."""
    found = []
    for quad in quads(peaks(heat[0], centre_threshold), peaks(heat[1], corner_threshold)):
        bits = sample_code(grey, quad)
        if bits is None:
            continue
        m = match(bits)
        if m is not None:
            found.append((m[0], m[1], quad))
    return found


def score(sess, samples, rng, backgrounds):
    """Detection and id accuracy on synthetic pictures whose truth is known."""
    tags_total = 0
    found_right = 0
    found_wrong = 0
    corner_err = []
    for _ in range(samples):
        grey, _, truth = synth.make_sample(rng, backgrounds)
        x = grey.astype(np.float32)[None, None] / 255.0
        heat = sess.run(None, {"picture": x})[0][0]
        got = detect(grey, heat)
        tags_total += len(truth)
        for tag_id, quad in truth:
            centre = quad.mean(axis=0)
            hit = [g for g in got if g[0] == tag_id and np.linalg.norm(g[2].mean(axis=0) - centre) < 8]
            if hit:
                found_right += 1
                corners = hit[0][2]
                err = min(np.linalg.norm(np.roll(corners, k, axis=0) - quad, axis=1).mean() for k in range(4))
                corner_err.append(err)
        for g in got:
            if not any(g[0] == t and np.linalg.norm(g[2].mean(axis=0) - q.mean(axis=0)) < 8 for t, q in truth):
                found_wrong += 1
    print(f"{samples} pictures, {tags_total} tags: {found_right} found with the right id ({100.0 * found_right / max(1, tags_total):.1f}%), {found_wrong} false")
    if corner_err:
        print(f"corner error: mean {np.mean(corner_err):.2f} px, worst {np.max(corner_err):.2f} px at input resolution")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx")
    ap.add_argument("--samples", type=int, default=200)
    args = ap.parse_args()
    import onnxruntime as ort

    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    bg = synth.Backgrounds(HERE / "frames" if (HERE / "frames").exists() else None)
    score(sess, args.samples, random.Random(99), bg)


if __name__ == "__main__":
    main()
