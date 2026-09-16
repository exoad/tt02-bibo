"""Synthetic training pictures for the tag detector: tag36h11 tags, as the
AprilTag library itself renders them, warped into real camera frames.

A sample is a grey picture at the detector's input size and two heatmaps at a
quarter of it: where tag centres are, and where tag corners are. Corners are
one channel, not four, on purpose: which corner is "top-left" is not a fact
about a tag lying at 45 degrees, and the decoder tries all four rotations of
the code anyway, exactly as the library does.

Every number a tag ends up with here (its four corners in input pixels) is
also what the label is, so the label cannot drift from the picture.
"""
import math
import random
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

HERE = Path(__file__).resolve().parent

# The detector's input, half the camera's 640x480 - the same decimation the
# CPU detector uses for its quad search. Output heatmaps are a quarter of this.
WIDTH = 320
HEIGHT = 240
STRIDE = 4
OUT_W = WIDTH // STRIDE
OUT_H = HEIGHT // STRIDE

# The tag family: every tag as its 10x10 cell picture (white ring, black
# border, 6x6 code), dumped from apriltag_to_image on the board.
FAMILY_FILE = HERE / "tag36h11.txt"


def load_family(path=FAMILY_FILE):
    lines = path.read_text().split("\n")
    count, side = (int(v) for v in lines[0].split())
    tags = []
    at = 1
    for _ in range(count):
        rows = lines[at : at + side]
        at += side
        tags.append(np.array([[c == "1" for c in row] for row in rows], dtype=np.uint8))
    return tags


FAMILY = load_family()


def random_homography(size, rng):
    """Maps the unit square (the tag's 10 cells) to a quadrilateral of about
    `size` pixels across, tilted and turned at random. Returns the 3x3 matrix
    and the four destination corners of the unit square in order."""
    tilt = rng.uniform(0.0, 0.55)  # how far the far edge shrinks, 0 = square on
    yaw = rng.uniform(0.0, 2.0 * math.pi)
    # A square, foreshortened along one axis (a tilt), then rotated in the plane.
    src = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    dst = src - 0.5
    # Perspective: the top edge narrower than the bottom by the tilt.
    for i in range(4):
        shrink = 1.0 - tilt * (0.5 - dst[i, 1])
        dst[i, 0] *= shrink
    dst[:, 1] *= 1.0 - tilt * 0.5
    c, s = math.cos(yaw), math.sin(yaw)
    dst = dst @ np.array([[c, -s], [s, c]]).T
    dst *= size
    return dst


def solve_homography(src, dst):
    a = []
    for (x, y), (u, v) in zip(src, dst):
        a.append([x, y, 1, 0, 0, 0, -u * x, -u * y, -u])
        a.append([0, 0, 0, x, y, 1, -v * x, -v * y, -v])
    _, _, vt = np.linalg.svd(np.array(a))
    h = vt[-1].reshape(3, 3)
    return h / h[2, 2]


def render_tag(tag, corners_px, canvas):
    """Draws `tag` (10x10 cells) onto `canvas` (float32 HxW, 0..255) so that the
    unit square lands on corners_px. Inverse-mapped, one sample per pixel."""
    h, w = canvas.shape
    xs = corners_px[:, 0]
    ys = corners_px[:, 1]
    x0, x1 = int(max(0, math.floor(xs.min()))), int(min(w, math.ceil(xs.max()) + 1))
    y0, y1 = int(max(0, math.floor(ys.min()))), int(min(h, math.ceil(ys.max()) + 1))
    if x1 <= x0 or y1 <= y0:
        return
    src = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    inv = np.linalg.inv(solve_homography(src, corners_px))
    gy, gx = np.mgrid[y0:y1, x0:x1]
    pts = np.stack([gx.ravel() + 0.5, gy.ravel() + 0.5, np.ones(gx.size)], axis=0)
    uvw = inv @ pts
    u = uvw[0] / uvw[2]
    v = uvw[1] / uvw[2]
    inside = (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
    cu = np.clip((u * 10).astype(int), 0, 9)
    cv = np.clip((v * 10).astype(int), 0, 9)
    white = tag[cv, cu].astype(np.float32)
    ink = np.where(white > 0, 235.0, 18.0)
    patch = canvas[y0:y1, x0:x1].ravel()
    patch[inside] = ink[inside]
    canvas[y0:y1, x0:x1] = patch.reshape(y1 - y0, x1 - x0)


def black_corners(corners_px):
    """The corners the detector reports: the black border's, one cell in from
    the unit square's, found by mapping (0.1,0.1)..(0.9,0.9) through the same
    homography."""
    src = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    hm = solve_homography(src, corners_px)
    inner = np.array([[0.1, 0.1, 1], [0.9, 0.1, 1], [0.9, 0.9, 1], [0.1, 0.9, 1]]).T
    out = hm @ inner
    return (out[:2] / out[2]).T


def gaussian_splat(heat, x, y, sigma=1.0):
    """Adds a peak at output-resolution (x, y) to `heat`, taking the max so two
    close peaks do not sum past one."""
    r = int(math.ceil(3 * sigma))
    cx, cy = int(round(x)), int(round(y))
    for yy in range(max(0, cy - r), min(OUT_H, cy + r + 1)):
        for xx in range(max(0, cx - r), min(OUT_W, cx + r + 1)):
            d2 = (xx - x) ** 2 + (yy - y) ** 2
            heat[yy, xx] = max(heat[yy, xx], math.exp(-d2 / (2 * sigma * sigma)))
    # The nearest pixel is exactly 1: the focal loss counts positives as the
    # pixels AT one, and a gaussian centred between pixels never reaches it.
    if 0 <= cy < OUT_H and 0 <= cx < OUT_W:
        heat[cy, cx] = 1.0


class Backgrounds:
    """Real frames from the car's camera, decimated to the input size, plus
    synthetic gradients and noise so the net never learns one room."""

    def __init__(self, folder):
        self.pictures = []
        for p in sorted(Path(folder).glob("*.jpg")) if folder else []:
            im = Image.open(p).convert("L").resize((WIDTH, HEIGHT), Image.BILINEAR)
            self.pictures.append(np.asarray(im, dtype=np.float32))

    def pick(self, rng):
        if self.pictures and rng.random() < 0.75:
            im = self.pictures[rng.randrange(len(self.pictures))].copy()
            if rng.random() < 0.5:
                im = im[:, ::-1].copy()
            return im
        # A gradient with noise: cheap, and every intensity the camera sees.
        a = rng.uniform(20, 200)
        b = rng.uniform(20, 200)
        gy, gx = np.mgrid[0:HEIGHT, 0:WIDTH].astype(np.float32)
        t = (gx / WIDTH) * rng.uniform(-1, 1) + (gy / HEIGHT) * rng.uniform(-1, 1)
        im = a + (b - a) * (t - t.min()) / max(1e-6, t.max() - t.min())
        return im + np.random.default_rng(rng.getrandbits(32)).normal(0, rng.uniform(2, 12), im.shape)


def make_sample(rng, backgrounds, min_size=22, max_size=190):
    """One picture and its two heatmaps. Returns (grey uint8 HxW, heat 2xOUT_HxOUT_W,
    tags) where tags lists (id, corners 4x2 input px) for evaluation."""
    canvas = backgrounds.pick(rng)
    heat = np.zeros((2, OUT_H, OUT_W), dtype=np.float32)
    tags = []
    count = rng.choice([0, 1, 1, 1, 2, 2, 3])
    placed = []
    for _ in range(count):
        for _attempt in range(10):
            size = rng.uniform(min_size, max_size)
            quad = random_homography(size, rng)
            cx = rng.uniform(size * 0.35, WIDTH - size * 0.35)
            cy = rng.uniform(size * 0.35, HEIGHT - size * 0.35)
            corners = quad + np.array([cx, cy])
            if all(math.hypot(cx - px, cy - py) > (size + ps) * 0.6 for px, py, ps in placed):
                break
        else:
            continue
        placed.append((cx, cy, size))
        tag_id = rng.randrange(len(FAMILY))
        render_tag(FAMILY[tag_id], corners, canvas)
        black = black_corners(corners)
        centre = black.mean(axis=0)
        sigma = max(0.8, min(2.5, size / 60.0))
        gaussian_splat(heat[0], centre[0] / STRIDE, centre[1] / STRIDE, sigma)
        for x, y in black:
            gaussian_splat(heat[1], x / STRIDE, y / STRIDE, sigma * 0.8)
        tags.append((tag_id, black))
    # The camera's own degradations: blur, noise, exposure.
    im = Image.fromarray(np.clip(canvas, 0, 255).astype(np.uint8))
    if rng.random() < 0.6:
        im = im.filter(ImageFilter.GaussianBlur(rng.uniform(0.3, 1.6)))
    arr = np.asarray(im, dtype=np.float32)
    gain = rng.uniform(0.55, 1.35)
    bias = rng.uniform(-40, 40)
    arr = arr * gain + bias
    arr += np.random.default_rng(rng.getrandbits(32)).normal(0, rng.uniform(0, 10), arr.shape)
    return np.clip(arr, 0, 255).astype(np.uint8), heat, tags


if __name__ == "__main__":
    import sys

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "samples"
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(1)
    bg = Backgrounds(HERE / "frames" if (HERE / "frames").exists() else None)
    for i in range(8):
        grey, heat, tags = make_sample(rng, bg)
        Image.fromarray(grey).save(out / f"sample{i}.png")
        peaks = (np.clip(heat.max(axis=0) * 255, 0, 255)).astype(np.uint8)
        Image.fromarray(peaks).resize((WIDTH, HEIGHT), Image.NEAREST).save(out / f"sample{i}_heat.png")
        print(i, [(t, c.round(1).tolist()) for t, c in tags])
