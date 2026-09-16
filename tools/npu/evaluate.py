"""The model against the CPU detector on the same pictures, by tag size.

    python evaluate.py make DIR              # 150 synthetic JPEGs with their truth
    python evaluate.py model DIR ONNX        # the model + decode.py over them -> model.json
    python evaluate.py compare DIR [cpu.txt] # the table

cpu.txt is the board's answer for the same JPEGs, made with the apriltag
library's own detector (the `probe` program from the first day, on the board):

    scp DIR/*.jpg jack@bibobox:/tmp/eval/
    ssh jack@bibobox 'cd /tmp/eval && for f in e*.jpg; do echo "== $f";
        taskset -c 7 ~/apriltag/probe $f 1 1 2 | grep -E "^  id|per frame"; done' > DIR/cpu.txt

The set is made with a fixed seed, so two models are scored on the same
pictures, and it is deliberately hard: tags down to 22 px (2 px cells), tilts
to 55 percent, blur, noise and exposure. A tag counts as found when its id is
right and the reported centre is within 8 px of the truth.
"""
import json
import math
import random
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import synth

HERE = Path(__file__).resolve().parent
BINS = [(0, 30), (30, 45), (45, 70), (70, 120), (120, 400)]


def make(folder, count=150, seed=2024):
    folder.mkdir(parents=True, exist_ok=True)
    bg = synth.Backgrounds(HERE / "frames" if (HERE / "frames").exists() else None)
    rng = random.Random(seed)
    truth = {}
    n = 0
    while n < count:
        grey, _, tags = synth.make_sample(rng, bg)
        if not tags:
            continue
        name = f"e{n:03d}"
        Image.fromarray(grey).save(folder / f"{name}.jpg", quality=90)
        truth[name] = [(int(t), q.tolist()) for t, q in tags]
        n += 1
    json.dump(truth, open(folder / "truth.json", "w"))
    print(f"{n} pictures, {sum(len(v) for v in truth.values())} tags in {folder}")


def model(folder, onnx_path):
    import onnxruntime as ort

    import decode

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    truth = json.load(open(folder / "truth.json"))
    found = {}
    for name in truth:
        grey = np.asarray(Image.open(folder / f"{name}.jpg").convert("L"))
        heat = sess.run(None, {"picture": grey.astype(np.float32)[None, None] / 255.0})[0][0]
        found[name] = [(int(i), int(h), q.tolist()) for i, h, q in decode.detect(grey, heat)]
    json.dump(found, open(folder / "model.json", "w"))
    print(f"model answers for {len(found)} pictures written")


def read_cpu(path):
    cpu = {}
    times = []
    cur = None
    for line in open(path):
        if line.startswith("=="):
            cur = line.split()[1].replace(".jpg", "")
            cpu[cur] = []
        elif line.startswith("  id"):
            m = re.match(r"\s+id (\d+) ham (\d+) margin ([\d.]+) centre \(([\d.\-]+),([\d.\-]+)\)", line)
            cpu[cur].append((int(m.group(1)), float(m.group(4)), float(m.group(5))))
        elif "per frame" in line:
            times.append(float(re.search(r"detect ([\d.]+) ms", line).group(1)))
    return cpu, times


def side(q):
    q = np.array(q)
    return float(np.mean([np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)]))


def hit(answers, tid, centre):
    return any(i == tid and math.hypot(x - centre[0], y - centre[1]) < 8 for i, x, y in answers)


def compare(folder, cpu_path=None):
    truth = json.load(open(folder / "truth.json"))
    model_answers = json.load(open(folder / "model.json"))
    model_c = {n: [(i, *np.array(q).mean(axis=0)) for i, h, q in v] for n, v in model_answers.items()}
    cpu_c, times = read_cpu(cpu_path) if cpu_path else ({}, [])
    stats = {b: [0, 0, 0] for b in BINS}
    for name, tags in truth.items():
        for tid, q in tags:
            c = np.array(q).mean(axis=0)
            b = next(b for b in BINS if b[0] <= side(q) < b[1])
            stats[b][0] += 1
            stats[b][1] += hit(model_c.get(name, []), tid, c)
            stats[b][2] += hit(cpu_c.get(name, []), tid, c)
    print("tag side px   tags   model        cpu (apriltag)")
    for b in BINS:
        t, m, c = stats[b]
        print(f"{b[0]:3d}-{b[1]:<3d}       {t:4d}   {m:4d} ({100 * m / max(1, t):3.0f}%)   {c:4d} ({100 * c / max(1, t):3.0f}%)")
    t = sum(v[0] for v in stats.values())
    m = sum(v[1] for v in stats.values())
    c = sum(v[2] for v in stats.values())
    print(f"all           {t:4d}   {m:4d} ({100 * m / t:3.0f}%)   {c:4d} ({100 * c / t:3.0f}%)")

    def false_count(answers):
        n = 0
        for name, found in answers.items():
            for i, x, y in found:
                if not any(i == tid and math.hypot(x - np.mean(np.array(q)[:, 0]), y - np.mean(np.array(q)[:, 1])) < 8 for tid, q in truth[name]):
                    n += 1
        return n

    line = f"false ids: model {false_count(model_c)}"
    if cpu_c:
        line += f", cpu {false_count(cpu_c)}; cpu detect {np.mean(times):.1f} ms a frame at 320x240"
    print(line)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    what, folder = sys.argv[1], Path(sys.argv[2])
    if what == "make":
        make(folder)
    elif what == "model":
        model(folder, Path(sys.argv[3]))
    elif what == "compare":
        compare(folder, Path(sys.argv[3]) if len(sys.argv) > 3 else None)
    else:
        print(__doc__)
        sys.exit(2)
