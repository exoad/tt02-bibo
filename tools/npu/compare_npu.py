"""The NPU's answer against the model's, on one real frame.

    python compare_npu.py make IMAGE OUT.i16      # the frame as the NBG's int16 input
    python compare_npu.py check IMAGE ONNX DUMP.0.bin [--fl 15]

`make` writes the picture at 320x240 grey as the network binary's input tensor:
dynamic fixed point int16 with `fl` fractional bits of the raw 0..255 intensity
(the 1/255 scale lives inside the graph), NCHW, which for one channel is just
the rows. `check` reads the tensor `npu_probe` dumped, dequantises it as
value / 2^fl, and compares it with ONNX Runtime's heatmaps: cosine
similarity, worst difference, and whether the decoder finds the same tags.
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image

import decode
import synth


def load_grey(path):
    return np.asarray(Image.open(path).convert("L").resize((synth.WIDTH, synth.HEIGHT), Image.BILINEAR), dtype=np.uint8)


def make(image, out, fl=7):
    grey = load_grey(image)
    q = np.clip(np.round(grey.astype(np.float32) * (1 << fl)), -32768, 32767).astype("<i2")
    q.tofile(out)
    print(f"{out}: {q.size} int16 values, fl {fl}, from {image}")


def check(image, onnx_path, dump, fl=15):
    import onnxruntime as ort

    grey = load_grey(image)
    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    want = sess.run(None, {"picture": grey.astype(np.float32)[None, None] / 255.0})[0][0]
    raw = np.fromfile(dump, dtype="<i2")
    got = raw.astype(np.float32) / float(1 << fl)
    if got.size != want.size:
        raise SystemExit(f"dump holds {got.size} values, the model gives {want.size}")
    got = got.reshape(want.shape)   # CHW, W fastest, whatever the NPU reports the order as
    cos = float(np.dot(got.ravel(), want.ravel()) / (np.linalg.norm(got) * np.linalg.norm(want) + 1e-9))
    print(f"cosine {cos:.5f}, worst |diff| {np.abs(got - want).max():.4f}, model max {want.max():.3f}, npu max {got.max():.3f}")
    for name, heat in (("model", want), ("npu", got)):
        found = decode.detect(grey, heat)
        print(f"  {name}: {[(i, h, q.mean(axis=0).round(1).tolist()) for i, h, q in found]}")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(2)
    if sys.argv[1] == "make":
        make(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == "check":
        fl = int(sys.argv[sys.argv.index("--fl") + 1]) if "--fl" in sys.argv else 15
        check(sys.argv[2], sys.argv[3], sys.argv[4], fl)
    else:
        print(__doc__)
        sys.exit(2)
