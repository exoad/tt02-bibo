"""Trains the tag detector on synthetic pictures, then exports it as a static
ONNX graph the ACUITY toolkit can import, and checks that graph against the
PyTorch model with ONNX Runtime.

    python train.py [--epochs N] [--steps-per-epoch N] [--out runs/tagnet]

Writes <out>/tagnet.pt, <out>/tagnet.onnx (opset 12, simplified) and
<out>/inputs_outputs.txt, the line the pegasus scripts want.
"""
import argparse
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, IterableDataset

import synth
from model import Deployed, TagNet

HERE = Path(__file__).resolve().parent


class Synthetic(IterableDataset):
    def __init__(self, seed, backgrounds):
        self.seed = seed
        self.backgrounds = backgrounds

    def __iter__(self):
        info = torch.utils.data.get_worker_info()
        rng = random.Random(self.seed + (info.id if info else 0) * 7919)
        while True:
            grey, heat, _ = synth.make_sample(rng, self.backgrounds)
            x = torch.from_numpy(grey.astype(np.float32) / 255.0)[None]
            yield x, torch.from_numpy(heat)


def focal_loss(logits, target, alpha=2.0, beta=4.0):
    """CenterNet's penalty-reduced focal loss: every pixel counts, the ones
    near a peak count less, and the peaks themselves count most."""
    p = torch.sigmoid(logits).clamp(1e-4, 1 - 1e-4)
    pos = target.eq(1.0).float()
    neg = 1.0 - pos
    pos_loss = torch.log(p) * (1 - p) ** alpha * pos
    neg_loss = torch.log(1 - p) * p**alpha * (1 - target) ** beta * neg
    n = pos.sum().clamp(min=1.0)
    return -(pos_loss.sum() + neg_loss.sum()) / n


def export(net, out):
    net.eval()
    deployed = Deployed(net).cpu().eval()
    example = torch.zeros(1, 1, synth.HEIGHT, synth.WIDTH)
    onnx_path = out / "tagnet.onnx"
    torch.onnx.export(
        deployed,
        example,
        str(onnx_path),
        opset_version=12,
        input_names=["picture"],
        output_names=["heat"],
        dynamic_axes=None,
        do_constant_folding=True,
    )
    import onnx
    from onnxsim import simplify

    model = onnx.load(str(onnx_path))
    simplified, ok = simplify(model, overwrite_input_shapes={"picture": [1, 1, synth.HEIGHT, synth.WIDTH]})
    if not ok:
        raise SystemExit("onnxsim could not simplify the graph")
    onnx.save(simplified, str(onnx_path))
    ops = sorted({n.op_type for n in simplified.graph.node})
    print("onnx ops:", ", ".join(ops))
    # The graph answers the same as the model, to float precision.
    import onnxruntime as ort

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    rng = random.Random(3)
    bg = synth.Backgrounds(HERE / "frames" if (HERE / "frames").exists() else None)
    worst = 0.0
    for _ in range(5):
        grey, _, _ = synth.make_sample(rng, bg)
        x = grey.astype(np.float32)[None, None] / 255.0
        with torch.no_grad():
            want = deployed(torch.from_numpy(x)).numpy()
        got = sess.run(None, {"picture": x})[0]
        worst = max(worst, float(np.abs(got - want).max()))
    print(f"onnxruntime agrees with torch to {worst:.2e}")
    (out / "inputs_outputs.txt").write_text(
        f"--inputs picture --input-size-list '1,{synth.HEIGHT},{synth.WIDTH}' --outputs heat\n"
    )
    print("wrote", onnx_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=12)
    ap.add_argument("--steps-per-epoch", type=int, default=400)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--out", default=str(HERE / "runs" / "tagnet"))
    ap.add_argument("--export-only", action="store_true")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    net = TagNet().to(device)
    ckpt = out / "tagnet.pt"
    if args.export_only:
        net.load_state_dict(torch.load(ckpt, map_location=device))
        export(net, out)
        return
    backgrounds = synth.Backgrounds(HERE / "frames" if (HERE / "frames").exists() else None)
    print(f"{len(backgrounds.pictures)} real frames as backgrounds, training on {device}")
    loader = DataLoader(Synthetic(1, backgrounds), batch_size=args.batch, num_workers=6, persistent_workers=True)
    opt = torch.optim.AdamW(net.parameters(), lr=2e-3, weight_decay=1e-4)
    total = args.epochs * args.steps_per_epoch
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=2e-3, total_steps=total, pct_start=0.15)
    it = iter(loader)
    step = 0
    for epoch in range(args.epochs):
        net.train()
        began = time.time()
        running = 0.0
        for _ in range(args.steps_per_epoch):
            x, y = next(it)
            x = x.to(device, non_blocking=True)
            y = y.to(device, non_blocking=True)
            loss = focal_loss(net(x), y)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
            running += loss.item()
            step += 1
        print(f"epoch {epoch + 1}/{args.epochs}: loss {running / args.steps_per_epoch:.4f}, {time.time() - began:.0f} s")
        torch.save(net.state_dict(), ckpt)
    export(net, out)


if __name__ == "__main__":
    main()
