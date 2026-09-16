"""The tag detector network, sized for the A733's NPU and written from the ops
the ACUITY toolkit is known to import: Conv, BatchNorm (folded at export),
ReLU, ConvTranspose and a Sigmoid on the way out. No Resize, no TopK, no
dynamic shape: the peak-picking that turns heatmaps into corners is the CPU's.

Grey 240x320 in, two heatmaps at 60x80 out: tag centres and tag corners.
About 350k parameters and 0.4 GFLOP a frame, well inside what the NPU does in a
few milliseconds.
"""
import torch
from torch import nn


def block(cin, cout, stride=1, dilation=1):
    return nn.Sequential(
        nn.Conv2d(cin, cout, 3, stride=stride, padding=dilation, dilation=dilation, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
    )


def up(cin, cout):
    return nn.Sequential(
        nn.ConvTranspose2d(cin, cout, 4, stride=2, padding=1, bias=False),
        nn.BatchNorm2d(cout),
        nn.ReLU(inplace=True),
    )


class TagNet(nn.Module):
    """Four stride-2 stages, the last dilated, so a pixel of the 60x80 output
    sees about 200 input pixels: the centre of a big tag is a uniform patch
    whose nearest edge is far away, and the first version of this net, three
    stages deep, found 11 percent of tags over 120 px against the CPU
    detector's 68."""

    def __init__(self):
        super().__init__()
        self.down1 = nn.Sequential(block(1, 16, 2), block(16, 16))                 # 120x160
        self.down2 = nn.Sequential(block(16, 32, 2), block(32, 32))                # 60x80
        self.down3 = nn.Sequential(block(32, 64, 2), block(64, 64))                # 30x40
        self.down4 = nn.Sequential(block(64, 96, 2), block(96, 96, dilation=2))    # 15x20
        self.up4 = up(96, 64)                                                      # 30x40
        self.up3 = up(64, 32)                                                      # 60x80
        self.head = nn.Sequential(block(32, 32), nn.Conv2d(32, 2, 1))

    def forward(self, x):
        d1 = self.down1(x)
        d2 = self.down2(d1)
        d3 = self.down3(d2)
        d4 = self.down4(d3)
        u3 = self.up4(d4) + d3
        u2 = self.up3(u3) + d2
        return self.head(u2)


class Deployed(nn.Module):
    """What is exported: uint8-scaled input already divided by 255 by the
    quantizer's input scale, and the sigmoid inside the graph so the NPU hands
    back probabilities."""

    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, x):
        return torch.sigmoid(self.net(x))


if __name__ == "__main__":
    net = TagNet()
    n = sum(p.numel() for p in net.parameters())
    y = net(torch.zeros(1, 1, 240, 320))
    print(f"{n} parameters, output {tuple(y.shape)}")
