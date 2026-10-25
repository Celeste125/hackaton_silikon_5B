"""The location prior network: a small U-Net over the occupancy window.

Input  (B, 2, 128, 128)   occupied and unknown masks, robot-centric
Output (B, C, 128, 128)   one logit map per landmark, softmax over cells

Deliberately tiny: at the default base width 8 and depth 3 it is about 121k
parameters, trains in a couple of minutes on any GPU, and runs in well under a
millisecond on a laptop CPU through ONNX.

That size is a decision, not an accident. A dataset here is a few hundred
samples drawn from a handful of runs, and samples within a run are nearly
identical -- so the *independent* information is a few dozen data points. A
wider network (base 16, depth 4 is 1.9M parameters) fits that in a few epochs
and then memorises: in testing its validation loss tripled while its training
loss fell to zero. If you later have many runs from many arena layouts,
raising --base-width is the first thing to try.

Why a U-Net rather than a regressor to (x, y): the output is a *distribution*,
and the shape of that distribution is the useful part. A network that emits
one point cannot say "somewhere along this wall", and the estimator needs a
covariance to combine the prior correctly with the camera's bearings. The
skip connections matter for the same reason -- the prediction has to stay
registered to the walls it is reasoning about, and a bottleneck-only network
blurs that registration away.

Kept free of any ROS import so it can live on a GPU box with nothing else
installed. The inference side does not use this file at all: it loads the ONNX
export through onnxruntime.
"""

from __future__ import annotations

import torch
from torch import nn


def _block(in_channels: int, out_channels: int) -> nn.Sequential:
    """Two 3x3 convolutions. GroupNorm, not BatchNorm.

    BatchNorm would be a poor fit here: the batches are small, the samples
    within a run are highly correlated (consecutive map snapshots), and the
    running statistics then differ between training and the single-sample
    inference the robot does. GroupNorm sidesteps all three.
    """
    groups = min(8, out_channels)
    return nn.Sequential(
        nn.Conv2d(in_channels, out_channels, 3, padding=1, bias=False),
        nn.GroupNorm(groups, out_channels),
        nn.ReLU(inplace=True),
        nn.Conv2d(out_channels, out_channels, 3, padding=1, bias=False),
        nn.GroupNorm(groups, out_channels),
        nn.ReLU(inplace=True),
    )


class LandmarkPriorNet(nn.Module):
    """Predict, per landmark, a spatial distribution over the window."""

    def __init__(
        self,
        in_channels: int = 2,
        num_classes: int = 2,
        base_width: int = 8,
        depth: int = 3,
    ):
        super().__init__()
        if depth < 1:
            raise ValueError('depth must be at least 1')
        self.in_channels = in_channels
        self.num_classes = num_classes

        widths = [base_width * 2 ** level for level in range(depth + 1)]
        self.downs = nn.ModuleList()
        previous = in_channels
        for width in widths[:-1]:
            self.downs.append(_block(previous, width))
            previous = width
        self.bottleneck = _block(previous, widths[-1])
        self.pool = nn.MaxPool2d(2)

        self.ups = nn.ModuleList()
        self.up_blocks = nn.ModuleList()
        for width in reversed(widths[:-1]):
            self.ups.append(
                nn.ConvTranspose2d(width * 2, width, kernel_size=2, stride=2))
            self.up_blocks.append(_block(width * 2, width))

        self.head = nn.Conv2d(widths[0], num_classes, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        skips = []
        for block in self.downs:
            x = block(x)
            skips.append(x)
            x = self.pool(x)
        x = self.bottleneck(x)
        for up, block, skip in zip(self.ups, self.up_blocks, reversed(skips)):
            x = up(x)
            x = block(torch.cat([skip, x], dim=1))
        # Logits, not probabilities: the loss takes logits, and the robot
        # applies its own softmax over both spatial axes at inference.
        return self.head(x)

    def parameter_count(self) -> int:
        return sum(p.numel() for p in self.parameters())


def spatial_cross_entropy(
    logits: torch.Tensor,
    target_index: torch.Tensor,
    valid: torch.Tensor,
) -> torch.Tensor:
    """Cross-entropy over cells, per class, masked to usable labels.

    `logits` is (B, C, H, W), `target_index` is (B, C) holding the flattened
    cell index of the true location, and `valid` is (B, C) marking which of
    those are real -- a landmark the run never converged on, or one that fell
    outside the window, must contribute nothing rather than contribute a
    wrong answer.

    Returns the mean over valid entries, or zero when a batch has none.
    """
    batch, classes, height, width = logits.shape
    flat = logits.reshape(batch * classes, height * width)
    log_probabilities = torch.log_softmax(flat, dim=1)
    picked = log_probabilities.gather(
        1, target_index.reshape(-1, 1).clamp(min=0, max=height * width - 1))
    losses = -picked.reshape(batch, classes)

    mask = valid.to(losses.dtype)
    total = mask.sum()
    if float(total) == 0.0:
        return logits.sum() * 0.0
    return (losses * mask).sum() / total


def expected_error_cells(
    logits: torch.Tensor,
    target_index: torch.Tensor,
    valid: torch.Tensor,
) -> torch.Tensor:
    """Distance from the predicted mode to the true cell, in cells.

    Reported alongside the loss because cross-entropy is hard to read: this is
    in units you can multiply by the window resolution to get metres, which is
    the number that decides whether the prior is worth switching on.
    """
    batch, classes, height, width = logits.shape
    flat = logits.reshape(batch * classes, height * width)
    predicted = flat.argmax(dim=1)
    truth = target_index.reshape(-1).clamp(min=0, max=height * width - 1)
    delta_row = (predicted // width).float() - (truth // width).float()
    delta_column = (predicted % width).float() - (truth % width).float()
    distance = torch.sqrt(delta_row ** 2 + delta_column ** 2)
    mask = valid.reshape(-1).to(distance.dtype)
    total = mask.sum()
    if float(total) == 0.0:
        return distance.sum() * 0.0
    return (distance * mask).sum() / total
