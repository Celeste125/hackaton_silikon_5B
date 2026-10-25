#!/usr/bin/env python3
"""Train the location prior network and export it for the robot.

    ./train_landmark_prior.py --dataset landmark_prior/model/dataset.npz \
        --out-dir landmark_prior/model --epochs 60

Writes three things next to each other: the torch checkpoint (for resuming or
fine-tuning), the ONNX export (the only one the robot loads), and a manifest
recording what went into it. Keeping the manifest matters because the network
is tied to a window geometry and a channel order -- a model whose provenance
is unknown cannot be safely deployed, only retrained.

Reading the output: `mode_err` is the distance from the network's most likely
cell to the true one, converted to metres. That is the number to judge it by.

  under ~0.3 m   genuinely useful as a prior; switch it on
  0.3 - 0.8 m    weak, but harmless -- it will be outvoted quickly
  worse, or val much worse than train   it has memorised the training runs.
                 More runs, not more epochs.

Expect that last case at first. One arena gives the network very little to
generalise over, and a few runs is a small dataset; that is a property of the
problem, not a bug in the training.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from landmark_prior_net import (  # noqa: E402
    LandmarkPriorNet,
    expected_error_cells,
    spatial_cross_entropy,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--out-dir', required=True, type=Path)
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--learning-rate', type=float, default=2e-3)
    parser.add_argument('--weight-decay', type=float, default=1e-4)
    parser.add_argument(
        '--base-width', type=int, default=8,
        help='Network width. The default is small on purpose -- see '
             'landmark_prior_net.py. Raise it only with many runs.')
    parser.add_argument('--depth', type=int, default=3)
    parser.add_argument(
        '--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument(
        '--select-on', default='loss', choices=['loss', 'mode'],
        help="Which validation number picks the saved epoch. 'loss' (the "
             'default) scores the whole predicted distribution; "mode" scores '
             'only its peak. Prefer loss: the prior is built from the '
             "heatmap's mean *and covariance*, and a network goes "
             'overconfident -- peak still right, spread far too tight -- '
             'well before its mode degrades.')
    parser.add_argument(
        '--export-only', action='store_true',
        help='Skip training and re-export the existing checkpoint to ONNX. '
             'For re-exporting after an onnxruntime upgrade, or exporting a '
             'checkpoint trained elsewhere.')
    parser.add_argument(
        '--augment-flip', action='store_true',
        help='Mirror samples left/right. Valid only if the arena and the '
             'landmark layout have no handedness -- check before using it.')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--name', default='landmark_prior_v1')
    return parser.parse_args()


def load_split(dataset, prefix: str, device: str):
    channels = torch.from_numpy(
        dataset[f'{prefix}_channels'].astype(np.float32))
    target = torch.from_numpy(dataset[f'{prefix}_target'].astype(np.int64))
    valid = torch.from_numpy(dataset[f'{prefix}_valid'].astype(bool))
    return TensorDataset(channels, target, valid)


def mirror(channels, target, valid, cells: int):
    """Flip left/right, moving the labels with the image.

    The window's j axis is 'left of the robot', so mirroring is a flip of the
    last axis and j -> cells-1-j for every label. The caller has to decide
    whether that is physically meaningful: it is only valid if a mirrored
    arena is as likely as the real one.
    """
    flipped = torch.flip(channels, dims=[-1])
    rows = torch.div(target, cells, rounding_mode='floor')
    columns = target % cells
    return flipped, rows * cells + (cells - 1 - columns), valid


def run_epoch(model, loader, device, cells, optimiser=None, augment=False):
    training = optimiser is not None
    model.train(training)
    totals = {'loss': 0.0, 'cells': 0.0, 'count': 0}

    for channels, target, valid in loader:
        channels = channels.to(device)
        target = target.to(device)
        valid = valid.to(device)
        if training and augment and bool(torch.rand(1) < 0.5):
            channels, target, valid = mirror(channels, target, valid, cells)

        with torch.set_grad_enabled(training):
            logits = model(channels)
            loss = spatial_cross_entropy(logits, target, valid)
        if training:
            optimiser.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimiser.step()

        with torch.no_grad():
            error = expected_error_cells(logits, target, valid)
        batch = channels.shape[0]
        totals['loss'] += float(loss.detach()) * batch
        totals['cells'] += float(error.detach()) * batch
        totals['count'] += batch

    count = max(totals['count'], 1)
    return totals['loss'] / count, totals['cells'] / count


def main():
    args = parse_args()
    torch.manual_seed(args.seed)

    dataset = np.load(args.dataset, allow_pickle=False)
    meta = json.loads(str(dataset['meta']))
    classes = [str(name) for name in dataset['classes']]
    cells = int(meta['window_cells'])
    resolution = float(meta['window_resolution_m'])

    print(f'classes: {classes}')
    print(f'window: {cells} cells at {resolution} m')
    print(f'train runs: {meta["train_runs"]}')
    print(f'val runs:   {meta["val_runs"]}')

    train_set = load_split(dataset, 'train', args.device)
    val_set = load_split(dataset, 'val', args.device)
    if len(train_set) == 0:
        raise SystemExit('the dataset has no training samples')

    train_loader = DataLoader(
        train_set, batch_size=args.batch_size, shuffle=True, drop_last=False)
    val_loader = (
        DataLoader(val_set, batch_size=args.batch_size) if len(val_set)
        else None)

    model = LandmarkPriorNet(
        in_channels=train_set[0][0].shape[0],
        num_classes=len(classes),
        base_width=args.base_width,
        depth=args.depth,
    ).to(args.device)
    print(f'model: {model.parameter_count()} parameters on {args.device}')

    optimiser = torch.optim.AdamW(
        model.parameters(), lr=args.learning_rate,
        weight_decay=args.weight_decay)
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=max(args.epochs, 1))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_path = args.out_dir / f'{args.name}.pt'
    best = float('inf')
    best_epoch = -1
    started = time.time()

    epochs = 0 if args.export_only else args.epochs
    if args.export_only and not checkpoint_path.is_file():
        raise SystemExit(f'--export-only needs {checkpoint_path} to exist')

    for epoch in range(1, epochs + 1):
        loss, error = run_epoch(
            model, train_loader, args.device, cells, optimiser,
            augment=args.augment_flip)
        line = (
            f'epoch {epoch:3d}  train loss {loss:7.4f}  '
            f'mode_err {error * resolution:5.3f} m')

        if val_loader is not None:
            val_loss, val_error = run_epoch(
                model, val_loader, args.device, cells)
            line += (
                f'   |  val loss {val_loss:7.4f}  '
                f'mode_err {val_error * resolution:5.3f} m')
            score = val_loss if args.select_on == 'loss' else val_error
        else:
            score = loss if args.select_on == 'loss' else error

        # Selected on the held-out runs when there are any. Without them this
        # keeps the best training score, which is not a real model selection
        # and is reported as such in the manifest.
        if score < best:
            best, best_epoch = score, epoch
            torch.save({
                'model': model.state_dict(),
                'classes': classes,
                'window_cells': cells,
                'window_resolution_m': resolution,
                'base_width': args.base_width,
                'depth': args.depth,
                'in_channels': train_set[0][0].shape[0],
            }, checkpoint_path)
            line += '  *'
        print(line)
        schedule.step()

    if epochs:
        units = 'loss' if args.select_on == 'loss' else 'm mode error'
        shown = best if args.select_on == 'loss' else best * resolution
        print(f'\nkept epoch {best_epoch} ({shown:.3f} {units})')
        print(f'trained in {time.time() - started:.0f} s')
    else:
        print(f'\nre-exporting {checkpoint_path} without training')

    # Reload the best weights before exporting, so the ONNX is the model the
    # numbers above describe rather than whatever the last epoch happened to be.
    state = torch.load(checkpoint_path, map_location='cpu')
    model = LandmarkPriorNet(
        in_channels=state['in_channels'], num_classes=len(classes),
        base_width=state['base_width'], depth=state['depth'])
    model.load_state_dict(state['model'])
    model.eval()

    onnx_path = args.out_dir / f'{args.name}.onnx'
    export_options = dict(
        input_names=['channels'],
        output_names=['logits'],
        opset_version=18,
    )
    try:
        # One self-contained file. Left to itself the exporter may split the
        # weights into a sidecar .onnx.data, and a deployment that copies only
        # the .onnx then fails on the robot with a missing-file error that
        # does not mention the sidecar.
        torch.onnx.export(
            model, torch.zeros(1, state['in_channels'], cells, cells),
            str(onnx_path), external_data=False, **export_options)
    except TypeError:
        # Older torch has no external_data argument and never splits.
        torch.onnx.export(
            model, torch.zeros(1, state['in_channels'], cells, cells),
            str(onnx_path), **export_options)

    sidecar = onnx_path.with_suffix('.onnx.data')
    if sidecar.exists():
        raise SystemExit(
            f'the exporter split the weights into {sidecar.name}; the robot '
            'loads a single file, so shrink the model or copy both')

    manifest = {
        'name': args.name,
        'created': time.strftime('%Y-%m-%d %H:%M:%S'),
        'classes': classes,
        'window_cells': cells,
        'window_resolution_m': resolution,
        'best_score': None if not epochs else round(float(best), 4),
        'best_epoch': best_epoch,
        'selection_criterion': args.select_on,
        'model_selected_on': 'held-out runs' if val_loader else 'training set',
        'parameters': model.parameter_count(),
        'train_runs': meta['train_runs'],
        'val_runs': meta['val_runs'],
        'train_samples': len(train_set),
        'val_samples': len(val_set),
        'epochs': args.epochs,
        'augment_flip': args.augment_flip,
    }
    manifest_path = args.out_dir / f'{args.name}.json'
    if epochs or not manifest_path.is_file():
        manifest_path.write_text(
            json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    else:
        print(f'kept the existing {manifest_path.name} from the training run')

    print(f'wrote {onnx_path}   <- copy this to the robot')
    if not meta['val_runs'] and epochs:
        print(
            '\nNOTE: no held-out runs, so the error above is a training '
            'score. Do not read it as accuracy.')


if __name__ == '__main__':
    main()
