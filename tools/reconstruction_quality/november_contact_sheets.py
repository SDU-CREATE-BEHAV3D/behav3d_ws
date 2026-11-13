#!/usr/bin/env python3
"""Render every raw RGB capture and the corresponding unmodified depth frame."""
import argparse
from pathlib import Path
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=True, parents=True)
    for manifest in sorted(args.data.glob('*/*/*/scan_yaml_targets/manifest.yaml')):
        scan = manifest.parent
        captures = yaml.safe_load(manifest.read_text())['captures']
        rows = (len(captures) + 4) // 5
        fig, axes = plt.subplots(rows, 5, figsize=(15, rows * 2.0), squeeze=False)
        for ax in axes.flat:
            ax.axis('off')
        for c, ax in zip(captures, axes.flat):
            image = cv2.imread(str(scan / c['color']))
            ax.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            ax.set_title(f"{c['index']} · {c['timestamp'].split('T')[-1][:8]}", fontsize=9)
        fig.suptitle(manifest.parents[1].name + ' · all raw color captures')
        fig.tight_layout()
        fig.savefig(args.output / (manifest.parents[1].name + '.png'), dpi=130)
        plt.close(fig)
        fig, axes = plt.subplots(rows, 5, figsize=(15, rows * 2.5), squeeze=False)
        for ax in axes.flat:
            ax.axis('off')
        for c, ax in zip(captures, axes.flat):
            dep = cv2.imread(str(scan / c['depth']), cv2.IMREAD_UNCHANGED)
            ax.imshow(np.ma.masked_equal(dep, 0), vmin=300, vmax=900, cmap='viridis')
            ax.set_title(f"{c['index']} · {100 * (dep == 0).mean():.1f}% zero depth", fontsize=9)
        fig.suptitle(manifest.parents[1].name + ' · raw depth, fixed scale 300–900 mm; white = no return')
        fig.tight_layout()
        fig.savefig(args.output / (manifest.parents[1].name + '_depth.png'), dpi=100)
        plt.close(fig)
