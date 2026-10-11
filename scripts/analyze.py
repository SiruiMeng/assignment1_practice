"""Compare odometry estimates against ground truth for one or more logger CSV files.

Usage:
    python3 analyze.py baseline.csv noise_001.csv noise_005.csv --out results
Writes <out>/summary.csv, <out>/trajectories.png and <out>/errors.png.
Ground truth is only used here, for evaluation.
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    d = np.genfromtxt(path, delimiter=',', names=True)
    if d.size < 2:
        raise SystemExit(f'{path}: not enough rows')
    return d


def metrics(d):
    t = d['stamp'] - d['stamp'][0]
    pos_err = np.hypot(d['est_x'] - d['gt_x'], d['est_y'] - d['gt_y'])
    dth = d['est_theta'] - d['gt_theta']
    yaw_err = np.degrees(np.abs(np.arctan2(np.sin(dth), np.cos(dth))))
    path_len = np.sum(np.hypot(np.diff(d['gt_x']), np.diff(d['gt_y'])))
    return {
        't': t, 'pos_err': pos_err, 'yaw_err': yaw_err,
        'pos_rmse': float(np.sqrt(np.mean(pos_err ** 2))),
        'pos_max': float(np.max(pos_err)),
        'pos_final': float(pos_err[-1]),
        'yaw_rmse': float(np.sqrt(np.mean(yaw_err ** 2))),
        'yaw_final': float(yaw_err[-1]),
        'path_len': float(path_len),
        'drift_pct': float(100.0 * pos_err[-1] / path_len) if path_len > 0 else float('nan'),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--out', default='results')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    runs = []
    for path in args.csv:
        name = os.path.splitext(os.path.basename(path))[0]
        d = load(path)
        runs.append((name, d, metrics(d)))

    keys = ['pos_rmse', 'pos_max', 'pos_final', 'yaw_rmse', 'yaw_final', 'path_len', 'drift_pct']
    with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run'] + keys)
        for name, _, m in runs:
            w.writerow([name] + [f'{m[k]:.4f}' for k in keys])

    print(f'{"run":20s} {"pos RMSE[m]":>11s} {"final[m]":>9s} {"yaw RMSE[deg]":>13s} '
          f'{"final[deg]":>10s} {"path[m]":>8s} {"drift[%]":>8s}')
    for name, _, m in runs:
        print(f'{name:20s} {m["pos_rmse"]:11.3f} {m["pos_final"]:9.3f} {m["yaw_rmse"]:13.2f} '
              f'{m["yaw_final"]:10.2f} {m["path_len"]:8.2f} {m["drift_pct"]:8.2f}')

    # 轨迹图：真值只画第一组（同一条路线）
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(runs[0][1]['gt_x'], runs[0][1]['gt_y'], 'k-', lw=2.5, label='Ground truth')
    for name, d, _ in runs:
        ax.plot(d['est_x'], d['est_y'], '--', lw=1.3, label=name)
    ax.set(xlabel='x [m]', ylabel='y [m]', title='Estimated vs ground-truth trajectory')
    ax.axis('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'trajectories.png'), dpi=150)

    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for name, _, m in runs:
        ax[0].plot(m['t'], m['pos_err'], label=name)
        ax[1].plot(m['t'], m['yaw_err'], label=name)
    ax[0].set(ylabel='position error [m]', title='Error over time')
    ax[1].set(xlabel='time [s]', ylabel='|heading error| [deg]')
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'errors.png'), dpi=150)
    print(f'Saved figures and summary to {args.out}/')


if __name__ == '__main__':
    main()
