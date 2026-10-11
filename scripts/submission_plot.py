"""Submission figure: estimated vs ground-truth trajectory, plus position error over time.

Usage: python3 submission_plot.py EXPERIMENT_DIR OUT_PREFIX
Writes OUT_PREFIX.png and OUT_PREFIX.pdf.
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

exp_dir, out = sys.argv[1], sys.argv[2]
cases = [
    ('baseline', 'Estimate: ideal encoders', '#1f77b4', '-'),
    ('noise_005', 'Estimate: random noise (k = 0.05)', '#d62728', '--'),
    ('separation_plus5pct', 'Estimate: wheel separation +5%', '#2ca02c', '-.'),
]
data = {name: np.genfromtxt(os.path.join(exp_dir, name + '.csv'), delimiter=',', names=True)
        for name, *_ in cases}

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={'width_ratios': [1, 1.25]})

gt = data['baseline']
ax1.plot(gt['gt_x'], gt['gt_y'], 'k-', lw=3, label='Ground truth (Gazebo)')
for name, label, color, style in cases:
    d = data[name]
    ax1.plot(d['est_x'], d['est_y'], style, color=color, lw=1.8, label=label)
ax1.plot(0, 0, 'ko', ms=7)
ax1.annotate('Start / end', (0, 0), (0.12, 0.08), fontsize=9)
ax1.set_xlabel('x [m]')
ax1.set_ylabel('y [m]')
ax1.set_title('(a) Estimated vs ground-truth trajectory (1 m square)')
ax1.axis('equal')
ax1.grid(alpha=0.3)
ax1.legend(fontsize=8.5, loc='center')

for name, label, color, style in cases:
    d = data[name]
    t = d['stamp'] - d['stamp'][0]
    err = np.hypot(d['est_x'] - d['gt_x'], d['est_y'] - d['gt_y'])
    ax2.plot(t, err, style, color=color, lw=1.8, label=label.replace('Estimate: ', ''))
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Position error [m]')
ax2.set_title('(b) Position error relative to ground truth')
ax2.grid(alpha=0.3)
ax2.legend(fontsize=8.5, loc='upper left')

fig.tight_layout()
fig.savefig(out + '.png', dpi=200)
fig.savefig(out + '.pdf')
print('saved', out + '.png', out + '.pdf')
