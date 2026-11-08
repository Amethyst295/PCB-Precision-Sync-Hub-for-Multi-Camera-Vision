"""Figures 3 (VSM capture) and 8 (confusion matrix). Run from the repository root."""
import json, numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
R = json.load(open('ptp_results.json')); A = json.load(open('anomaly_results.json'))
# Fig 8: normalised confusion matrix
names = ['Normal', 'Disconnected', 'Intermittent', 'Lat. drift', 'Double trig.', 'PTP loss', 'Congestion', 'Osc. fault']
cm = np.array(A['cm_tree'], float); cmn = cm / cm.sum(1, keepdims=True)
fig, ax = plt.subplots(figsize=(6, 5), dpi=200)
ax.imshow(cmn, cmap='Blues', vmin=0, vmax=1)
for i in range(8):
    for j in range(8):
        if cm[i, j] > 0: ax.text(j, i, f'{cmn[i, j]*100:.0f}', ha='center', va='center', fontsize=7, color='white' if cmn[i, j] > 0.6 else '#1f2933')
ax.set_xticks(range(8)); ax.set_yticks(range(8)); ax.set_xticklabels(names, rotation=40, ha='right', fontsize=7); ax.set_yticklabels(names, fontsize=7)
ax.set_xlabel('predicted', fontsize=8); ax.set_ylabel('true', fontsize=8)
plt.savefig('fig8_confusion.png', bbox_inches='tight', facecolor='white')
# Fig 3: VSM logic analyser capture with time axis (needs docs/vsm_capture.png, exported from Proteus)
import os
if os.path.exists('docs/vsm_capture.png'):
    im = Image.open('docs/vsm_capture.png').convert('RGB'); im = im.crop((0, 0, im.width, 700))
    fig, ax = plt.subplots(figsize=(7.5, 3.3), dpi=200)
    ax.imshow(np.array(im), extent=[1.05, 1.05 + 20 * 6.72, 7.5 + 0.5, -0.5 + 0.0], aspect='auto')
    ax.set_yticks(range(8)); ax.set_yticklabels([f'TRIG{i+1}' for i in range(8)], fontsize=7)
    ax.set_xlabel('simulated time (ms)  ·  6.72 ms/div, capture resolution 50 µs', fontsize=8); ax.tick_params(axis='x', labelsize=7)
    plt.savefig('fig3_vsm.png', bbox_inches='tight', facecolor='white')
