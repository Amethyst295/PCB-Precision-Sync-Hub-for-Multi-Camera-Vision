import json, numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
R = json.load(open('ml/c5/ptp_results.json')); A = json.load(open('ml/anomaly_results.json'))
# Fig 7: offset traces under heavy traffic
pi, ai = (np.array(v) for v in R['trace_heavy']); t = (np.arange(len(pi)) + R['warmup']) * 0.125
fig, ax = plt.subplots(figsize=(7.5, 3.2), dpi=200)
ax.plot(t, pi / 1e3, color='#9aa5b1', lw=0.8, label=f'PI servo (lucky-packet), RMSE {np.sqrt(np.mean(pi**2))/1e3:.2f} µs')
ax.plot(t, ai / 1e3, color='#2563eb', lw=0.9, label=f'PI + MLP bias correction, RMSE {np.sqrt(np.mean(ai**2))/1e3:.2f} µs')
ax.axhline(0, color='#1f2933', lw=0.5); ax.set_xlabel('time (s)'); ax.set_ylabel('true clock offset (µs)')
ax.spines[['top', 'right']].set_visible(False); ax.grid(alpha=0.25); ax.legend(frameon=False, fontsize=7.5, loc='upper right')
plt.savefig('fig7_offset.png', bbox_inches='tight', facecolor='white')
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
# Fig 3: VSM logic analyser capture with time axis
im = Image.open('../vsm/vsm_img.png').convert('RGB'); im = im.crop((0, 0, im.width, 700))
fig, ax = plt.subplots(figsize=(7.5, 3.3), dpi=200)
ax.imshow(np.array(im), extent=[1.05, 1.05 + 20 * 6.72, 7.5 + 0.5, -0.5 + 0.0], aspect='auto')
ax.set_yticks(range(8)); ax.set_yticklabels([f'TRIG{i+1}' for i in range(8)], fontsize=7)
ax.set_xlabel('simulated time (ms)  ·  6.72 ms/div, capture resolution 50 µs', fontsize=8); ax.tick_params(axis='x', labelsize=7)
plt.savefig('fig3_vsm.png', bbox_inches='tight', facecolor='white')
