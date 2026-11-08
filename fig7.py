"""Figure 7: PI servo vs PI + MLP on unseen heavy-traffic sequences (run from the repository root)."""
import sys, re, json, numpy as np
sys.path.insert(0, '.'); import ptp_sim as P
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
txt = open('firmware/STM32F407VE_1/mlp_weights.h').read()
def arr(n):
    m = re.search(n + r'\[[^\]]*\](?:\[[^\]]*\])? = \{(.*?)\};', txt, re.S); return np.array([float(x.rstrip('f')) for x in re.findall(r'[-0-9.e+]+f', m.group(1))])
mlp = P.MLP(dict(mu=arr('MLP_MU'), sd=arr('MLP_SD'), W1=arr('MLP_W1').reshape(13, 16), b1=arr('MLP_B1'), W2=arr('MLP_W2').reshape(16, 8), b2=arr('MLP_B2'), W3=arr('MLP_W3').reshape(8, 1), b3=arr('MLP_B3')))
best = None
for sd in range(100, 112):                          # unseen heavy sequences; show the median-improvement one
    rng = np.random.default_rng(sd); s = P.make_sequence(rng, 'heavy')
    np.random.seed(sd); a = P.run_servo(s, None); np.random.seed(sd); b = P.run_servo(s, mlp)
    ra, rb = [np.sqrt(np.mean(x['theta'][P.WARMUP:] ** 2)) for x in (a, b)]
    best = (best or []) + [(1 - rb / ra, sd, a, b, ra, rb)]
best.sort(key=lambda z: z[0]); imp, sd, a, b, ra, rb = best[len(best) // 2]
print('median improvement', imp, [round(z[0] * 100, 1) for z in best])
t = np.arange(P.N_SEQ_STEPS) * P.TS; k = slice(P.WARMUP, None)
fig, ax = plt.subplots(2, 1, figsize=(7.5, 5), dpi=200, sharex=True)
ax[0].plot(t[k], a['theta'][k] / 1e3, color='#9aa5b1', lw=0.8, label=f'PI servo, RMSE {ra/1e3:.2f} µs')
ax[0].plot(t[k], b['theta'][k] / 1e3, color='#2563eb', lw=0.9, label=f'PI + MLP, RMSE {rb/1e3:.2f} µs')
ax[0].set_ylabel('true offset (µs)'); ax[0].legend(frameon=False, fontsize=7.5)
ax[1].plot(t[k], b['true_bias'][k] / 1e3, color='#9aa5b1', lw=0.7, label='true bias of the lucky-packet filter')
ax[1].plot(t[k], b['bias_used'][k] / 1e3, color='#d64545', lw=1.0, label='correction applied (EMA, gated ±0.5 µs, clipped ±5 µs)')
ax[1].set_ylabel('bias (µs)'); ax[1].set_xlabel('time (s)'); ax[1].legend(frameon=False, fontsize=7.5)
for x in ax: x.spines[['top', 'right']].set_visible(False); x.grid(alpha=0.25)
plt.savefig('fig7_offset.png', bbox_inches='tight', facecolor='white')
json.dump(dict(seq_seed=sd, rmse_pi=ra, rmse_ai=rb, imp=imp, all=[z[0] for z in best]), open('fig7_meta.json', 'w'))
