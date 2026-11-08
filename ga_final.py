"""Final GA run + legalisation + exports (CSV for Proteus, figures, JSON metrics)."""
import sys, json, csv, time
sys.path.insert(0, '.')
import numpy as np
import ga_place as G
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

W, H = float(sys.argv[1]), float(sys.argv[2]); SEEDS = [int(s) for s in sys.argv[3].split(',')]; GENS = int(sys.argv[4])

def legalise(B, xy, rot, iters=400):
    """greedy push-apart of overlapping parts (keeps zones), then accept only if cost does not explode."""
    xy = xy.copy()
    for _ in range(iters):
        P = np.vstack([xy, B.fxy]); wh = B.dims(rot) + G.CLR
        moved = False
        for i in range(G.NM):
            for j in range(len(P)):
                if i == j or (j < G.NM and j < i): continue
                d = P[i] - P[j]; ov = (wh[i] + wh[j]) / 2 - np.abs(d)
                if ov[0] > 0 and ov[1] > 0:
                    ax = int(ov[1] < ov[0]); s = np.sign(d[ax]) or 1.0
                    if j < G.NM: xy[i, ax] += s * ov[ax] / 2 + 0.05; xy[j, ax] -= s * ov[ax] / 2 + 0.05
                    else: xy[i, ax] += s * (ov[ax] + 0.05)
                    moved = True
        xy, rot = B.repair(xy, rot)
        if not moved: break
    return xy, rot


def anneal(B, xy, rot, iters=60000, T0=300.0, T1=0.5):
    xy = xy.copy(); rot = rot.copy(); cur = B.metrics(xy, rot)[0]; bestc, bx, br = cur, xy.copy(), rot.copy()
    for it in range(iters):
        fr = it / iters; T = T0 * (T1 / T0) ** fr; sig = 6.0 * (1 - fr) + 0.25
        nx, nr = xy.copy(), rot.copy(); i = G.RNG.integers(G.NM); u = G.RNG.random()
        if u < 0.75: nx[i] += G.RNG.normal(0, sig, 2)
        elif u < 0.88: nr[i] = G.RNG.integers(0, 4)
        else:
            j = G.RNG.integers(G.NM)
            if B.zone[i] != B.zone[j]: continue
            nx[[i, j]] = nx[[j, i]]
        nx, nr = B.repair(nx, nr); c = B.metrics(nx, nr)[0]
        if c < cur or G.RNG.random() < np.exp((cur - c) / T):
            xy, rot, cur = nx, nr, c
            if c < bestc: bestc, bx, br = c, nx.copy(), nr.copy()
    return bx, br

best = None; runs = []
for sd in SEEDS:
    G.RNG = np.random.default_rng(sd)
    B, sol, (cost, m), hist, dt = G.run(W, H, pop=160, gens=GENS)
    xa, ra = anneal(B, *sol); c_sa = B.metrics(xa, ra)[0]
    xy, rot = legalise(B, xa, ra)
    c2, m2 = B.metrics(xy, rot)
    runs.append(dict(seed=sd, cost_ga=cost, cost_sa=c_sa, cost=c2, overlap=m2['overlap'], crit=m2['hpwl_crit'], total=m2['hpwl_total'], time=dt,
                     gen_conv=next(g for g, f, _ in hist if f <= hist[-1][1] * 1.01)))
    print(runs[-1], flush=True)
    if best is None or c2 < best[0]: best = (c2, m2, xy, rot, hist, B, dt, sd)

c2, m, xy, rot, hist, B, dt, sd = best
# random-placement reference (generation-0 statistics) for context
G.RNG = np.random.default_rng(123)
rnd = [B.metrics(*B.random())[1] for _ in range(300)]
ref = dict(crit_median=float(np.median([r['hpwl_crit'] for r in rnd])), total_median=float(np.median([r['hpwl_total'] for r in rnd])))

# ---- exports ----
wh = B.dims(rot)
rows = []
for i, k in enumerate(G.MOV):
    rows.append([k, G.PARTS[k][4], round(xy[i, 0], 2), round(xy[i, 1], 2), int(rot[i]) * 90, round(wh[i, 0], 1), round(wh[i, 1], 1),
                 {'N': 'board side', 'I': 'isolated side', 'S': 'straddles barrier'}[G.PARTS[k][2]]])
for j, (k, v) in enumerate(B.fixed.items()):
    rows.append([k, G.FIXED_DESC[k], round(v[0], 2), round(v[1], 2), 0, v[2], v[3], 'fixed (edge/connector)'])
with open('placement_GA.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['unit', 'contents', 'x_mm', 'y_mm', 'rot_deg', 'w_mm', 'h_mm', 'zone'])
    w.writerows(rows)

def P(name):
    if name in G.IDX: return xy[G.IDX[name]]
    return np.array(B.fixed[name][:2])
man = lambda a, b: float(np.abs(P(a) - P(b)).sum())
phys = dict(board=f'{W:.0f} x {H:.0f}', area=W * H,
            tcxo_to_mcu=man('Y1_TCXO', 'U_CLKBUF') + man('U_CLKBUF', 'U1_MCU'),
            buf_to_phy=man('U_CLKBUF', 'U3_PHY'), tcxo_to_buf=man('Y1_TCXO', 'U_CLKBUF'),
            rmii_total=9 * man('U1_MCU', 'U3_PHY'), rmii_each=man('U1_MCU', 'U3_PHY'),
            eth_pairs=man('U3_PHY', 'J3_RJ45'),
            trig_mismatch=m['trig_mismatch'], trig_len=m['trig_len'],
            reg_to_tcxo={n: man(n, 'Y1_TCXO') for n in G.NOISY}, noise_min=m['noise_min_dist'],
            sw_loop=m['hpwl_sw'])
json.dump(dict(runs=runs, best_seed=sd, metrics={k: v for k, v in m.items()}, phys=phys, ref_random=ref,
               hist=[(g, float(f), float(md)) for g, f, md in hist], time=dt, W=W, H=H,
               ga=dict(pop=160, gens=GENS, tournament=3, elite=6, mut=0.12, rot_mut=0.03, swap=0.15)),
          open('ga_result.json', 'w'), indent=1, default=float)

# ---- figure: placement ----
fig, ax = plt.subplots(figsize=(8, 8 * H / W), dpi=200)
ax.add_patch(Rectangle((0, 0), W, H, fc='#fbfbfa', ec='#1f2933', lw=1.4))
ax.add_patch(Rectangle((B.XB + G.GAP, 0), W - B.XB - G.GAP, H, fc='#e8f1ff', ec='none'))
ax.add_patch(Rectangle((B.XB - G.GAP, 0), 2 * G.GAP, H, fc='none', ec='#2563eb', ls='--', lw=0.8, hatch='////', alpha=0.35))
col = {'N': '#cbd2d9', 'I': '#93b4f5', 'S': '#f5c26b'}
for i, k in enumerate(G.MOV):
    w_, h_ = wh[i]; x, y = xy[i]
    ax.add_patch(Rectangle((x - w_ / 2, y - h_ / 2), w_, h_, fc=col[G.PARTS[k][2]], ec='#3e4c59', lw=0.6))
    ax.text(x, y, k.replace('_', '\n', 1), fontsize=4.6, ha='center', va='center', color='#1f2933')
for k, v in B.fixed.items():
    ax.add_patch(Rectangle((v[0] - v[2] / 2, v[1] - v[3] / 2), v[2], v[3], fc='#7b8794', ec='#1f2933', lw=0.6))
    ax.text(v[0], v[1], k.split('_')[-1], fontsize=4.6, ha='center', va='center', color='white')
for w_, c, ms in G.NETS:
    if c in ('clk', 'rmii', 'eth') and len(ms) == 2:
        a, b = P(ms[0]), P(ms[1])
        ax.plot([a[0], b[0]], [a[1], b[1]], color={'clk': '#d64545', 'rmii': '#2563eb', 'eth': '#2f9e44'}[c], lw=0.9, alpha=0.6)
ax.set_xlim(-2, W + 2); ax.set_ylim(-2, H + 2); ax.set_aspect('equal')
ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')
ax.plot([], [], color='#d64545', label='clock'); ax.plot([], [], color='#2563eb', label='RMII'); ax.plot([], [], color='#2f9e44', label='Ethernet pairs')
ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=3, fontsize=7, frameon=False)
plt.savefig('fig9_ga_placement.png', bbox_inches='tight', facecolor='white')

# ---- figure: convergence ----
fig, ax = plt.subplots(figsize=(7, 3.4), dpi=200)
g = [h[0] for h in hist]
ax.plot(g, [h[1] for h in hist], color='#2563eb', lw=1.6, label='best individual')
ax.plot(g, [h[2] for h in hist], color='#9aa5b1', lw=1.0, label='population median')
ax.set_yscale('log'); ax.set_xlabel('generation'); ax.set_ylabel('weighted cost (log)')
ax.spines[['top', 'right']].set_visible(False); ax.grid(alpha=0.25); ax.legend(frameon=False, fontsize=8)
plt.savefig('fig6_ga_convergence.png', bbox_inches='tight', facecolor='white')
print('best', sd, 'cost', c2, 'overlap', m['overlap'], phys, ref)
