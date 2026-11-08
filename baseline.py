"""Conventional (unoptimised) block placement on the initial 90 x 90 mm outline, evaluated with the same
cost model as the GA, plus the side-by-side before/after figure."""
import sys, json, csv
sys.path.insert(0, '.')
import numpy as np
import ga_place as G
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# functional blocks laid out following the schematic flow: power along the bottom, Ethernet + MCU on the left,
# clock beside the MCU, isolators on the barrier, camera-side parts next to the connectors.
CONV = {
 'U3_PHY': (30, 80, 0), 'U1_MCU': (30, 54, 0), 'Y1_TCXO': (11, 54, 0), 'U_CLKBUF': (11, 46, 0),
 'FU1_PTC': (16, 6, 0), 'D_REV': (24, 6, 0), 'D_TVS': (32, 6, 0), 'C_BULK': (42, 7, 0),
 'U_BUCK': (16, 18, 0), 'L1': (26, 18, 0), 'D_CATCH': (35, 18, 0), 'C_OUT5': (43, 18, 0),
 'U_LDO': (12, 31, 0), 'FB1_A': (24, 31, 0), 'FB2_CLK': (31, 31, 0),
 'R_TRIG_MA': (49, 60, 1), 'R_TRIG_MB': (49, 48, 1),
 'J_SWD': (34, 88, 0), 'J_UART': (49, 38, 1), 'J_PPS': (49, 28, 1), 'LEDS': (49, 75, 1),
 'U_ISO_T1': (57, 81, 0), 'U_ISO_T2': (57, 67, 0), 'U_ISO_S1': (57, 41, 0), 'U_ISO_S2': (57, 27, 0), 'U_RFM': (57, 10, 0),
 'U_DRV': (68, 66, 1), 'U_TSSOP': (68, 52, 0),
 'R_TRIG_IA': (66, 85, 0), 'R_TRIG_IB': (75, 85, 0), 'D_ESD_T1': (82, 85, 0), 'D_ESD_T2': (87, 85, 0),
 'R_STB_IA': (67, 41, 1), 'R_STB_IB': (67, 30, 1), 'D_ESD_S1': (66, 19, 0), 'D_ESD_S2': (72, 19, 0),
}

def legalise(B, xy, rot, iters=400):
    xy = xy.copy()
    for _ in range(iters):
        P = np.vstack([xy, B.fxy]); wh = B.dims(rot) + G.CLR; moved = False
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

W, H = 90.0, 90.0
B = G.Board(W, H)
xy = np.array([CONV[k][:2] for k in G.MOV], float); rot = np.array([CONV[k][2] for k in G.MOV])
xy, rot = B.repair(xy, rot)
xy, rot = legalise(B, xy, rot)   # nudge parts until clearance-clean (as a designer would)
G.CLR = 0.0; phys_ov = B.metrics(xy, rot)[1]['overlap']; G.CLR = 1.2
cost, m = B.metrics(xy, rot)
P = lambda n: xy[G.IDX[n]] if n in G.IDX else np.array(B.fixed[n][:2])
man = lambda a, b: float(np.abs(P(a) - P(b)).sum())
phys = dict(board=f'{W:.0f} x {H:.0f}', area=W * H, tcxo_to_mcu=man('Y1_TCXO', 'U_CLKBUF') + man('U_CLKBUF', 'U1_MCU'),
            buf_to_phy=man('U_CLKBUF', 'U3_PHY'), rmii_total=9 * man('U1_MCU', 'U3_PHY'), rmii_each=man('U1_MCU', 'U3_PHY'),
            trig_mismatch=m['trig_mismatch'], reg_to_tcxo={n: man(n, 'Y1_TCXO') for n in G.NOISY}, noise_min=m['noise_min_dist'])
json.dump(dict(cost=cost, metrics=m, phys=phys, physical_overlap=phys_ov), open('baseline_result.json', 'w'), indent=1, default=float)
with open('placement_before.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['unit', 'x_mm', 'y_mm', 'rot_deg'])
    for i, k in enumerate(G.MOV): w.writerow([k, round(xy[i, 0], 2), round(xy[i, 1], 2), int(rot[i]) * 90])
print('cost', round(cost), 'overlap(clr)', round(m['overlap'], 2), 'physical', phys_ov, 'crit', round(m['hpwl_crit']), 'total', round(m['hpwl_total']), phys)

# ---- side-by-side figure (same scale) ----
A = json.load(open('ga_result.json')); Ba = G.Board(A['W'], A['H'])
rows = list(csv.DictReader(open('placement_GA.csv')))
xa = np.array([[float(r['x_mm']), float(r['y_mm'])] for r in rows[:G.NM]]); ra = np.array([int(r['rot_deg']) // 90 for r in rows[:G.NM]])
fig, axs = plt.subplots(1, 2, figsize=(12, 6.2), dpi=200)
col = {'N': '#cbd2d9', 'I': '#93b4f5', 'S': '#f5c26b'}
for ax, (BB, X, R, title) in zip(axs, [(B, xy, rot, f'Before: conventional block placement, {W:.0f} × {H:.0f} mm'),
                                       (Ba, xa, ra, f'After: AI-seeded placement, {A["W"]:.0f} × {A["H"]:.0f} mm')]):
    wh = BB.dims(R)
    ax.add_patch(Rectangle((0, 0), BB.W, BB.H, fc='#fbfbfa', ec='#1f2933', lw=1.2))
    ax.add_patch(Rectangle((BB.XB + G.GAP, 0), BB.W - BB.XB - G.GAP, BB.H, fc='#e8f1ff', ec='none'))
    for i, k in enumerate(G.MOV):
        w_, h_ = wh[i]; x, y = X[i]
        ax.add_patch(Rectangle((x - w_ / 2, y - h_ / 2), w_, h_, fc=col[G.PARTS[k][2]], ec='#3e4c59', lw=0.5))
        ax.text(x, y, k.split('_', 1)[1] if k.startswith(('U_', 'D_', 'R_', 'C_', 'J_')) else k.split('_')[0], fontsize=3.8, ha='center', va='center')
    for k, v in BB.fixed.items():
        ax.add_patch(Rectangle((v[0] - v[2] / 2, v[1] - v[3] / 2), v[2], v[3], fc='#7b8794', ec='#1f2933', lw=0.5))
    PP = lambda n: X[G.IDX[n]] if n in G.IDX else np.array(BB.fixed[n][:2])
    for w_, c, ms in G.NETS:
        if c in ('clk', 'rmii', 'eth') and len(ms) == 2:
            a, b = PP(ms[0]), PP(ms[1]); ax.plot([a[0], b[0]], [a[1], b[1]], color={'clk': '#d64545', 'rmii': '#2563eb', 'eth': '#2f9e44'}[c], lw=0.9, alpha=0.6)
    for n in G.NOISY:
        a, b = PP(n), PP('Y1_TCXO'); ax.plot([a[0], b[0]], [a[1], b[1]], color='#b45309', lw=0.5, ls=':', alpha=0.7)
    ax.set_xlim(-2, 92); ax.set_ylim(-2, 92); ax.set_aspect('equal'); ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=7); ax.set_xlabel('x (mm)', fontsize=7)
axs[0].plot([], [], color='#d64545', label='clock'); axs[0].plot([], [], color='#2563eb', label='RMII'); axs[0].plot([], [], color='#2f9e44', label='Ethernet')
axs[0].plot([], [], color='#b45309', ls=':', label='regulator → TCXO')
fig.legend(loc='lower center', ncol=4, fontsize=7, frameon=False, bbox_to_anchor=(0.5, -0.01))
plt.tight_layout(rect=(0, 0.03, 1, 1)); plt.savefig('fig9_before_after.png', bbox_inches='tight', facecolor='white')
