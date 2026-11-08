"""Genetic-algorithm component placement for the Precision Sync Hub (AI-SED 2026, Borrenautas).

Placement units are the ICs / connectors / power parts of the finished Proteus schematic; small passives
are folded into the courtyard of the IC they serve (decoupling, series resistors, pull-ups).
Coordinates: mm, origin = bottom-left corner of the board, rotation in degrees (CCW).
Zones: non-isolated side x < XB - GAP, isolated side x > XB + GAP; isolators (ISO7740, RFM-0505S) straddle XB.
"""
import numpy as np, json, time, sys, csv

RNG = np.random.default_rng(int(sys.argv[1]) if len(sys.argv) > 1 else 7)
GAP = 2.0          # half keep-out on each side of the isolation barrier (4 mm copper-free gap)
CLR = 1.2          # courtyard clearance kept around every part for routing (mm)

# name: (w, h, zone, kind, desc)   zone: 'N' non-iso, 'I' iso, 'S' straddles barrier, 'F' fixed
PARTS = {
 'U1_MCU':   (20, 20, 'N', 'mov', 'STM32F407VET6 LQFP-100 + decoupling, VCAP'),
 'U3_PHY':   (11, 10, 'N', 'mov', 'LAN8742A QFN-24 + 49.9R terminations, RBIAS, RMII 33R'),
 'Y1_TCXO':  (5.5, 4.5, 'N', 'mov', 'ECS-TXO-2016-33-250 + 10R/100nF/10uF filter'),
 'U_CLKBUF': (4.5, 4.5, 'N', 'mov', 'SN74LVC2G34 + 2x33R source series'),
 'FU1_PTC':  (5.5, 4.0, 'N', 'mov', 'PTC 1812 0.5 A'),
 'D_TVS':    (6.0, 4.2, 'N', 'mov', 'SMBJ33A (SMB)'),
 'D_REV':    (5.6, 3.0, 'N', 'mov', 'Schottky 60 V 1 A (SMA) reverse-polarity'),
 'C_BULK':   (7.0, 7.0, 'N', 'mov', 'EEE-FK1J330P 33 uF 63 V + 2.2 uF 100 V'),
 'U_BUCK':   (6.5, 5.5, 'N', 'mov', 'LMR16006X + FB 56k/10k + CB'),
 'L1':       (6.6, 6.6, 'N', 'mov', 'Bourns 1255AY-220M 22 uH'),
 'D_CATCH':  (5.6, 3.0, 'N', 'mov', 'Schottky catch diode (SMA)'),
 'C_OUT5':   (5.0, 4.0, 'N', 'mov', '2x 22 uF 1206 (5 V output)'),
 'U_LDO':    (8.0, 8.0, 'N', 'mov', 'TLV1117LV33 SOT-223 + 10 uF'),
 'FB1_A':    (3.5, 2.5, 'N', 'mov', 'BLM18KG601 -> 3V3A (PHY/VDDA)'),
 'FB2_CLK':  (3.5, 2.5, 'N', 'mov', 'BLM18KG601 -> 3V3_CLK (TCXO)'),
 'R_TRIG_MA':(6.0, 2.5, 'N', 'mov', '4x 33R MCU-side trigger series (ch1-4)'),
 'R_TRIG_MB':(6.0, 2.5, 'N', 'mov', '4x 33R MCU-side trigger series (ch5-8)'),
 'J_SWD':    (15.5, 3.0, 'N', 'edge', 'TSW-106 SWD/SWO/NRST header'),
 'J_UART':   (10.5, 3.0, 'N', 'edge', 'TSW-104 UART header'),
 'J_PPS':    (8.0, 3.0, 'N', 'edge', 'TSW-103 PPS in/out + BAT54S clamp'),
 'LEDS':     (9.0, 3.0, 'N', 'edge', '4 status LEDs + resistors'),
 'U_ISO_T1': (11, 12, 'S', 'mov', 'ISO7740F trigger ch1-4'),
 'U_ISO_T2': (11, 12, 'S', 'mov', 'ISO7740F trigger ch5-8'),
 'U_ISO_S1': (11, 12, 'S', 'mov', 'ISO7740F strobe ch1-4'),
 'U_ISO_S2': (11, 12, 'S', 'mov', 'ISO7740F strobe ch5-8'),
 'U_RFM':    (12.5, 6.5, 'S', 'mov', 'RFM-0505S isolated 5 V/1 W + 270R min load'),
 'U_DRV':    (13.5, 11.5, 'I', 'mov', '74AC541 SO-20W line driver (5V_ISO)'),
 'U_TSSOP':  (7.0, 7.0, 'I', 'mov', 'TSSOP-20 (TRIG group)'),
 'R_TRIG_IA':(6.0, 4.5, 'I', 'mov', '4x 82R + 4x 10k (ch1-4)'),
 'R_TRIG_IB':(6.0, 4.5, 'I', 'mov', '4x 82R + 4x 10k (ch5-8)'),
 'R_STB_IA': (6.0, 4.5, 'I', 'mov', '4x 100R + 4x 4.7k (ch1-4)'),
 'R_STB_IB': (6.0, 4.5, 'I', 'mov', '4x 100R + 4x 4.7k (ch5-8)'),
 'D_ESD_T1': (3.5, 3.5, 'I', 'mov', 'SRV05-4 trigger ch1-4'),
 'D_ESD_T2': (3.5, 3.5, 'I', 'mov', 'SRV05-4 trigger ch5-8'),
 'D_ESD_S1': (3.5, 3.5, 'I', 'mov', 'SRV05-4 strobe ch1-4'),
 'D_ESD_S2': (3.5, 3.5, 'I', 'mov', 'SRV05-4 strobe ch5-8'),
}
FIXED_DESC = {'J3_RJ45': 'HanRun HR911105A RJ45 (left edge)', 'J_PWR': 'Phoenix 1729128 terminal block (left edge)'}
for i in range(8): FIXED_DESC[f'J{10+i}_CAM{i+1}'] = f'JST B4B-XH-A camera {i+1} (trig/strobe/5V_ISO/GND_ISO)'

def fixed_parts(W, H):
    f = {'J3_RJ45': (10.75, H - 13.0, 21.5, 16.0), 'J_PWR': (5.0, 7.0, 9.8, 10.2)}
    y0 = H / 2 + 1.5 * 13.6
    for i in range(8):
        col, row = divmod(i, 4)                      # cameras 1-4 outer column, 5-8 inner column
        f[f'J{10+i}_CAM{i+1}'] = (W - 3.6 - col * 7.6, y0 - row * 13.6, 6.5, 12.9)
    return f

# nets: (weight, category, [members])  -- members may include fixed parts
NETS = []
def net(w, cat, *m): NETS.append((w, cat, list(m)))
for _ in range(3): net(4, 'clk', 'Y1_TCXO', 'U_CLKBUF')
for _ in range(3): net(4, 'clk', 'U_CLKBUF', 'U1_MCU')         # HSE_IN
for _ in range(3): net(4, 'clk', 'U_CLKBUF', 'U3_PHY')         # PHY CLKIN
for _ in range(9): net(3, 'rmii', 'U1_MCU', 'U3_PHY')          # REF_CLK, CRS_DV, RXD0/1, TX_EN, TXD0/1, MDC, MDIO
for _ in range(4): net(4, 'eth', 'U3_PHY', 'J3_RJ45')          # TX+/-, RX+/-
TRIG = {}
for ch in range(8):
    a = 'A' if ch < 4 else 'B'; n = 1 if ch < 4 else 2
    path = ['U1_MCU', f'R_TRIG_M{a}', f'U_ISO_T{n}', 'U_DRV', f'R_TRIG_I{a}', f'D_ESD_T{n}', f'J{10+ch}_CAM{ch+1}']
    TRIG[ch] = path
    for p, q in zip(path, path[1:]): net(1.5, 'trig', p, q)
    spath = [f'J{10+ch}_CAM{ch+1}', f'D_ESD_S{n}', f'R_STB_I{a}', f'U_ISO_S{n}', 'U1_MCU']
    for p, q in zip(spath, spath[1:]): net(1.0, 'stb', p, q)
net(2, 'pwr', 'J_PWR', 'FU1_PTC'); net(2, 'pwr', 'FU1_PTC', 'D_REV'); net(2, 'pwr', 'D_REV', 'D_TVS', 'C_BULK', 'U_BUCK')
net(6, 'sw', 'U_BUCK', 'L1', 'D_CATCH')                         # hot switching loop: keep tiny
net(3, 'pwr', 'L1', 'C_OUT5', 'U_BUCK')                         # 5 V + feedback
net(2, 'pwr', 'C_OUT5', 'U_LDO', 'U_RFM')                       # 5 V distribution
net(2, 'pwr', 'U_LDO', 'U1_MCU', 'FB1_A', 'FB2_CLK', 'U_ISO_T1', 'U_ISO_T2', 'U_ISO_S1', 'U_ISO_S2', 'U_CLKBUF', 'LEDS')
net(2, 'pwr', 'FB1_A', 'U3_PHY'); net(1, 'pwr', 'FB1_A', 'U1_MCU')
net(3, 'pwr', 'FB2_CLK', 'Y1_TCXO')
net(2, 'pwr', 'U_RFM', 'U_DRV', 'U_TSSOP', 'R_STB_IA', 'R_STB_IB', 'U_ISO_T1', 'U_ISO_T2', 'U_ISO_S1', 'U_ISO_S2')
net(1, 'trig', 'U_DRV', 'U_TSSOP')
net(1, 'io', 'U1_MCU', 'J_SWD'); net(1, 'io', 'U1_MCU', 'J_UART'); net(1, 'io', 'U1_MCU', 'J_PPS'); net(1, 'io', 'U1_MCU', 'LEDS')
NOISY = ['U_BUCK', 'L1', 'D_CATCH', 'U_LDO', 'U_RFM']
CRIT = ('clk', 'rmii', 'eth')

MOV = [k for k in PARTS]
IDX = {k: i for i, k in enumerate(MOV)}
NM = len(MOV)

class Board:
    def __init__(self, W, H):
        self.W, self.H, self.XB = W, H, W - 33.0
        self.fixed = fixed_parts(W, H)
        self.names = MOV + list(self.fixed)
        self.wh0 = np.array([PARTS[k][:2] for k in MOV] + [v[2:] for v in self.fixed.values()], float)
        self.zone = [PARTS[k][2] for k in MOV]
        self.fxy = np.array([v[:2] for v in self.fixed.values()], float)
        self.nidx = [(w, c, [self.names.index(m) for m in ms]) for w, c, ms in NETS]

    def dims(self, rot):
        wh = self.wh0.copy()
        r = np.concatenate([rot, np.zeros(len(self.fixed), int)])
        sw = (r % 2) == 1
        wh[sw] = wh[sw][:, ::-1]
        return wh

    def repair(self, xy, rot):
        """clip every movable part into its legal zone (board + isolation side)."""
        wh = self.dims(rot)[:NM]
        for i, z in enumerate(self.zone):
            hw, hh = wh[i] / 2
            lo_x, hi_x = hw + 0.5, self.W - hw - 0.5
            if z == 'N': hi_x = min(hi_x, self.XB - GAP - hw)
            if z == 'I': lo_x = max(lo_x, self.XB + GAP + hw)
            if z == 'S': lo_x = hi_x = self.XB; rot[i] = 0
            xy[i, 0] = np.clip(xy[i, 0], lo_x, hi_x)
            xy[i, 1] = np.clip(xy[i, 1], hh + 0.5, self.H - hh - 0.5)
        return xy, rot

    def random(self):
        xy = np.column_stack([RNG.uniform(0, self.W, NM), RNG.uniform(0, self.H, NM)])
        rot = RNG.integers(0, 4, NM)
        return self.repair(xy, rot)

    def metrics(self, xy, rot):
        P = np.vstack([xy, self.fxy]); wh = self.dims(rot)
        hp = {'clk': 0, 'rmii': 0, 'eth': 0, 'trig': 0, 'stb': 0, 'pwr': 0, 'sw': 0, 'io': 0}
        tot = 0.0
        for w, c, ms in self.nidx:
            q = P[ms]; l = (q[:, 0].max() - q[:, 0].min()) + (q[:, 1].max() - q[:, 1].min())
            hp[c] += l; tot += w * l
        # overlap (with routing clearance)
        lo = P - wh / 2 - CLR / 2; hi = P + wh / 2 + CLR / 2
        ox = np.clip(np.minimum(hi[:, None, 0], hi[None, :, 0]) - np.maximum(lo[:, None, 0], lo[None, :, 0]), 0, None)
        oy = np.clip(np.minimum(hi[:, None, 1], hi[None, :, 1]) - np.maximum(lo[:, None, 1], lo[None, :, 1]), 0, None)
        ov = np.triu(ox * oy, 1); nf = len(self.fixed); ov[NM:, NM:] = 0
        overlap = ov.sum()
        # noise: regulators / switchers away from the TCXO (penalise < 25 mm)
        t = xy[IDX['Y1_TCXO']]
        dn = [np.abs(xy[IDX[n]] - t).sum() for n in NOISY]
        noise = sum(max(0, 25 - d) ** 2 for d in dn)
        # trigger path mismatch (Manhattan, cluster centres)
        L = []
        for ch, path in TRIG.items():
            ids = [self.names.index(p) for p in path]
            L.append(sum(np.abs(P[a] - P[b]).sum() for a, b in zip(ids, ids[1:])))
        mism = max(L) - min(L)
        # edge parts: distance to closest board edge
        edge = 0
        for k in ('J_SWD', 'J_UART', 'J_PPS', 'LEDS'):
            i = IDX[k]; hw, hh = wh[i] / 2; x, y = xy[i]
            edge += min(x - hw, self.W - x - hw, y - hh, self.H - y - hh)
        crit = hp['clk'] + hp['rmii'] + hp['eth']
        cost = 1.0 * tot + 4.0 * crit + 400.0 * overlap + 3.0 * noise + 1.5 * mism + 2.0 * edge
        return cost, dict(hpwl_total=sum(hp.values()), hpwl_crit=crit, **{f'hpwl_{k}': v for k, v in hp.items()},
                          overlap=overlap, noise_min_dist=min(dn), trig_mismatch=mism, trig_len=L, edge=edge)

def run(W, H, pop=160, gens=500, verbose=False):
    B = Board(W, H)
    popl = [B.random() for _ in range(pop)]
    fit = np.array([B.metrics(*p)[0] for p in popl])
    hist = []; t0 = time.time()
    sig0 = max(W, H) * 0.15
    for g in range(gens):
        sig = sig0 * (1 - g / gens) + 0.3
        new = [popl[i] for i in np.argsort(fit)[:6]]                                   # elitism
        while len(new) < pop:
            a = min(RNG.choice(pop, 3), key=lambda i: fit[i]); b = min(RNG.choice(pop, 3), key=lambda i: fit[i])
            m = RNG.random(NM) < 0.5                                                   # uniform crossover per part
            xy = np.where(m[:, None], popl[a][0], popl[b][0]).copy(); rot = np.where(m, popl[a][1], popl[b][1]).copy()
            mu = RNG.random(NM) < 0.12
            xy[mu] += RNG.normal(0, sig, (mu.sum(), 2))
            rm = RNG.random(NM) < 0.03; rot[rm] = RNG.integers(0, 4, rm.sum())
            if RNG.random() < 0.15:                                                    # swap two same-zone parts
                i, j = RNG.choice(NM, 2, replace=False)
                if B.zone[i] == B.zone[j]: xy[[i, j]] = xy[[j, i]]
            new.append(B.repair(xy, rot))
        popl = new; fit = np.array([B.metrics(*p)[0] for p in popl])
        hist.append((g, fit.min(), np.median(fit)))
        if verbose and g % 50 == 0: print(W, H, g, round(fit.min(), 1), flush=True)
    best = popl[int(np.argmin(fit))]
    return B, best, B.metrics(*best), hist, time.time() - t0

if __name__ == '__main__':
    out = {}
    for W, H in [(75, 70), (80, 70), (85, 75), (90, 80), (90, 90)]:
        B, best, (cost, m), hist, dt = run(W, H)
        print(W, H, 'cost', round(cost, 1), 'overlap', round(m['overlap'], 2), 'crit', round(m['hpwl_crit'], 1),
              'tot', round(m['hpwl_total'], 1), 'noise', round(m['noise_min_dist'], 1), 'mism', round(m['trig_mismatch'], 1), 't', round(dt, 1), flush=True)
        out[f'{W}x{H}'] = dict(cost=cost, metrics={k: (v if not isinstance(v, list) else [float(x) for x in v]) for k, v in m.items()},
                               xy=best[0].tolist(), rot=best[1].tolist(), hist=hist, time=dt)
    json.dump(out, open('ga_sweep.json', 'w'), default=float)
