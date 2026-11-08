"""Synchronisation-fault dataset + decision-tree classifier for the Sync Hub (AI-SED 2026, Borrenautas).

Frame-level simulation (30 fps) of every camera's trigger->strobe latency, with the network-health signals
taken from the PTP simulator (ptp_sim.py). Features are computed exactly as firmware/sync_monitor.c does:
  [0] lat_dev_us   [1] lat_std_us   [2] missing_rate   [3] extra_pulses   [4] lat_slope_us_min
  [5] ptp_offset_ns  [6] path_delay_std_us  [7] freq_corr_rate_ppb_s  [8] pps_phase_err_ns
Outputs: anomaly_tree.c (if/else C, depth 6), anomaly_results.json, confusion matrices.
"""
import numpy as np, json, sys, time
sys.path.insert(0, '.')
import ptp_sim as P
from sklearn.tree import DecisionTreeClassifier
from sklearn.metrics import f1_score, accuracy_score, confusion_matrix, classification_report

CLASSES = ['normal', 'cam_disconnected', 'intermittent_cable', 'latency_drift', 'double_trigger_EMI',
           'ptp_master_loss', 'net_congestion', 'oscillator_fault']
FPS, T_SEQ, CAL = 30, 300, 60
rng = np.random.default_rng(7)

# ---------------- network-health per second, from the PTP simulator (PI servo, 8 Hz) ----------------
def net_seconds(profile, n_sec):
    P.N_SEQ_STEPS = n_sec * 8 + 400
    s = P.make_sequence(rng, profile); n = P.N_SEQ_STEPS
    theta = 0.0; I = s['y'][0]; fwd = np.empty(n); rev = np.empty(n); u = np.empty(n); z = np.empty(n)
    for k in range(n):
        fwd[k] = theta + s['df'][k] + rng.normal(0, 4); rev[k] = s['dr'][k] - theta + rng.normal(0, 4)
        lo = max(0, k - 63); z1 = 0.5 * (fwd[lo:k + 1].min() - rev[lo:k + 1].min()); zc = z1 * 1e-9
        I += P.SERVO['ki'] * zc / P.TS; u[k] = P.SERVO['kp'] * zc / P.TS + I; z[k] = z1
        theta += (s['y'][k] - u[k]) * P.TS * 1e9
    sl = slice(400, n)
    Z = z[sl].reshape(n_sec, 8); U = u[sl].reshape(n_sec, 8) * 1e9; PD = (0.5 * (fwd[sl] + rev[sl])).reshape(n_sec, 8) * 1e-3
    off = np.abs(Z).mean(1); pstd = PD.std(1); frate = np.abs(np.diff(np.r_[U[0, 0], U[:, -1]]))
    pps = np.abs(Z[:, -1]) * 0.5 + np.abs(rng.normal(0, 12, n_sec))                    # PPS loop-back phase error
    return np.column_stack([off, pstd, frate, pps])

NET_POOL = {}
def net_pool(profile, n_seq=12):
    if profile not in NET_POOL: NET_POOL[profile] = [net_seconds(profile, T_SEQ) for _ in range(n_seq)]
    return NET_POOL[profile][rng.integers(len(NET_POOL[profile]))].copy()

# ---------------- one camera, one sequence ----------------
def camera_sequence(cls):
    T = T_SEQ; t0 = int(rng.integers(90, 240)) if cls else T + 1
    base = rng.uniform(15, 60); jit = rng.uniform(0.03, 0.15)
    lat = base + rng.normal(0, jit, (T, FPS)); got = np.ones((T, FPS), bool); extra = np.zeros(T)
    net = net_pool('light' if rng.random() < 0.5 else 'medium')
    sec = np.arange(T); after = sec >= t0
    if cls == 1: got[after] = False
    if cls == 2:
        for s_ in np.nonzero(after)[0]:
            if rng.random() < 0.85:
                got[s_] &= rng.random(FPS) > rng.uniform(0.03, 0.5); lat[s_] += rng.normal(0, jit * rng.uniform(3, 10), FPS)
    if cls == 3: lat += (np.clip(sec - t0, 0, None) / 60.0 * rng.uniform(0.5, 3.0))[:, None]
    if cls == 4: extra[after] = rng.poisson(rng.uniform(1, 10), after.sum())
    if cls == 5:
        r = rng.uniform(20, 200); k = np.clip(sec - t0, 0, None)
        net[after, 0] = net[t0 - 1, 0] + r * k[after]; net[after, 1] = 0.0
        net[after, 2] = np.abs(rng.normal(0, 0.05, after.sum())); net[after, 3] = net[after, 0] * rng.uniform(0.8, 1.2)
    if cls == 6:
        heavy = net_pool('heavy'); net[after] = heavy[after]
    if cls == 7:
        m = after.sum()
        net[after, 2] = rng.uniform(20, 500, m); net[after, 0] = rng.uniform(300, 3000, m); net[after, 3] = rng.uniform(100, 1000, m)
    # ---- features exactly as sync_monitor.c ----
    nrx = got.sum(1); m = np.where(nrx > 0, np.where(got, lat, 0).sum(1) / np.maximum(nrx, 1), 0)
    sq = np.where(got, lat ** 2, 0).sum(1) / np.maximum(nrx, 1)
    sd = np.where(nrx > 1, np.sqrt(np.maximum(sq - m ** 2, 0)), 0)
    basev = m[:CAL].mean()
    ld = np.where(nrx > 0, m - basev, 0.0)
    hist = np.zeros(30); slope = np.zeros(T)
    for s_ in range(T):
        slope[s_] = (ld[s_] - hist[s_ % 30]) * 2.0; hist[s_ % 30] = ld[s_]
    X = np.column_stack([ld, sd, 1 - nrx / FPS, extra, slope, net])
    y = np.where(after, cls, 0)
    return X[CAL:], y[CAL:], t0 - CAL

TH = dict(miss_disc=0.9, extra=0.5, miss_int=0.02, path=5.0, frate=20.0, off=1000.0, ld=1.0)
def thresholds(X, th=None):
    """fixed-threshold baseline: one threshold per symptom, checked in priority order
    (threshold values tuned on the training set by coordinate search, see tune_thresholds)."""
    th = th or TH; out = np.zeros(len(X), int)
    rules = [(X[:, 2] > th['miss_disc'], 1), (X[:, 3] > th['extra'], 4), (X[:, 2] > th['miss_int'], 2),
             (X[:, 6] > th['path'], 6), (X[:, 7] > th['frate'], 7), (X[:, 5] > th['off'], 5), (np.abs(X[:, 0]) > th['ld'], 3)]
    done = np.zeros(len(X), bool)
    for m, c in rules:
        sel = m & ~done; out[sel] = c; done |= m
    return out

def tune_thresholds(X, y):
    grid = {'miss_disc': [0.5, 0.7, 0.9, 0.97], 'extra': [0.5, 1.5, 2.5], 'miss_int': [0.01, 0.02, 0.04, 0.08, 0.15],
            'path': list(np.quantile(X[:, 6], [0.5, 0.7, 0.8, 0.9, 0.95, 0.98])), 'frate': list(np.quantile(X[:, 7], [0.7, 0.8, 0.9, 0.95, 0.98, 0.99])),
            'off': list(np.quantile(X[:, 5], [0.7, 0.8, 0.9, 0.95, 0.98])), 'ld': [0.3, 0.5, 1.0, 2.0, 4.0]}
    th = dict(TH)
    for _ in range(3):
        for k, vals in grid.items():
            th[k] = max(vals, key=lambda v: f1_score(y, thresholds(X, {**th, k: v}), average='macro'))
    return th

def persist(pred, k=3):
    st = np.zeros_like(pred); cand = 0; run = 0
    for i, p in enumerate(pred):
        if p != 0 and p == cand: run += 1
        else: cand, run = p, int(p != 0)
        st[i] = cand if run >= k else 0
    return st

def export_tree(clf, path):
    t = clf.tree_; L = []
    def rec(n, d):
        ind = '    ' * d
        if t.children_left[n] == -1:
            c = int(np.argmax(t.value[n][0])); L.append(f'{ind}return (sync_fault_t){c}; /* {CLASSES[c]} */'); return
        L.append(f'{ind}if (x[{t.feature[n]}] <= {t.threshold[n]:.6g}f) {{'); rec(t.children_left[n], d + 1)
        L.append(f'{ind}}} else {{'); rec(t.children_right[n], d + 1); L.append(f'{ind}}}')
    rec(0, 1)
    hdr = (f'/* Auto-generated by anomaly.py - decision tree (depth {clf.get_depth()}, {t.node_count} nodes) for sync-fault diagnosis */\n'
           '#include "anomaly_tree.h"\n/* x[]: [0] lat_dev_us, [1] lat_std_us, [2] missing_rate, [3] extra_pulses, [4] lat_slope_us_min, '
           '[5] ptp_offset_ns, [6] path_delay_std_us, [7] freq_corr_rate_ppb_s, [8] pps_phase_err_ns */\n'
           'sync_fault_t anomaly_classify(const float x[ANOM_NFEAT])\n{\n')
    open(path, 'w').write(hdr + '\n'.join(L) + '\n}\n')

if __name__ == '__main__':
    t_start = time.time(); N = int(sys.argv[1]) if len(sys.argv) > 1 else 120
    seqs = [(c, *camera_sequence(c)) for c in range(8) for _ in range(N * (2 if c == 0 else 1))]
    idx = rng.permutation(len(seqs)); ntr = int(0.7 * len(seqs)); nva = int(0.15 * len(seqs))
    sp = {'train': idx[:ntr], 'val': idx[ntr:ntr + nva], 'test': idx[ntr + nva:]}
    XY = {k: (np.vstack([seqs[i][1] for i in v]), np.concatenate([seqs[i][2] for i in v])) for k, v in sp.items()}
    # class_weight=None chosen on the validation set (macro-F1 0.955 vs 0.929 with class_weight='balanced')
    clf = DecisionTreeClassifier(max_depth=6, min_samples_leaf=20, class_weight=None, random_state=0).fit(*XY['train'])
    export_tree(clf, 'anomaly_tree.c')
    sub = rng.choice(len(XY['train'][1]), 60000, replace=False)
    TH.update(tune_thresholds(XY['train'][0][sub], XY['train'][1][sub]))
    Xt, yt = XY['test']; pt = clf.predict(Xt); pb = thresholds(Xt)
    res = dict(tree=dict(acc=accuracy_score(yt, pt), f1=f1_score(yt, pt, average='macro')),
               thr=dict(acc=accuracy_score(yt, pb), f1=f1_score(yt, pb, average='macro')),
               val=dict(acc=accuracy_score(XY['val'][1], clf.predict(XY['val'][0])), f1=f1_score(XY['val'][1], clf.predict(XY['val'][0]), average='macro')),
               cm_tree=confusion_matrix(yt, pt, labels=range(8)).tolist(), cm_thr=confusion_matrix(yt, pb, labels=range(8)).tolist(),
               report=classification_report(yt, pt, target_names=CLASSES, output_dict=True, zero_division=0),
               n_seq={k: int(len(v)) for k, v in sp.items()}, n_windows={k: int(len(XY[k][1])) for k in XY},
               class_balance={CLASSES[c]: int((XY['train'][1] == c).sum()) for c in range(8)},
               depth=int(clf.get_depth()), nodes=int(clf.tree_.node_count), thresholds={k: float(v) for k, v in TH.items()})
    # detection latency + false alarms with the 3-of-3 persistence filter (test sequences)
    lat_t, lat_b, fa_t, fa_b, miss_t, miss_b = [], [], 0, 0, 0, 0; normal_s = 0
    for i in sp['test']:
        c, X, y, t0 = seqs[i]
        st = persist(clf.predict(X)); sb = persist(thresholds(X))
        if c == 0:
            fa_t += (st != 0).sum(); fa_b += (sb != 0).sum(); normal_s += len(X)
        else:
            pre = slice(0, max(t0, 0)); fa_t += (st[pre] != 0).sum(); fa_b += (sb[pre] != 0).sum(); normal_s += max(t0, 0)
            for s_, L_, mc in ((st, lat_t, 't'), (sb, lat_b, 'b')):
                hit = np.nonzero(s_[t0:] == c)[0]
                if len(hit): L_.append(int(hit[0]) + 1)
                elif mc == 't': miss_t += 1
                else: miss_b += 1
    res['latency'] = dict(tree_median_s=float(np.median(lat_t)), tree_p90_s=float(np.percentile(lat_t, 90)),
                          thr_median_s=float(np.median(lat_b)), thr_p90_s=float(np.percentile(lat_b, 90)),
                          tree_detected=len(lat_t), thr_detected=len(lat_b), tree_missed=miss_t, thr_missed=miss_b,
                          false_alarm_tree_per_h=fa_t / normal_s * 3600, false_alarm_thr_per_h=fa_b / normal_s * 3600)
    res['time_s'] = time.time() - t_start
    json.dump(res, open('anomaly_results.json', 'w'), indent=1, default=float)
    print(json.dumps({k: v for k, v in res.items() if k not in ('cm_tree', 'cm_thr', 'report')}, indent=1, default=float))
    print(classification_report(yt, pt, target_names=CLASSES, zero_division=0))
