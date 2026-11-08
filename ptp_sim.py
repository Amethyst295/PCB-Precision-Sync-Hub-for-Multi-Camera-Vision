"""PTP master-slave link simulator + MLP bias corrector for the Sync Hub (AI-SED 2026, Borrenautas).

Reproduces exactly the servo implemented in firmware/ptp_servo.c:
  lucky-packet filter (sliding minimum, W = 64) -> 12 queue statistics over the last 32 exchanges ->
  MLP 13-16-8-1 estimates the residual bias of the filter -> OOD guard, EMA, gate, clip -> PI servo.
Steps:  python ptp_sim.py   (generates the dataset, trains the MLP, exports mlp_weights.h, evaluates PI vs AI)
"""
import numpy as np, json, time, sys
from numpy.lib.stride_tricks import sliding_window_view as swv
from sklearn.neural_network import MLPRegressor

TS = 0.125                 # Sync interval (8 Hz)
W_LUCKY, FW = 64, 32       # lucky-packet window, feature window (firmware: cfg.W, FW)
N_SEQ_STEPS, WARMUP = 2400, 400
HOPS, D0_NS = 3, 25_000    # 3 switch hops, fixed one-way latency 25 us (cables, PHYs, store-and-forward)
import os
SERVO = dict(kp=0.01, ki=1e-4, gate_us=0.5, clip_us=float(os.environ.get('CLIP_US', 5.0)), ema=1 / 16, ood_z=3.0)
PROFILES = {               # per-hop link load (fwd, rev) ranges
    'light':  ((0.05, 0.25), (0.05, 0.25)),
    'medium': ((0.30, 0.55), (0.30, 0.55)),
    'heavy':  ((0.60, 0.85), (0.35, 0.85)),     # asymmetric: camera video loads one direction more
    'temp_ramp': ((0.30, 0.55), (0.30, 0.55)),
}

def frame_ns(rng, n):
    """GbE frame service times: 40 % 64 B, 50 % 1518 B, 10 % uniform."""
    u = rng.random(n); size = np.where(u < 0.4, 64, np.where(u < 0.9, 1518, rng.uniform(64, 1518, n)))
    return (size + 20) * 8.0                       # ns at 1 Gb/s (incl. preamble/IFG)

def queue_delay(rng, rho):
    """one-way queuing delay over HOPS hops for a vector of per-packet loads rho (ns)."""
    n = len(rho); tot = np.zeros(n)
    for _ in range(HOPS):
        busy = rng.random(n) < rho
        k = rng.geometric(1 - rho) - 1             # frames waiting ahead (0, 1, 2, ...)
        tot += busy * (rng.random(n) * frame_ns(rng, n))            # residual of the frame in service
        maxk = int(k.max()) if n else 0
        for j in range(maxk):                      # whole frames ahead
            m = k > j
            tot[m] += frame_ns(rng, m.sum())
    return tot

def load_series(rng, lo, hi, n):
    """slowly varying load with bursts (AR(1) + random bursts), clipped to [lo, hi + 0.08]."""
    base = rng.uniform(lo, hi); x = np.empty(n); s = base
    for i in range(n):
        s = base + 0.97 * (s - base) + rng.normal(0, 0.02); x[i] = s
    bursts = rng.random(n) < 0.004
    for i in np.nonzero(bursts)[0]: x[i:i + rng.integers(8, 80)] += rng.uniform(0.05, 0.15)
    return np.clip(x, 0.0, min(hi + 0.08, 0.93))

def make_sequence(rng, profile):
    n = N_SEQ_STEPS
    (fl, fh), (rl, rh) = PROFILES[profile]
    if rng.random() < 0.5 and profile == 'heavy': (fl, fh), (rl, rh) = (rl, rh), (fl, fh)
    qf = queue_delay(rng, load_series(rng, fl, fh, n)); qr = queue_delay(rng, load_series(rng, rl, rh, n))
    y0 = rng.uniform(-2.5e-6, 2.5e-6)                                   # TCXO initial error
    t = np.arange(n) * TS
    if profile == 'temp_ramp':
        temp = 25 + 15 * np.clip(t / (n * TS * 0.8), 0, 1)              # 25 -> 40 C ramp (cabinet warm-up, ~3.75 C/min)
        y = y0 + 0.05e-6 * (temp - 25)
    else:
        y = y0 + np.cumsum(rng.normal(0, 2e-12, n))                     # random-walk FM
    return dict(df=D0_NS + qf, dr=D0_NS + qr, y=y, profile=profile)

# ----------------------------------------------------------------- firmware-identical features
def features_from(fwd, rev):
    """vectorised version of the feature block in ptp_servo.c (rows k >= FW-1)."""
    F = swv(fwd, FW); R = swv(rev, FW)                                  # last 32 incl. current
    fm = F.min(1); rm = R.min(1)
    df = (F - fm[:, None]) * 1e-3; dr = (R - rm[:, None]) * 1e-3
    cur_f, cur_r = fwd[FW - 1:], rev[FW - 1:]
    X = np.column_stack([(cur_f - fm) * 1e-3, (cur_r - rm) * 1e-3, ((cur_f - fm) - (cur_r - rm)) * 1e-3,
                         df.mean(1), dr.mean(1), df.std(1), dr.std(1), np.median(df, 1), np.median(dr, 1),
                         df[:, -4:].mean(1), dr[:, -4:].mean(1),
                         (0.5 * (cur_f + cur_r) - 0.5 * (fm + rm)) * 1e-3, np.zeros(len(fm))])
    return X

def features_step(fbuf, rbuf):
    f = np.asarray(fbuf[-FW:], float); r = np.asarray(rbuf[-FW:], float)
    return features_from(f, r)[-1]

class MLP:
    def __init__(self, d): self.__dict__.update(d)
    def __call__(self, x):
        z = (x - self.mu) / self.sd
        h1 = np.maximum(z @ self.W1 + self.b1, 0); h2 = np.maximum(h1 @ self.W2 + self.b2, 0)
        return float(h2 @ self.W3[:, 0] + self.b3[0])

def run_servo(seq, mlp=None, record=False):
    """closed-loop simulation. Returns true offset theta (ns) per step and features/targets."""
    n = N_SEQ_STEPS; theta = 0.0; I = seq['y'][0]; ema = 0.0
    fwd = np.empty(n); rev = np.empty(n); th = np.empty(n); z1s = np.empty(n); bias_used = np.zeros(n)
    q = lambda v, r: np.round(v / r) * r
    for k in range(n):
        f = q(theta + seq['df'][k], 20) + np.random.normal(0, 4)        # 20 ns slave TS resolution + jitter
        r = q(seq['dr'][k] - theta, 8) + np.random.normal(0, 4)         # 8 ns master TS resolution
        fwd[k], rev[k], th[k] = f, r, theta
        lo = max(0, k - W_LUCKY + 1)
        z1 = 0.5 * (fwd[lo:k + 1].min() - rev[lo:k + 1].min()); z1s[k] = z1
        b = 0.0
        if mlp is not None and k >= FW - 1:
            x = features_step(fwd[:k + 1], rev[:k + 1])
            ood = np.any(np.abs((x[:12] - mlp.mu[:12]) / mlp.sd[:12]) > SERVO['ood_z'])
            bb = 0.0 if ood else mlp(x) * 1e3
            ema = (1 - SERVO['ema']) * ema + SERVO['ema'] * bb; b = ema
            if abs(b) < SERVO['gate_us'] * 1e3: b = 0.0
            b = float(np.clip(b, -SERVO['clip_us'] * 1e3, SERVO['clip_us'] * 1e3))
        bias_used[k] = b
        zc = (z1 - b) * 1e-9
        I += SERVO['ki'] * zc / TS
        u = SERVO['kp'] * zc / TS + I
        theta += (seq['y'][k] - u) * TS * 1e9                          # negative feedback on the slave clock
    out = dict(theta=th, bias_used=bias_used, true_bias=z1s - th)
    if record:
        X = features_from(fwd, rev); tgt = (z1s - th)[FW - 1:] * 1e-3   # bias of the lucky-packet offset (us)
        out.update(X=X[WARMUP - FW + 1:], y=tgt[WARMUP - FW + 1:])
    return out

def export_c(m, mu, sd, path):
    W1, W2, W3 = m.coefs_; b1, b2, b3 = m.intercepts_
    fmt = lambda a: ', '.join(f'{(0.0 if abs(v) < 1e-30 else v):.8e}f' for v in np.ravel(a))
    rows = lambda A: ',\n'.join('    {' + fmt(r) + '}' for r in A)
    s = f"""/* Auto-generated by ptp_sim.py - PTP lucky-packet bias estimator, 13-16-8-1 ReLU MLP */
#ifndef MLP_WEIGHTS_H
#define MLP_WEIGHTS_H
#define MLP_NIN 13
#define MLP_H1 16
#define MLP_H2 8
static const float MLP_MU[MLP_NIN] = {{{fmt(mu)}}};
static const float MLP_SD[MLP_NIN] = {{{fmt(sd)}}};
static const float MLP_W1[MLP_NIN][MLP_H1] = {{
{rows(W1)}
}};
static const float MLP_B1[MLP_H1] = {{{fmt(b1)}}};
static const float MLP_W2[MLP_H1][MLP_H2] = {{
{rows(W2)}
}};
static const float MLP_B2[MLP_H2] = {{{fmt(b2)}}};
static const float MLP_W3[MLP_H2][1] = {{
{rows(W3)}
}};
static const float MLP_B3[1] = {{{fmt(b3)}}};
#endif
"""
    open(path, 'w').write(s)

if __name__ == '__main__':
    rng = np.random.default_rng(2026); np.random.seed(2026)
    N_PER = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    t0 = time.time(); seqs = []
    for p in PROFILES:
        for i in range(N_PER): seqs.append(make_sequence(rng, p))
    idx = rng.permutation(len(seqs)); ntr = int(0.7 * len(seqs)); nva = int(0.15 * len(seqs))
    split = {'train': idx[:ntr], 'val': idx[ntr:ntr + nva], 'test': idx[ntr + nva:]}
    # dataset: closed loop with the classic PI servo
    D = {k: [run_servo(seqs[i], None, record=True) for i in v] for k, v in split.items()}
    X = {k: np.vstack([d['X'] for d in v]) for k, v in D.items()}; Y = {k: np.concatenate([d['y'] for d in v]) for k, v in D.items()}
    print('dataset', {k: X[k].shape for k in X}, 'gen s', round(time.time() - t0, 1), flush=True)
    mu = X['train'].mean(0); sd = X['train'].std(0); mu[12] = 0; sd[12] = 1e-9
    Z = lambda A: (A - mu) / np.where(sd < 1e-6, 1, sd)
    m = MLPRegressor(hidden_layer_sizes=(16, 8), activation='relu', alpha=1e-4, learning_rate_init=2e-3,
                     max_iter=300, early_stopping=True, n_iter_no_change=12, random_state=1)
    m.fit(Z(X['train']), Y['train'])
    pred = {k: m.predict(Z(X[k])) for k in X}
    reg = {k: dict(rmse_us=float(np.sqrt(np.mean((pred[k] - Y[k]) ** 2))), std_target_us=float(Y[k].std()),
                   r2=float(1 - np.mean((pred[k] - Y[k]) ** 2) / Y[k].var())) for k in X}
    print('MLP', reg, 'epochs', m.n_iter_, flush=True)
    export_c(m, mu, sd, 'mlp_weights.h')
    mlp = MLP(dict(mu=mu, sd=np.where(sd < 1e-6, 1e-9, sd), W1=m.coefs_[0], W2=m.coefs_[1], W3=m.coefs_[2],
                   b1=m.intercepts_[0], b2=m.intercepts_[1], b3=m.intercepts_[2]))
    # closed-loop evaluation on the TEST sequences: PI vs PI + AI
    res = {}; traces = {}
    for i in split['test']:
        s = seqs[i]; p = s['profile']
        np.random.seed(int(i)); a = run_servo(s, None)['theta'][WARMUP:]
        np.random.seed(int(i)); b = run_servo(s, mlp)['theta'][WARMUP:]
        r = res.setdefault(p, dict(pi=[], ai=[], pimax=[], aimax=[]))
        r['pi'].append(np.mean(a ** 2)); r['ai'].append(np.mean(b ** 2)); r['pimax'].append(np.abs(a).max()); r['aimax'].append(np.abs(b).max())
        if p == 'heavy' and p not in traces: traces[p] = (a.tolist(), b.tolist())
    summ = {p: dict(n=len(r['pi']), rmse_pi=float(np.sqrt(np.mean(r['pi']))), rmse_ai=float(np.sqrt(np.mean(r['ai']))),
                    max_pi=float(np.max(r['pimax'])), max_ai=float(np.max(r['aimax']))) for p, r in res.items()}
    for p, v in summ.items(): v['improvement_pct'] = 100 * (1 - v['rmse_ai'] / v['rmse_pi'])
    print(json.dumps(summ, indent=1))
    json.dump(dict(dataset={k: int(X[k].shape[0]) for k in X}, n_seq={k: int(len(v)) for k, v in split.items()},
                   seq_steps=N_SEQ_STEPS, warmup=WARMUP, per_profile=N_PER, mlp=reg, epochs=int(m.n_iter_), closed_loop=summ,
                   trace_heavy=traces.get('heavy'), servo=SERVO, time_s=time.time() - t0,
                   xtest_sample=X['test'][:50].tolist(), pred_sample=pred['test'][:50].tolist()),
              open('ptp_results.json', 'w'))
