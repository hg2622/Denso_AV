"""How well does QCNet forecast COLLISIONS H seconds ahead on a logged run (e.g. a RIFT free-roam log)?

At every 10 Hz step t, for every pair of vehicles both observed at t:
  truth(t)      = their oriented boxes overlap at some step in (t, t+H]  (from the log; this is what actually happened)
  predicted(t)  = some pair of predicted modes (one per vehicle), at the same future step, overlap within H, and the pair's
                  joint probability p_i * p_j is at least --pmin  (0 = any mode pair)
Alarm at step t = predicted(t) for any pair. We report, per horizon, pair-level and step-level precision / recall / F1, plus
how early the first alarm comes before each real collision ("lead time"), and the same for a constant-velocity baseline.

    ~/miniconda3/envs/qcnet/bin/python qcnet_carla/collision_forecast.py --log selected_runs/rift/rift5.npz --town Town05 [--horizons 1,2,3,4] [--pmin 0.02]
"""
import argparse, math, os, sys
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla, numpy as np
from shapely.geometry import Polygon
from shapely.strtree import STRtree
from qcnet_model import QCNetCarla, CarlaMapGraph, HIST, FUT, DT, to_av

ap = argparse.ArgumentParser(); ap.add_argument('--log', required=True); ap.add_argument('--town', default='Town05')
ap.add_argument('--horizons', default='1,2,3,4'); ap.add_argument('--pmin', type=float, default=0.02, help='min joint mode probability for a predicted overlap to count')
ap.add_argument('--radius', type=float, default=70); ap.add_argument('--stride', type=int, default=1); ap.add_argument('--out', default='')
a = ap.parse_args(); horizons = [float(h) for h in a.horizons.split(',')]

z = np.load(a.log); rows = z['rows']; rift_dt = float(z['dt']); sub = int(round(DT / rift_dt))
ids = np.unique(rows[:, 1]).astype(int); id_to_j = {i: j for j, i in enumerate(ids)}
ticks = np.unique(rows[:, 0]).astype(int); ticks = ticks[ticks % sub == 0]; t_to_k = {t: k for k, t in enumerate(ticks)}
T, N = len(ticks), len(ids); log = np.full((T, N, 5), np.nan, np.float32); role = np.zeros((T, N), np.int8); ext = np.zeros((N, 2))
for r in rows:
    t = int(r[0])
    if t not in t_to_k: continue
    k, j = t_to_k[t], id_to_j[int(r[1])]; x, y, h = to_av(r[2], r[3], r[4]); log[k, j] = (x, y, h, r[5], -r[6]); role[k, j] = int(r[9]); ext[j] = (r[7], r[8])
ego_j = int(np.argmax((role == 1).sum(0)))

def box(x, y, h, L, W):
    c, s = math.cos(h), math.sin(h); pts = np.array([[L, W], [L, -W], [-L, -W], [-L, W]]) @ np.array([[c, s], [-s, c]]) + [x, y]; return Polygon(pts)

# ------------------------------------------------------------------ ground-truth overlaps per step (all pairs, first time they touch)
print('scanning ground truth ...', flush=True)
gt_overlap = {}   # k -> set of (j1, j2) overlapping at step k
for k in range(T):
    alive = [j for j in range(N) if not np.isnan(log[k, j, 0])]
    polys = [box(*log[k, j, :3], ext[j, 0], ext[j, 1]) for j in alive]; tree = STRtree(polys); S = set()
    for a_, pa in zip(alive, polys):
        for i2 in tree.query(pa):
            b_ = alive[i2]
            if b_ > a_ and polys[i2].intersects(pa): S.add((a_, b_))
    gt_overlap[k] = S
events = sorted({(k, p) for k, S in gt_overlap.items() for p in S if p not in gt_overlap.get(k - 1, set())})   # first-contact events
print(f'{len(events)} first-contact events in {T / 10:.0f} s ({sum(1 for k, p in events if 2 in (role[k, p[0]], role[k, p[1]]))} involve a RIFT car, {sum(1 for k, p in events if ego_j in p)} involve the ego)', flush=True)

# ------------------------------------------------------------------ predictions
client = carla.Client('localhost', 2000); client.set_timeout(180.0); w = client.get_world()
if a.town not in w.get_map().name: w = client.load_world(a.town)
graph = CarlaMapGraph(w.get_map()); model = QCNetCarla(); print('model ready', flush=True)
Hmax = int(max(horizons) / DT)
res = {h: dict(tp=0, fp=0, fn=0, step_tp=0, step_fp=0, step_fn=0, step_tn=0, lead=[]) for h in horizons}
cv_res = {h: dict(tp=0, fp=0, fn=0, step_tp=0, step_fp=0, step_fn=0, step_tn=0, lead=[]) for h in horizons}
first_alarm = {h: {} for h in horizons}; cv_first_alarm = {h: {} for h in horizons}
for k in range(HIST - 1, T - Hmax, a.stride):
    ex, ey = log[k, ego_j, :2]
    if np.isnan(ex): continue
    sel = [j for j in range(N) if not np.isnan(log[k, j, 0]) and np.hypot(log[k, j, 0] - ex, log[k, j, 1] - ey) < a.radius and (~np.isnan(log[k - HIST + 1:k + 1, j, 0])).sum() >= 10]
    if ego_j not in sel: sel.insert(0, ego_j)
    agents = [dict(id=int(ids[j]), type='vehicle', hist=log[k - HIST + 1:k + 1, j]) for j in sel]
    traj, pi, _ = model.predict(agents, graph, sel.index(ego_j))                   # (n, 6, 60, 2)
    n = len(sel)
    # heading along each predicted mode (for the boxes)
    cur = np.repeat(log[k, sel, :2][:, None, None, :], 6, axis=1)                      # (n, 6, 1, 2)
    d = np.diff(np.concatenate([cur, traj], 2), axis=2); hd = np.arctan2(d[..., 1], d[..., 0])   # (n, 6, 60)
    # constant-velocity baseline
    cv = log[k, sel, None, :2] + log[k, sel, None, 3:5] * (np.arange(1, Hmax + 1) * DT)[None, :, None]; cvh = log[k, sel, 2]
    # truth within each horizon, per pair
    for h in horizons:
        Hs = int(h / DT); truth = set()
        for kk in range(k + 1, k + 1 + Hs):
            for (p, q) in gt_overlap.get(kk, set()):
                if p in sel and q in sel: truth.add((p, q))
        # predicted overlaps: any pair of modes at the same step within Hs with joint prob >= pmin
        pred = set(); cvpred = set()
        for s in range(0, Hs, 2):   # every 0.2 s is enough for a box test
            polys = {}; cvp = {}
            for i_, j in enumerate(sel):
                for m in range(6):
                    if pi[i_, m] < 1e-3: continue
                    polys[(i_, m)] = box(traj[i_, m, s, 0], traj[i_, m, s, 1], hd[i_, m, s], ext[j, 0], ext[j, 1])
                cvp[i_] = box(cv[i_, s, 0], cv[i_, s, 1], cvh[i_], ext[j, 0], ext[j, 1])
            keys = list(polys); tree = STRtree([polys[kk_] for kk_ in keys])
            for ia, ka in enumerate(keys):
                for ib in tree.query(polys[ka]):
                    kb = keys[ib]
                    if kb[0] <= ka[0]: continue
                    if pi[ka[0], ka[1]] * pi[kb[0], kb[1]] >= a.pmin and polys[ka].intersects(polys[kb]): pred.add((sel[ka[0]], sel[kb[0]]))
            ck = list(cvp); ctree = STRtree([cvp[c] for c in ck])
            for ia, ca in enumerate(ck):
                for ib in ctree.query(cvp[ca]):
                    cb = ck[ib]
                    if cb > ca and cvp[ca].intersects(cvp[cb]): cvpred.add((sel[ca], sel[cb]))
        for R, P, FA in ((res[h], pred, first_alarm[h]), (cv_res[h], cvpred, cv_first_alarm[h])):
            R['tp'] += len(truth & P); R['fp'] += len(P - truth); R['fn'] += len(truth - P)
            R['step_tp'] += int(bool(truth) and bool(P)); R['step_fp'] += int(not truth and bool(P)); R['step_fn'] += int(bool(truth) and not P); R['step_tn'] += int(not truth and not P)
            for p in P: FA.setdefault(p, []).append(k)
# lead time: for each real first-contact event, how long before it was the pair first flagged (continuously within the horizon window)
for h in horizons:
    for R, FA in ((res[h], first_alarm[h]), (cv_res[h], cv_first_alarm[h])):
        for k, p in events:
            ks = [kk for kk in FA.get(p, []) if k - int(h / DT) <= kk < k]
            R['lead'].append((k - min(ks)) * DT if ks else 0.0)

def report(name, R):
    pr = R['tp'] / max(1, R['tp'] + R['fp']); rc = R['tp'] / max(1, R['tp'] + R['fn']); f1 = 2 * pr * rc / max(1e-9, pr + rc)
    spr = R['step_tp'] / max(1, R['step_tp'] + R['step_fp']); src = R['step_tp'] / max(1, R['step_tp'] + R['step_fn'])
    lead = np.array(R['lead']); caught = (lead > 0).mean() if len(lead) else 0
    return f'{name:18s} pairs: precision {pr:.2f} recall {rc:.2f} F1 {f1:.2f} (tp {R["tp"]}, fp {R["fp"]}, fn {R["fn"]})   steps: precision {spr:.2f} recall {src:.2f}   events caught {caught * 100:.0f}% , mean lead {lead[lead > 0].mean() if caught else 0:.2f} s'
lines = [f'log {a.log}: {T / 10:.0f} s, {N} vehicles, {len(events)} first-contact events; pmin {a.pmin}']
for h in horizons:
    lines.append(f'--- horizon {h:.0f} s ---'); lines.append(report('QCNet', res[h])); lines.append(report('constant velocity', cv_res[h]))
print('\n'.join(lines), flush=True)
if a.out: open(a.out, 'w').write('\n'.join(lines) + '\n')
