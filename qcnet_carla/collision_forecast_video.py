"""Video of QCNet collision forecasting on a logged run: at every step, each vehicle's predicted modes (opacity = probability)
up to the horizon H; when a pair of modes overlaps within H, both predicted boxes at that moment are drawn in magenta with a
cross at the overlap point, labelled "+t s  p" (time ahead, joint probability). A real contact that happens within the next H
seconds is shown with a black ring at the place it will happen, so a warning on top of a ring is a hit and a warning alone is
a false alarm. Top-right counters keep the running tally.

    ~/miniconda3/envs/qcnet/bin/python qcnet_carla/collision_forecast_video.py --log selected_runs/rift/rift5.npz --town Town05 --out X.mp4 [--horizon 2 --pmin 0.02]
"""
import argparse, math, os, sys
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla, numpy as np, cv2
from shapely.geometry import Polygon
from shapely.strtree import STRtree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; from matplotlib.patches import Polygon as MplPoly, Circle
from qcnet_model import QCNetCarla, CarlaMapGraph, HIST, FUT, DT, to_av

ap = argparse.ArgumentParser(); ap.add_argument('--log', required=True); ap.add_argument('--out', required=True); ap.add_argument('--town', default='Town05')
ap.add_argument('--horizon', type=float, default=2.0); ap.add_argument('--pmin', type=float, default=0.02); ap.add_argument('--view', type=float, default=45); ap.add_argument('--radius', type=float, default=70)
a = ap.parse_args(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True); Hs = int(a.horizon / DT)

z = np.load(a.log); rows = z['rows']; sub = int(round(DT / float(z['dt'])))
ids = np.unique(rows[:, 1]).astype(int); id_to_j = {i: j for j, i in enumerate(ids)}
ticks = np.unique(rows[:, 0]).astype(int); ticks = ticks[ticks % sub == 0]; t_to_k = {t: k for k, t in enumerate(ticks)}
T, N = len(ticks), len(ids); log = np.full((T, N, 5), np.nan, np.float32); role = np.zeros((T, N), np.int8); ext = np.zeros((N, 2))
for r in rows:
    t = int(r[0])
    if t not in t_to_k: continue
    k, j = t_to_k[t], id_to_j[int(r[1])]; x, y, h = to_av(r[2], r[3], r[4]); log[k, j] = (x, y, h, r[5], -r[6]); role[k, j] = int(r[9]); ext[j] = (r[7], r[8])
ego_j = int(np.argmax((role == 1).sum(0)))

def corners(x, y, h, L, W):
    c, s = math.cos(h), math.sin(h); return np.array([[L, W], [L, -W], [-L, -W], [-L, W]]) @ np.array([[c, s], [-s, c]]) + [x, y]

# ground truth contacts: k -> {(p, q): overlap centroid}
gt = {}
for k in range(T):
    alive = [j for j in range(N) if not np.isnan(log[k, j, 0])]; polys = [Polygon(corners(*log[k, j, :3], ext[j, 0], ext[j, 1])) for j in alive]; tree = STRtree(polys); S = {}
    for a_, pa in zip(alive, polys):
        for i2 in tree.query(pa):
            b_ = alive[i2]
            if b_ > a_ and polys[i2].intersects(pa): c = polys[i2].intersection(pa).centroid; S[(a_, b_)] = (c.x, c.y)
    gt[k] = S
events = {(k, p): c for k, S in gt.items() for p, c in S.items() if p not in gt.get(k - 1, {})}
event_of = {}   # (step, pair) -> first-contact step of that contiguous overlap
for k in range(T):
    for p in gt[k]: event_of[(k, p)] = event_of.get((k - 1, p), k)
print(f'{T / 10:.0f} s, {N} vehicles, {len(events)} first-contact events', flush=True)

client = carla.Client('localhost', 2000); client.set_timeout(180.0); w = client.get_world()
if a.town not in w.get_map().name: w = client.load_world(a.town)
graph = CarlaMapGraph(w.get_map()); model = QCNetCarla()
lane_lines = [(p['center'][:, :2], p['left'][:, :2], p['right'][:, :2], p['ptype']) for p in graph.polys]
Wn, Hn = 1100, 1100; writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), 10.0, (Wn, Hn))
fig = plt.figure(figsize=(11, 11), dpi=100); tp = fp = fn = 0; caught = set(); n = 0; hit_log = []
for k in range(HIST - 1, T - Hs):
    ex, ey = log[k, ego_j, :2]
    if np.isnan(ex): continue
    sel = [j for j in range(N) if not np.isnan(log[k, j, 0]) and np.hypot(log[k, j, 0] - ex, log[k, j, 1] - ey) < a.radius and (~np.isnan(log[k - HIST + 1:k + 1, j, 0])).sum() >= 10]
    if ego_j not in sel: sel.insert(0, ego_j)
    traj, pi, _ = model.predict([dict(id=int(ids[j]), type='vehicle', hist=log[k - HIST + 1:k + 1, j]) for j in sel], graph, sel.index(ego_j))
    cur = np.repeat(log[k, sel, :2][:, None, None, :], 6, axis=1); d = np.diff(np.concatenate([cur, traj], 2), axis=2); hd = np.arctan2(d[..., 1], d[..., 0])
    # predicted overlaps within the horizon: keep the earliest per pair, with the highest joint probability at that step
    warn = {}   # (p, q) -> (s, jp, cornersA, cornersB, (cx, cy))
    for s in range(0, Hs, 2):
        polys = {}
        for i_, j in enumerate(sel):
            for m in range(6):
                if pi[i_, m] >= 1e-3: polys[(i_, m)] = Polygon(corners(traj[i_, m, s, 0], traj[i_, m, s, 1], hd[i_, m, s], ext[j, 0], ext[j, 1]))
        keys = list(polys); tree = STRtree([polys[q] for q in keys])
        for ia, ka in enumerate(keys):
            for ib in tree.query(polys[ka]):
                kb = keys[ib]
                if kb[0] <= ka[0]: continue
                jp = float(pi[ka[0], ka[1]] * pi[kb[0], kb[1]])
                if jp < a.pmin or not polys[ka].intersects(polys[kb]): continue
                pair = (sel[ka[0]], sel[kb[0]]); c = polys[ka].intersection(polys[kb]).centroid
                if pair not in warn or (warn[pair][0] == s and jp > warn[pair][1]): warn[pair] = (s, jp, np.array(polys[ka].exterior.coords)[:4], np.array(polys[kb].exterior.coords)[:4], (c.x, c.y))
    truth = {}
    for kk in range(k + 1, k + 1 + Hs):
        for p, c in gt.get(kk, {}).items():
            if p[0] in sel and p[1] in sel and p not in truth: truth[p] = (kk, c)
    tp += len(set(warn) & set(truth)); fp += len(set(warn) - set(truth)); fn += len(set(truth) - set(warn))
    for p in set(warn) & set(truth):
        ev = (event_of[(truth[p][0], p)], p)
        if ev not in caught: caught.add(ev); hit_log.append((round(k * DT, 1), round(truth[p][0] * DT, 1), p))
    # ---- render
    fig.clf(); ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor('#f4f4f2'); V = a.view
    for c, l, r, pt in lane_lines:
        if abs(c[0, 0] - ex) > V + 40 or abs(c[0, 1] - ey) > V + 40 or pt == 'PEDESTRIAN': continue
        ax.plot(c[:, 0], c[:, 1], color='#d0d0d0', lw=0.6); ax.plot(l[:, 0], l[:, 1], color='#b8b8b8', lw=0.5); ax.plot(r[:, 0], r[:, 1], color='#b8b8b8', lw=0.5)
    for i_, j in enumerate(sel):
        x, y, h = log[k, j, :3]; under = role[k, j] == 2; col = '#d62728' if j == ego_j else ('#ff7f0e' if under else '#1f77b4')
        ax.add_patch(MplPoly(corners(x, y, h, ext[j, 0], ext[j, 1]), closed=True, fc=col, ec='k', lw=2.0 if under else 0.6, zorder=6))
        order = np.argsort(-pi[i_]); pmax = max(float(pi[i_].max()), 1e-6)
        for mode in order:
            p_ = float(pi[i_, mode]); al = 0.12 + 0.88 * (p_ / pmax); tr = traj[i_, mode, :Hs]
            ax.plot(tr[:, 0], tr[:, 1], color='#e41a1c', lw=0.8 + 1.4 * (p_ / pmax), alpha=al, zorder=4)
            ax.add_patch(MplPoly(corners(tr[-1, 0], tr[-1, 1], hd[i_, mode, Hs - 1], ext[j, 0], ext[j, 1]), closed=True, fc='none', ec='#e41a1c', lw=0.6, alpha=al, zorder=4))
        g = log[k + 1:k + 1 + Hs, j, :2]
        if not np.isnan(g[:, 0]).any(): ax.plot(g[:, 0], g[:, 1], color='#2ca02c', lw=1.8, zorder=7)
    for p, (s, jp, ca, cb, (cx, cy)) in warn.items():
        hit = p in truth
        for cc in (ca, cb): ax.add_patch(MplPoly(cc, closed=True, fc='#ff00ff', ec='#800080', lw=1.6, alpha=0.55, zorder=8))
        ax.plot(cx, cy, 'x', color='#800080', ms=18, mew=3.5, zorder=9)
        for q in p: ax.plot([log[k, q, 0], cx], [log[k, q, 1], cy], color='#800080', lw=1.0, ls=':', zorder=8)
        ax.text(cx + 1.5, cy + 1.5, f'+{(s + 1) * DT:.1f} s  p={jp:.2f}' + ('  HIT' if hit else ''), fontsize=9, color='#800080', weight='bold', zorder=10, bbox=dict(fc='white', ec='none', alpha=0.8))
    for p, (kk, c) in truth.items():
        ax.add_patch(Circle(c, 2.2, fill=False, ec='k', lw=2.0, zorder=9)); ax.text(c[0] - 1.5, c[1] - 3.5, f'contact in {(kk - k) * DT:.1f} s', fontsize=8.5, color='k', zorder=10, bbox=dict(fc='white', ec='none', alpha=0.8))
    ax.set_xlim(ex - V, ex + V); ax.set_ylim(ey - V, ey + V); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    ax.text(0.01, 0.985, f'Collision forecast {a.horizon:.0f} s ahead, QCNet on a RIFT run ({a.town})   t = {k * DT:5.1f} s   orange = RIFT policy cars', transform=ax.transAxes, va='top', fontsize=10.5, bbox=dict(fc='white', ec='none', alpha=0.85))
    ax.text(0.01, 0.955, 'red = predicted modes to +2 s (box = predicted pose at +2 s)   green = actual next 2 s   magenta boxes + X = WARNING: predicted overlap, where and when   black ring = a real contact coming within 2 s', transform=ax.transAxes, va='top', fontsize=8.8, bbox=dict(fc='white', ec='none', alpha=0.85))
    ax.text(0.99, 0.985, f'warnings: {tp} hits / {fp} false   missed: {fn}   events caught {len(caught)}/{len(events)}', transform=ax.transAxes, va='top', ha='right', fontsize=10.5, color='#800080', weight='bold', bbox=dict(fc='white', ec='none', alpha=0.85))
    fig.canvas.draw(); frame = cv2.resize(np.asarray(fig.canvas.buffer_rgba())[:, :, :3], (Wn, Hn)); writer.write(np.ascontiguousarray(frame[:, :, ::-1])); n += 1
writer.release(); print('first warning of each caught event (t_warn, t_contact, pair):', hit_log, flush=True); print(f'=== {n} frames -> {a.out}; pair warnings {tp} hits / {fp} false / {fn} missed; events caught {len(caught)}/{len(events)}', flush=True)
