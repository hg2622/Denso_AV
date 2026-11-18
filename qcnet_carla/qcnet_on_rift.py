"""QCNet predictions on a logged RIFT free-roam run: the RL policy drives up to 5 cars around the ego (red outline), QCNet
predicts every car's next 6 s from its last 5 s and the map. Bird's-eye video: green = what each car actually did, red =
the 6 predicted modes with opacity = probability and a ghost car at each endpoint, orange dashed = the RIFT policy's own
planned path for the cars it controls (its intent), so you can compare "what the policy planned" with "what the predictor
expected" with "what happened".

    ~/miniconda3/envs/qcnet/bin/python qcnet_carla/qcnet_on_rift.py --log selected_runs/rift/rift5.npz --out selected_runs/qcnet/qcnet_on_rift.mp4 [--town Town05]
"""
import argparse, math, os, sys
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; from matplotlib.patches import Polygon as MplPoly
from qcnet_model import QCNetCarla, CarlaMapGraph, HIST, FUT, DT, to_av

ap = argparse.ArgumentParser(); ap.add_argument('--log', required=True); ap.add_argument('--out', required=True); ap.add_argument('--town', default='Town05')
ap.add_argument('--view', type=float, default=45); ap.add_argument('--radius', type=float, default=70); ap.add_argument('--stride', type=int, default=1)
a = ap.parse_args(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

z = np.load(a.log); rows = z['rows']; rift_dt = float(z['dt'])
sub = int(round(DT / rift_dt)); assert abs(sub * rift_dt - DT) < 1e-6, 'log must be at a multiple of 10 Hz'
ids = np.unique(rows[:, 1]).astype(int); id_to_j = {i: j for j, i in enumerate(ids)}
ticks = np.unique(rows[:, 0]).astype(int); ticks = ticks[ticks % sub == 0]; t_to_k = {t: k for k, t in enumerate(ticks)}
T, N = len(ticks), len(ids)
log = np.full((T, N, 5), np.nan, np.float32); role = np.zeros((T, N), np.int8); ext = np.zeros((N, 2))
for r in rows:
    t = int(r[0])
    if t not in t_to_k: continue
    k, j = t_to_k[t], id_to_j[int(r[1])]; x, y, h = to_av(r[2], r[3], r[4]); log[k, j] = (x, y, h, r[5], -r[6]); role[k, j] = int(r[9]); ext[j] = (r[7], r[8])
ego_j = int(np.argmax((role == 1).sum(0)))
plans = {}
for pt, pid, px, py in zip(z['plan_tick'], z['plan_id'], z['plan_x'], z['plan_y']):
    if int(pt) in t_to_k: plans[(t_to_k[int(pt)], id_to_j.get(int(pid), -1))] = np.stack([px, py], -1)   # already Argoverse frame (RIFT policy frame)
print(f'log: {T} steps at 10 Hz, {N} vehicles, ego index {ego_j}, {len(plans)} policy plans', flush=True)

client = carla.Client('localhost', 2000); client.set_timeout(180.0); w = client.get_world()
if a.town not in w.get_map().name: w = client.load_world(a.town)
graph = CarlaMapGraph(w.get_map()); model = QCNetCarla(); print('map + model ready', flush=True)
lane_lines = [(p['center'][:, :2], p['left'][:, :2], p['right'][:, :2], p['ptype']) for p in graph.polys]
Wn, Hn = 1100, 1100; writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), 10.0 / a.stride, (Wn, Hn))
fig = plt.figure(figsize=(11, 11), dpi=100); stats = {'policy': [], 'tm': []}; n = 0
for k in range(HIST - 1, T - FUT, a.stride):
    ex, ey = log[k, ego_j, :2]
    if np.isnan(ex): continue
    sel = [j for j in range(N) if not np.isnan(log[k, j, 0]) and np.hypot(log[k, j, 0] - ex, log[k, j, 1] - ey) < a.radius and (~np.isnan(log[k - HIST + 1:k + 1, j, 0])).sum() >= 10]
    if ego_j not in sel: sel.insert(0, ego_j)
    agents = [dict(id=int(ids[j]), type='vehicle', hist=log[k - HIST + 1:k + 1, j]) for j in sel]
    traj, pi, npoly = model.predict(agents, graph, sel.index(ego_j))
    gt = log[k + 1:k + 1 + FUT, :, :2].transpose(1, 0, 2)[sel]
    for i_, j in enumerate(sel):
        g = gt[i_]
        if np.isnan(g[:, 0]).any() or j == ego_j: continue
        err = np.linalg.norm(traj[i_] - g[None], axis=-1); stats['policy' if role[k, j] == 2 else 'tm'].append((err.mean(-1).min(), err[:, -1].min()))
    fig.clf(); ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor('#f4f4f2'); V = a.view
    for c, l, r, pt in lane_lines:
        if abs(c[0, 0] - ex) > V + 40 or abs(c[0, 1] - ey) > V + 40 or pt == 'PEDESTRIAN': continue
        ax.plot(c[:, 0], c[:, 1], color='#d0d0d0', lw=0.6); ax.plot(l[:, 0], l[:, 1], color='#b8b8b8', lw=0.5); ax.plot(r[:, 0], r[:, 1], color='#b8b8b8', lw=0.5)
    for i_, j in enumerate(sel):
        x, y, h = log[k, j, :3]; L, Wd = ext[j]; under = role[k, j] == 2
        col = '#d62728' if j == ego_j else ('#ff7f0e' if under else '#1f77b4')
        cs, sn = math.cos(h), math.sin(h); box = np.array([[L, Wd], [L, -Wd], [-L, -Wd], [-L, Wd]]); R = np.array([[cs, sn], [-sn, cs]])
        ax.add_patch(MplPoly(box @ R + [x, y], closed=True, fc=col, ec='k', lw=2.0 if under else 0.6, zorder=6))
        if under: ax.text(x, y + 2.8, 'RIFT', fontsize=8, ha='center', color='#b35900', weight='bold', zorder=8)
        hist = log[k - HIST + 1:k + 1, j, :2]; ax.plot(hist[:, 0], hist[:, 1], color='#555', lw=0.8, alpha=0.6, zorder=3)
        order = np.argsort(-pi[i_]); pmax = max(float(pi[i_].max()), 1e-6)
        for m_, mode in enumerate(order):
            p_ = float(pi[i_, mode]); al = 0.12 + 0.88 * (p_ / pmax); tr = traj[i_, mode]
            ax.plot(tr[:, 0], tr[:, 1], color='#e41a1c', lw=1.0 + 1.6 * (p_ / pmax), alpha=al, zorder=4 + (m_ == 0))
            hd = math.atan2(tr[-1, 1] - tr[-4, 1], tr[-1, 0] - tr[-4, 0]) if np.linalg.norm(tr[-1] - tr[-4]) > 0.3 else h
            gc, gs = math.cos(hd), math.sin(hd); ax.add_patch(MplPoly(box @ np.array([[gc, gs], [-gs, gc]]) + tr[-1], closed=True, fc='#e41a1c', ec='#8b0000', lw=0.5, alpha=0.75 * al, zorder=4))
            if m_ < 2 and j != ego_j: ax.text(tr[-1, 0], tr[-1, 1] + 1.4, f'{p_ * 100:.0f}%', fontsize=7, color='#8b0000', ha='center', alpha=min(1.0, al + 0.3), zorder=8)
        pl = plans.get((k, j))
        if pl is not None: ax.plot(pl[:, 0], pl[:, 1], color='#ff9900', lw=1.8, ls='--', zorder=5)
        g = gt[i_]
        if not np.isnan(g[:, 0]).any(): ax.plot(g[:, 0], g[:, 1], color='#2ca02c', lw=2.0, zorder=7); ax.plot(g[-1, 0], g[-1, 1], 'o', color='#2ca02c', ms=4, zorder=7)
    ax.set_xlim(ex - V, ex + V); ax.set_ylim(ey - V, ey + V); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    sp = np.mean(stats['policy'], 0) if stats['policy'] else (0, 0); stm = np.mean(stats['tm'], 0) if stats['tm'] else (0, 0)
    ax.text(0.01, 0.985, f'QCNet predicting a RIFT free-roam run ({a.town})   t = {k * DT:5.1f} s   {len(sel)} vehicles, {int((role[k] == 2).sum())} under the RIFT policy (orange, bold)', transform=ax.transAxes, va='top', fontsize=10.5, bbox=dict(fc='white', ec='none', alpha=0.85))
    ax.text(0.01, 0.955, f'green = actual next 6 s   red = predicted modes, opacity = probability   orange dashed = RIFT policy\'s own plan   minADE6/minFDE6: policy cars {sp[0]:.2f}/{sp[1]:.2f} m, TM cars {stm[0]:.2f}/{stm[1]:.2f} m', transform=ax.transAxes, va='top', fontsize=9.5, bbox=dict(fc='white', ec='none', alpha=0.85))
    fig.canvas.draw(); frame = cv2.resize(np.asarray(fig.canvas.buffer_rgba())[:, :, :3], (Wn, Hn)); writer.write(np.ascontiguousarray(frame[:, :, ::-1])); n += 1
writer.release()
sp = np.mean(stats['policy'], 0); stm = np.mean(stats['tm'], 0)
print(f'=== {n} frames -> {a.out}\n    RIFT-policy cars: minADE6 {sp[0]:.2f} m  minFDE6 {sp[1]:.2f} m  ({len(stats["policy"])} predictions)\n    Traffic-Manager cars: minADE6 {stm[0]:.2f} m  minFDE6 {stm[1]:.2f} m  ({len(stats["tm"])} predictions)', flush=True)
