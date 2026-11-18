"""Focused QCNet experiment: two scripted cars on a Town04 highway, nobody else.
  car A (front): cruises in its lane and drifts half a lane toward the neighbour lane and back on a slow sine
                 (an undecided lane change), period --a-period s, amplitude --a-amp m
  car B (rear):  weaves: full lane changes left / right every --b-period s with fast --b-ramp s transitions
QCNet predicts both at every 0.1 s from their last 5 s; the video shows all 6 modes with opacity = probability, a ghost car
at each 6 s endpoint, and the actual future in green. Motion is kinematic along the OpenDRIVE lane geometry, so the
ground truth is exact; the CARLA server is only needed for the map.

    ~/miniconda3/envs/qcnet/bin/python qcnet_carla/qcnet_focus.py --out selected_runs/qcnet/qcnet_focus_highway.mp4
"""
import argparse, math, os, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; from matplotlib.patches import Polygon as MplPoly
from qcnet_model import QCNetCarla, CarlaMapGraph, HIST, FUT, DT, to_av

ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--town', default='Town04')
ap.add_argument('--seconds', type=float, default=38); ap.add_argument('--view', type=float, default=60)
ap.add_argument('--a-speed', type=float, default=17.0); ap.add_argument('--a-amp', type=float, default=1.6); ap.add_argument('--a-period', type=float, default=8.0)
ap.add_argument('--b-speed', type=float, default=18.0); ap.add_argument('--b-period', type=float, default=3.0); ap.add_argument('--b-ramp', type=float, default=1.0)
ap.add_argument('--gap', type=float, default=30.0, help='initial distance of B behind A [m]'); ap.add_argument('--lane-w', type=float, default=3.5)
a = ap.parse_args(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)

client = carla.Client('localhost', 2000); client.set_timeout(180.0)
w = client.get_world()
if a.town not in w.get_map().name: w = client.load_world(a.town)
cmap = w.get_map()

# ------------------------------------------------------------------ a long straight multi-lane stretch: the lane whose first 400 m bend the least
best = None
need = int(60 + a.gap + max(a.a_speed, a.b_speed) * a.seconds + 20)      # metres the scripted cars will cover
for sp in cmap.get_spawn_points():
    wp = cmap.get_waypoint(sp.location)
    ok_nb = all(nb is not None and nb.lane_type == carla.LaneType.Driving and nb.lane_id * wp.lane_id > 0 for nb in (wp.get_left_lane(), wp.get_right_lane()))
    if not ok_nb: continue                                                   # B needs a lane on each side to weave into
    pts = [wp]; ok = True
    for _ in range(need):
        nxt = pts[-1].next(1.0)
        if not nxt: ok = False; break
        same = [q for q in nxt if q.road_id == pts[-1].road_id]
        pts.append(same[0] if same else nxt[0])
    if not ok: continue
    yaws = np.unwrap(np.radians([p.transform.rotation.yaw for p in pts])); bend = float(np.abs(np.diff(yaws)).sum())
    if best is None or bend < best[2]: best = (1.0 / (1.0 + bend), pts, bend)
path_wps = best[1]
P = np.array([to_av(p.transform.location.x, p.transform.location.y, p.transform.rotation.yaw) for p in path_wps])   # x, y, heading (Argoverse frame)
S = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))])
print(f'{a.town}: highway stretch, total turning {math.degrees(best[2]):.0f} deg over {S[-1]:.0f} m, lane width {path_wps[0].lane_width:.1f} m', flush=True)

def along(s, lateral):
    """position/heading at arc length s with a lateral offset (positive = left in the Argoverse frame)."""
    s = np.clip(s, 0, S[-1] - 1e-3); i = np.searchsorted(S, s) - 1; i = np.clip(i, 0, len(S) - 2); f = (s - S[i]) / (S[i + 1] - S[i])
    x = P[i, 0] + f * (P[i + 1, 0] - P[i, 0]); y = P[i, 1] + f * (P[i + 1, 1] - P[i, 1]); h = P[i, 2]
    return np.array([x - math.sin(h) * lateral, y + math.cos(h) * lateral]), h

def weave(t):
    """B's lateral offset: left lane for b_period s, back for b_period s, right lane, back ... with linear ramps."""
    seq = [0.0, a.lane_w, 0.0, -a.lane_w]; k = int(t // a.b_period); cur, nxt = seq[k % 4], seq[(k + 1) % 4]
    f = min(1.0, (t - k * a.b_period) / a.b_ramp); return cur + (nxt - cur) * f

T = int(a.seconds / DT); log = np.full((T, 2, 5), np.nan, np.float32)
for k in range(T):
    t = k * DT
    pa, ha = along(a.gap + 60 + a.a_speed * t, a.a_amp * math.sin(2 * math.pi * t / a.a_period))
    pb, hb = along(60 + a.b_speed * t, weave(t))
    log[k, 0, :2] = pa; log[k, 1, :2] = pb
# velocities and headings from the actual motion (what a tracker would measure)
for j in range(2):
    v = np.gradient(log[:, j, :2], DT, axis=0); log[:, j, 3:5] = v; log[:, j, 2] = np.arctan2(v[:, 1], v[:, 0])
ext = np.array([[2.4, 1.0], [2.3, 0.95]]); names = ['A front: drifting half-lane', 'B rear: weaving lanes']

graph = CarlaMapGraph(cmap); model = QCNetCarla()
lane_lines = [(p['center'][:, :2], p['left'][:, :2], p['right'][:, :2], p['ptype']) for p in graph.polys]
Wn, Hn = 1100, 1100; writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), 10.0, (Wn, Hn))
fig = plt.figure(figsize=(11, 11), dpi=100); stats = {0: [], 1: []}; top1 = {0: [], 1: []}; n = 0
for k in range(HIST - 1, T - FUT):
    agents = [dict(id=j, type='vehicle', hist=log[k - HIST + 1:k + 1, j]) for j in range(2)]
    traj, pi, npoly = model.predict(agents, graph, 1)
    gt = log[k + 1:k + 1 + FUT, :, :2].transpose(1, 0, 2)
    for j in range(2):
        err = np.linalg.norm(traj[j] - gt[j][None], axis=-1); stats[j].append((err.mean(-1).min(), err[:, -1].min())); top1[j].append(err[int(np.argmax(pi[j])), -1])
    cx, cy = (log[k, 0, :2] + log[k, 1, :2]) / 2; V = a.view
    fig.clf(); ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor('#f4f4f2')
    for c, l, r, pt in lane_lines:
        if abs(c[0, 0] - cx) > V + 60 or abs(c[0, 1] - cy) > V + 60 or pt == 'PEDESTRIAN': continue
        ax.plot(c[:, 0], c[:, 1], color='#d0d0d0', lw=0.6); ax.plot(l[:, 0], l[:, 1], color='#b0b0b0', lw=0.6); ax.plot(r[:, 0], r[:, 1], color='#b0b0b0', lw=0.6)
    for j in range(2):
        x, y, h = log[k, j, :3]; L, Wd = ext[j]; col = '#1f77b4' if j == 0 else '#d62728'
        cs, sn = math.cos(h), math.sin(h); R = np.array([[cs, sn], [-sn, cs]]); box = np.array([[L, Wd], [L, -Wd], [-L, -Wd], [-L, Wd]])
        ax.add_patch(MplPoly(box @ R + [x, y], closed=True, fc=col, ec='k', lw=0.8, zorder=6)); ax.text(x, y - 3.2, names[j], fontsize=9, ha='center', color=col, weight='bold', zorder=8)
        hist = log[k - HIST + 1:k + 1, j, :2]; ax.plot(hist[:, 0], hist[:, 1], color='#555', lw=1.0, alpha=0.6, zorder=3)
        order = np.argsort(-pi[j]); pmax = max(float(pi[j].max()), 1e-6)
        for m_, mode in enumerate(order):
            p_ = float(pi[j, mode]); al = 0.12 + 0.88 * (p_ / pmax); tr = traj[j, mode]
            ax.plot(tr[:, 0], tr[:, 1], color='#e41a1c', lw=1.0 + 1.8 * (p_ / pmax), alpha=al, zorder=4 + (m_ == 0))
            hd = math.atan2(tr[-1, 1] - tr[-4, 1], tr[-1, 0] - tr[-4, 0]); gc, gs = math.cos(hd), math.sin(hd)
            ax.add_patch(MplPoly(box @ np.array([[gc, gs], [-gs, gc]]) + tr[-1], closed=True, fc='#e41a1c', ec='#8b0000', lw=0.5, alpha=0.75 * al, zorder=4))
            if m_ < 3: ax.text(tr[-1, 0] + 2.6, tr[-1, 1], f'{p_ * 100:.0f}%', fontsize=8, color='#8b0000', va='center', alpha=min(1.0, al + 0.3), zorder=8)
        g = gt[j]; ax.plot(g[:, 0], g[:, 1], color='#2ca02c', lw=2.2, zorder=7); ax.plot(g[-1, 0], g[-1, 1], 'o', color='#2ca02c', ms=5, zorder=7)
    ax.set_xlim(cx - V, cx + V); ax.set_ylim(cy - V, cy + V); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    sA, sB = np.mean(stats[0], 0), np.mean(stats[1], 0)
    ax.text(0.01, 0.985, f'QCNet on a scripted {a.town} highway   t = {k * DT:5.1f} s   A: drift +-{a.a_amp} m every {a.a_period:.0f} s at {a.a_speed * 3.6:.0f} km/h   B: lane change every {a.b_period:.0f} s in {a.b_ramp} s at {a.b_speed * 3.6:.0f} km/h',
            transform=ax.transAxes, va='top', fontsize=10.5, bbox=dict(fc='white', ec='none', alpha=0.85))
    ax.text(0.01, 0.955, f'green = actual next 6 s   red = 6 predicted modes, opacity = probability, ghost car at each endpoint   running minADE6/minFDE6: A {sA[0]:.2f}/{sA[1]:.2f} m   B {sB[0]:.2f}/{sB[1]:.2f} m   top-1 FDE: A {np.mean(top1[0]):.2f}  B {np.mean(top1[1]):.2f} m',
            transform=ax.transAxes, va='top', fontsize=9.5, bbox=dict(fc='white', ec='none', alpha=0.85))
    fig.canvas.draw(); frame = cv2.resize(np.asarray(fig.canvas.buffer_rgba())[:, :, :3], (Wn, Hn)); writer.write(np.ascontiguousarray(frame[:, :, ::-1])); n += 1
writer.release()
sA, sB = np.mean(stats[0], 0), np.mean(stats[1], 0)
print(f'=== {n} frames -> {a.out}\n    A (drifting):  minADE6 {sA[0]:.2f} m  minFDE6 {sA[1]:.2f} m  top-1 FDE {np.mean(top1[0]):.2f} m\n    B (weaving):   minADE6 {sB[0]:.2f} m  minFDE6 {sB[1]:.2f} m  top-1 FDE {np.mean(top1[1]):.2f} m', flush=True)
