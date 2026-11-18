"""Trajectory prediction with QCNet on CARLA traffic: run normal Traffic-Manager traffic and pedestrians in a city town,
log every agent at 10 Hz, then predict every agent's next 6 s from its last 5 s and the map, and render a bird's-eye
video with the ground-truth future (what the agent actually did next) against the predicted modes.

    ~/miniconda3/envs/qcnet/bin/python qcnet_carla/qcnet_carla.py --out selected_runs/qcnet/qcnet_town10.mp4 [--town Town10HD]
        [--vehicles 40 --walkers 30 --seconds 80 --view 60 --stride 1]

Needs a CARLA server on :2000. Prediction is offline on the log, so the video is one frame per simulated 0.1 s.
"""
import argparse, math, os, random, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla, numpy as np, cv2
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt; from matplotlib.patches import Polygon as MplPoly
from qcnet_model import QCNetCarla, CarlaMapGraph, HIST, FUT, DT, to_av

ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True); ap.add_argument('--town', default='Town10HD')
ap.add_argument('--vehicles', type=int, default=40); ap.add_argument('--walkers', type=int, default=30); ap.add_argument('--seconds', type=float, default=80)
ap.add_argument('--seed', type=int, default=1); ap.add_argument('--view', type=float, default=60, help='half-width of the bird\'s-eye window [m]')
ap.add_argument('--stride', type=int, default=1, help='predict every N log steps (0.1 s each)'); ap.add_argument('--agent-radius', type=float, default=80)
ap.add_argument('--modes', type=int, default=6, help='how many predicted modes to draw per agent')
a = ap.parse_args(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True); random.seed(a.seed); np.random.seed(a.seed)

# ------------------------------------------------------------------ simulate and log
client = carla.Client('localhost', 2000); client.set_timeout(180.0)
w = client.get_world()
if a.town not in w.get_map().name: w = client.load_world(a.town)
for v in w.get_actors().filter('*vehicle*'): v.destroy()
for v in w.get_actors().filter('*walker*'): v.destroy()
st = w.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = DT; w.apply_settings(st); w.set_weather(carla.WeatherParameters.ClearNoon); w.tick()
cmap = w.get_map(); lib = w.get_blueprint_library(); tm = client.get_trafficmanager(8000); tm.set_synchronous_mode(True); tm.set_random_device_seed(a.seed)
spawns = cmap.get_spawn_points(); random.shuffle(spawns)
eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '200,20,20')
ego = None
for sp in spawns:
    ego = w.try_spawn_actor(eb, sp)
    if ego: break
cars = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4 and not any(k in b.id for k in ('truck', 'bus', 'fusorosa', 'carlacola', 'firetruck', 'ambulance', 'sprinter', 'cybertruck'))]
bikes = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 2]
actors = {ego.id: ('vehicle', ego)}
for sp in spawns[1:]:
    if len(actors) > a.vehicles: break
    if sp.location.distance(ego.get_location()) < 8: continue
    bp = random.choice(cars) if random.random() > 0.1 else random.choice(bikes)
    v = w.try_spawn_actor(bp, sp)
    if v: v.set_autopilot(True, 8000); tm.vehicle_percentage_speed_difference(v, float(random.uniform(-15, 10))); actors[v.id] = ('motorcyclist' if int(bp.get_attribute('number_of_wheels')) == 2 else 'vehicle', v)
ego.set_autopilot(True, 8000)
wbps = lib.filter('walker.pedestrian.*'); cbp = lib.find('controller.ai.walker'); walkers, ctrls = [], []
for _ in range(a.walkers):
    loc = w.get_random_location_from_navigation()
    if loc is None: continue
    wk = w.try_spawn_actor(random.choice(wbps), carla.Transform(loc))
    if wk: walkers.append(wk); actors[wk.id] = ('pedestrian', wk)
w.tick()
for wk in walkers: ctrls.append(w.spawn_actor(cbp, carla.Transform(), attach_to=wk))
w.tick(); w.set_pedestrians_cross_factor(0.3)
for c in ctrls: c.start(); c.go_to_location(w.get_random_location_from_navigation()); c.set_max_speed(1.0 + random.random())
print(f'{a.town}: {sum(1 for t, _ in actors.values() if t != "pedestrian")} vehicles + {len(walkers)} pedestrians', flush=True)
for _ in range(30): w.tick()

ids = list(actors.keys()); types = [actors[i][0] for i in ids]; ext = np.array([[actors[i][1].bounding_box.extent.x, actors[i][1].bounding_box.extent.y] for i in ids])
T = int(a.seconds / DT); log = np.full((T, len(ids), 5), np.nan, np.float32)   # x, y, heading (Argoverse frame), vx, vy
t0 = time.time()
for k in range(T):
    w.tick()
    for j, i in enumerate(ids):
        act = actors[i][1]
        if not act.is_alive: continue
        tf = act.get_transform(); v = act.get_velocity(); x, y, h = to_av(tf.location.x, tf.location.y, tf.rotation.yaw)
        log[k, j] = (x, y, h, v.x, -v.y)
print(f'logged {T} steps ({a.seconds:.0f} s) in {time.time() - t0:.0f} s', flush=True)
graph = CarlaMapGraph(cmap); print(f'map graph: {len(graph.polys)} polygons', flush=True)
for c in ctrls:
    try: c.stop(); c.destroy()
    except Exception: pass
st = w.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; w.apply_settings(st)

# ------------------------------------------------------------------ predict + render
model = QCNetCarla(); print('QCNet loaded', flush=True)
ego_j = ids.index(ego.id)
lane_lines = [(p['center'][:, :2], p['left'][:, :2], p['right'][:, :2], p['ptype']) for p in graph.polys]
Wn, Hn = 1100, 1100; writer = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), 10.0 / a.stride, (Wn, Hn))
ade_all, fde_all, n_frames, lat = [], [], 0, []
fig = plt.figure(figsize=(11, 11), dpi=100)
for k in range(HIST - 1, T - FUT, a.stride):
    ex, ey = log[k, ego_j, :2]
    if np.isnan(ex): continue
    # agents observed now, within the radius, with at least 1 s of history
    sel = [j for j in range(len(ids)) if not np.isnan(log[k, j, 0]) and np.hypot(log[k, j, 0] - ex, log[k, j, 1] - ey) < a.agent_radius and (~np.isnan(log[k - HIST + 1:k + 1, j, 0])).sum() >= 10]
    if ego_j not in sel: sel.insert(0, ego_j)
    agents = [dict(id=ids[j], type=types[j], hist=log[k - HIST + 1:k + 1, j]) for j in sel]
    t1 = time.time(); traj, pi, npoly = model.predict(agents, graph, sel.index(ego_j)); lat.append(time.time() - t1)
    gt = log[k + 1:k + 1 + FUT, :, :2].transpose(1, 0, 2)[sel]            # (n, FUT, 2)
    # metrics (agents with full GT future)
    for i_, j in enumerate(sel):
        g = gt[i_]
        if np.isnan(g[:, 0]).any(): continue
        err = np.linalg.norm(traj[i_] - g[None], axis=-1)                  # (K, FUT)
        ade_all.append(err.mean(-1).min()); fde_all.append(err[:, -1].min())
    # render
    fig.clf(); ax = fig.add_axes([0, 0, 1, 1]); ax.set_facecolor('#f4f4f2')
    V = a.view
    for c, l, r, pt in lane_lines:
        if abs(c[0, 0] - ex) > V + 40 or abs(c[0, 1] - ey) > V + 40: continue
        if pt == 'PEDESTRIAN': ax.fill(np.concatenate([l[:, 0], r[::-1, 0]]), np.concatenate([l[:, 1], r[::-1, 1]]), color='#cfe3f5', alpha=0.5, lw=0)
        else:
            ax.plot(c[:, 0], c[:, 1], color='#d0d0d0', lw=0.6); ax.plot(l[:, 0], l[:, 1], color='#b8b8b8', lw=0.5); ax.plot(r[:, 0], r[:, 1], color='#b8b8b8', lw=0.5)
    for i_, j in enumerate(sel):
        x, y, h = log[k, j, :3]; L, Wd = ext[j]
        col = '#d62728' if j == ego_j else ('#ff7f0e' if types[j] == 'pedestrian' else ('#9467bd' if types[j] == 'motorcyclist' else '#1f77b4'))
        cs, sn = math.cos(h), math.sin(h); corners = np.array([[L, Wd], [L, -Wd], [-L, -Wd], [-L, Wd]]) @ np.array([[cs, sn], [-sn, cs]]) + [x, y]
        ax.add_patch(MplPoly(corners, closed=True, fc=col, ec='k', lw=0.6, alpha=0.9, zorder=5))
        hist = log[k - HIST + 1:k + 1, j, :2]; ax.plot(hist[:, 0], hist[:, 1], color='#555', lw=0.8, alpha=0.6, zorder=3)
        order = np.argsort(-pi[i_])
        for m_, mode in enumerate(order[:a.modes]):
            ax.plot(traj[i_, mode, :, 0], traj[i_, mode, :, 1], color='#e41a1c', lw=2.2 if m_ == 0 else 0.9, alpha=0.95 if m_ == 0 else 0.35, zorder=6 if m_ == 0 else 4)
        g = gt[i_]
        if not np.isnan(g[:, 0]).any(): ax.plot(g[:, 0], g[:, 1], color='#2ca02c', lw=2.0, zorder=7); ax.plot(g[-1, 0], g[-1, 1], 'o', color='#2ca02c', ms=4, zorder=7)
    ax.set_xlim(ex - V, ex + V); ax.set_ylim(ey - V, ey + V); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    ax.text(0.01, 0.985, f'QCNet (Argoverse 2 weights) on CARLA {a.town}   t = {k * DT:5.1f} s   {len(sel)} agents, {npoly} map polygons   inference {np.mean(lat[-10:]) * 1000:.0f} ms', transform=ax.transAxes, va='top', fontsize=11, bbox=dict(fc='white', ec='none', alpha=0.85))
    ax.text(0.01, 0.955, f'green = actual next 6 s   red thick = most likely predicted mode   red thin = other modes   grey = last 5 s   running minADE6 {np.mean(ade_all) if ade_all else 0:.2f} m  minFDE6 {np.mean(fde_all) if fde_all else 0:.2f} m', transform=ax.transAxes, va='top', fontsize=10, bbox=dict(fc='white', ec='none', alpha=0.85))
    fig.canvas.draw(); frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
    frame = cv2.resize(frame, (Wn, Hn)); writer.write(np.ascontiguousarray(frame[:, :, ::-1])); n_frames += 1
    if n_frames % 50 == 0: print(f'  frame {n_frames}  t={k * DT:.1f}s  agents {len(sel)}  minADE6 {np.mean(ade_all):.2f}  minFDE6 {np.mean(fde_all):.2f}  {np.mean(lat) * 1000:.0f} ms', flush=True)
writer.release()
print(f'=== {n_frames} frames -> {a.out}; minADE6 {np.mean(ade_all):.2f} m, minFDE6 {np.mean(fde_all):.2f} m over {len(ade_all)} agent-predictions; inference {np.mean(lat) * 1000:.0f} ms/frame', flush=True)
