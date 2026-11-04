"""CTG++ drives the NPCs around you in CARLA (language/loss-guided scene-level diffusion traffic model, NVlabs/CTG).

    ~/miniconda3/envs/ctg/bin/python ctg_carla/ctg_carla.py [--town Town05] [--ego manual|autopilot] [--npcs 24]
        [--control 8] [--guidance safe|trouble|cutin|tailgate|none|FILE.json] [--samples 5] [--stride 1] [--test SECONDS]

Every 0.5 s the ego plus the --control nearest moving vehicles (within 50 m) form a scene; CTG++ generates a joint 5.2 s
trajectory for all of them, steered by the guidance losses (collision avoidance, stay on road, "make NPC k hit the ego",
tailgating, ...), and the controlled NPCs follow their trajectory for the next 0.5 s (velocity tracking with physics on). Vehicles outside the scene keep driving with the Traffic Manager; the ego's own predicted
trajectory is discarded because YOU drive it (W/S A/D, SHIFT gentle, Q reverse, SPACE handbrake, TAB camera, ESC quit).
Guidance indices: agent 0 is always the ego, 1..K are the controlled NPCs sorted by distance (1 = nearest).
Needs a CARLA server on :2000 (bash carla_server.sh). Model: CTG/ckpt/ctg++8_9,10edge (CC-BY-NC-SA-4.0).
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import carla  # noqa: E402

from ctg_model import CTGPlusPlus, LaneGraph, DrivableRaster, HIST_T, FUT_T, DT, PIXEL_SIZE  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--port', type=int, default=2000); ap.add_argument('--town', default='Town05')
ap.add_argument('--ego', default='manual', choices=['manual', 'autopilot'])
ap.add_argument('--npcs', type=int, default=24, help='vehicles spawned near the ego (Traffic Manager when not controlled)')
ap.add_argument('--control', type=int, default=8, help='max NPCs handed to CTG++ at once')
ap.add_argument('--radius', type=float, default=60.0, help='scene radius around the ego (m)')
ap.add_argument('--guidance', default='trouble', help='safe | trouble | cutin | tailgate | none | path to a JSON list')
ap.add_argument('--trouble-weight', type=float, default=1.0)
ap.add_argument('--samples', type=int, default=5, help='diffusion samples per plan; the guidance picks the best one')
ap.add_argument('--stride', type=int, default=1, help='diffusion step stride (4 = 25 denoising steps instead of 100)')
ap.add_argument('--replan', type=int, default=5, help='model steps (0.1 s) between plans')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--test', type=float, default=0.0, help='headless: ego on autopilot for N seconds, then print stats')
ap.add_argument('--record', default='', help='write the window to this mp4')
ap.add_argument('--snapshot', default='', help='save window frames to this prefix every 5 s')
a = ap.parse_args()
rng = np.random.default_rng(a.seed)
if a.test: os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
import pygame  # noqa: E402

# ------------------------------------------------------------------ frames: CARLA (left-handed, y right) <-> model (y left)
def to_model(loc_x, loc_y, yaw_deg): return loc_x, -loc_y, -math.radians(yaw_deg)
def to_carla(x, y, yaw): return x, -y, -math.degrees(yaw)

# ------------------------------------------------------------------ world
client = carla.Client('localhost', a.port); client.set_timeout(120.0)
world = client.get_world()
if a.town and a.town not in world.get_map().name:
    print(f'loading {a.town} ...', flush=True); world = client.load_world(a.town)
cmap = world.get_map(); lib = world.get_blueprint_library()
st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = DT; world.apply_settings(st)
tm = client.get_trafficmanager(a.port + 6000); tm.set_synchronous_mode(True); tm.set_random_device_seed(a.seed)
tm.global_percentage_speed_difference(-10.0)
for act in world.get_actors().filter('vehicle.*'): act.destroy()
world.tick()

# ------------------------------------------------------------------ map -> lane graph + drivable raster (model frame)
t0 = time.time()
wps = cmap.generate_waypoints(1.0); lanes = {}
for wp in wps:
    if wp.lane_type != carla.LaneType.Driving: continue
    lanes.setdefault((wp.road_id, wp.section_id, wp.lane_id), []).append(wp)
polys = []
for key, group in lanes.items():
    group.sort(key=lambda w: w.s if w.lane_id < 0 else -w.s)   # CARLA: negative lane ids run along +s, positive against it
    pts = []
    for w in group:
        x, y, h = to_model(w.transform.location.x, w.transform.location.y, w.transform.rotation.yaw)
        pts.append((x, y, w.transform.location.z, h))
    polys.append(np.array(pts))
lane_graph = LaneGraph(polys)
allpts = np.concatenate([p[:, :2] for p in polys]); lo = allpts.min(0) - 60; hi = allpts.max(0) + 60
W_px, H_px = int((hi[0] - lo[0]) / PIXEL_SIZE), int((hi[1] - lo[1]) / PIXEL_SIZE)
import cv2  # noqa: E402
driv = np.zeros((H_px, W_px), np.uint8)
for wp in wps:
    if wp.lane_type != carla.LaneType.Driving: continue
    x, y, _ = to_model(wp.transform.location.x, wp.transform.location.y, 0)
    cv2.circle(driv, (int((x - lo[0]) / PIXEL_SIZE), int((y - lo[1]) / PIXEL_SIZE)), max(1, int(wp.lane_width / 2 / PIXEL_SIZE)), 1, -1)
drivable = DrivableRaster(driv, origin_xy=lo)
print(f'map: {len(polys)} lanes, drivable raster {W_px}x{H_px} px, built in {time.time() - t0:.1f}s', flush=True)

# ------------------------------------------------------------------ actors
car_bps = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4
           and not any(k in b.id for k in ('carlacola', 'firetruck', 'ambulance', 'sprinter', 'truck', 'bus', 'fusorosa', 'cybertruck', 't2', 'isetta', 'microlino'))]
import random as _random
spawns = cmap.get_spawn_points(); _random.Random(a.seed).shuffle(spawns)
eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '10,10,10')
ego = None
for sp in spawns:
    ego = world.try_spawn_actor(eb, sp)
    if ego: break
assert ego is not None, 'ego spawn failed'
npcs = []
def lane_slots(center_wp, dist_min, dist_max, step=4.0):
    """Driving-lane waypoints dist_min..dist_max metres ahead of AND behind center_wp (all branches, all parallel lanes)."""
    out = []
    for direction in ('next', 'previous'):
        frontier = [center_wp]; d = 0.0
        while d < dist_max and frontier:
            d += step; nxt = []
            for w in frontier: nxt += (w.next(step) if direction == 'next' else w.previous(step))
            frontier = nxt[:6]
            if d < dist_min: continue
            for w in frontier:
                for ww in (w, w.get_left_lane(), w.get_right_lane()):
                    if ww is not None and ww.lane_type == carla.LaneType.Driving and not ww.is_junction: out.append(ww)
    return out
def clear_of_vehicles(loc, margin=9.0):
    return all(v.get_location().distance(loc) > margin for v in [ego] + npcs if v.is_alive)
def spawn_npc(wp):
    bp = car_bps[int(rng.integers(len(car_bps)))]
    if bp.has_attribute('color'): cols = bp.get_attribute('color').recommended_values; bp.set_attribute('color', cols[int(rng.integers(len(cols)))])
    bp.set_attribute('role_name', 'npc'); tf = wp.transform; tf.location.z += 0.3
    v = world.try_spawn_actor(bp, tf)
    if v: v.set_autopilot(True, tm.get_port()); tm.vehicle_percentage_speed_difference(v, float(rng.uniform(-20, 10))); npcs.append(v)
    return v
ego_wp = cmap.get_waypoint(ego.get_location())
slots = lane_slots(ego_wp, 15.0, 110.0); rng.shuffle(slots)
for wp in slots:
    if len(npcs) >= a.npcs: break
    if clear_of_vehicles(wp.transform.location): spawn_npc(wp)
def recycle_far_npcs(max_dist=90.0):
    """Traffic lives around the ego: a Traffic-Manager NPC that fell further than max_dist behind is moved to a free
    lane slot 40-100 m ahead of the ego (facing the lane direction) so the scene never runs dry."""
    el = ego.get_location(); far = [v for v in npcs if v.is_alive and v.id not in controlled and v.get_location().distance(el) > max_dist]
    if not far: return
    slots = lane_slots(cmap.get_waypoint(el), 30.0, 80.0); rng.shuffle(slots)
    for v in far:
        for wp in slots:
            if clear_of_vehicles(wp.transform.location):
                tf = wp.transform; tf.location.z += 0.3; v.set_transform(tf); v.set_target_velocity(carla.Vector3D(0, 0, 0)); hist.pop(v.id, None); break
if a.ego == 'autopilot' or a.test: ego.set_autopilot(True, tm.get_port())
for _ in range(5): world.tick()
print(f'ego + {len(npcs)} NPCs spawned; ego mode {"autopilot" if (a.ego == "autopilot" or a.test) else "manual"}', flush=True)
collisions = []
cs = world.spawn_actor(lib.find('sensor.other.collision'), carla.Transform(), attach_to=ego)
def _on_collision(e):   # one event per contact: same actor again within 2 s = the same crash (continuous contact spams the sensor)
    t = world.get_snapshot().timestamp.elapsed_seconds
    if collisions and collisions[-1][2] == e.other_actor.id and t - collisions[-1][0] < 2.0: collisions[-1] = (t, collisions[-1][1], e.other_actor.id); return
    collisions.append((t, e.other_actor.type_id, e.other_actor.id))
cs.listen(_on_collision)

# ------------------------------------------------------------------ history buffers (model frame), 0.1 s per entry
hist = {}
controlled = {}          # actor id -> dict(actor, plan_world (T,3), k)
def extent_of(v): e = v.bounding_box.extent; return (2 * e.x, 2 * e.y, 2 * e.z)
def record_states():
    for v in [ego] + npcs:
        if not v.is_alive: continue
        tf = v.get_transform(); vel = v.get_velocity()
        x, y, yaw = to_model(tf.location.x, tf.location.y, tf.rotation.yaw)
        buf = hist.setdefault(v.id, deque(maxlen=HIST_T)); vx, vy = vel.x, -vel.y
        buf.append((x, y, yaw, vx, vy))
def hist_array(v):
    buf = hist.get(v.id, []); arr = np.full((HIST_T, 5), np.nan)
    if buf: arr[-len(buf):] = np.array(buf)
    return arr

# ------------------------------------------------------------------ model
print('loading CTG++ ...', flush=True); t0 = time.time()
model = CTGPlusPlus(num_samples=a.samples, diffusion_stride=a.stride)
print(f'model ready in {time.time() - t0:.1f}s (samples {a.samples}, stride {a.stride})', flush=True)

def guidance_list(K, nearest_ahead):
    """Guidance for a scene with ego at 0 and K NPCs at 1..K. nearest_ahead = index of the closest NPC ahead (or None)."""
    safe = [dict(name='agent_collision', weight=1.0, params=dict(num_disks=5, buffer_dist=0.2, decay_rate=0.9, guide_moving_speed_th=0.5, excluded_agents=None), agents=None),
            dict(name='map_collision', weight=1.0, params=dict(num_points_lw=(10, 10), decay_rate=0.9, guide_moving_speed_th=0.5), agents=None)]
    if a.guidance == 'none': return None
    if a.guidance == 'safe': return safe
    if a.guidance in ('trouble', 'cutin', 'tailgate'):
        k = nearest_ahead if nearest_ahead is not None else (1 if K >= 1 else None)
        if k is None: return safe
        if a.guidance == 'trouble':   # the chosen NPC is pulled onto the ego's future path, the others stay civil
            g = [dict(name='gptcollision', weight=a.trouble_weight, params=dict(target_ind=k, ref_ind=0, collision_radius=1.0), agents=None)]
            return g + [dict(name='agent_collision', weight=1.0, params=dict(num_disks=5, buffer_dist=0.2, decay_rate=0.9, guide_moving_speed_th=0.5, excluded_agents=[0, k]), agents=None), safe[1]]
        if a.guidance == 'cutin':     # hits the ego from the front = cuts in and brakes
            return [dict(name='front_collision', weight=a.trouble_weight, params=dict(target_ind=k, ref_ind=0), agents=None), safe[1]]
        if a.guidance == 'tailgate':  # stays 3-8 m from the ego
            return [dict(name='keep_distance', weight=a.trouble_weight, params=dict(target_ind=k, ref_ind=0, min_distance=3, max_distance=8), agents=None)] + safe
    cfg = json.load(open(a.guidance))   # custom: list of {name, weight, params, agents}; "ego" -> 0, "npc1" -> 1 ...
    def res(v): return 0 if v == 'ego' else int(v[3:]) if isinstance(v, str) and v.startswith('npc') else v
    for g in cfg:
        for pk in ('target_ind', 'ref_ind'):
            if pk in g.get('params', {}): g['params'][pk] = res(g['params'][pk])
    return cfg

# ------------------------------------------------------------------ control
def release(v):
    try: v.set_autopilot(True, tm.get_port())
    except RuntimeError: pass
def take(v):   # physics stays ON: a physics-less car is an immovable wall and a contact with it launches the ego off the road
    try: v.set_autopilot(False, tm.get_port())
    except RuntimeError: pass

def select_scene():
    el = ego.get_location(); etf = ego.get_transform(); fwd = etf.get_forward_vector()
    cands = []
    for v in npcs:
        if not v.is_alive: continue
        d = v.get_location().distance(el); h = hist.get(v.id, [])
        if d > a.radius or len(h) < HIST_T: continue
        if math.hypot(h[-1][3], h[-1][4]) < 0.6: continue   # stopped cars (red light, queue) stay with the TM: the model freezes stationary agents
        cands.append((d, v))
    cands.sort(key=lambda c: c[0]); chosen = [v for _, v in cands[:a.control]]
    ahead = None
    for i, v in enumerate(chosen):
        dv = v.get_location() - el
        if dv.x * fwd.x + dv.y * fwd.y > 3.0 and abs(-dv.x * fwd.y + dv.y * fwd.x) < 8.0: ahead = i + 1; break
    return chosen, ahead

plan_lat = []; n_plans = 0; last_scene = []; trouble_idx = None
def replan():
    global n_plans, last_scene, trouble_idx
    chosen, ahead = select_scene()
    for v in list(controlled):          # hand back the ones that left the scene
        if v not in [c.id for c in chosen]: release(controlled.pop(v)['actor'])
    if not chosen:
        if n_plans % 4 == 0: print(f'[tick {tick}] no moving NPC within {a.radius:.0f} m of the ego', flush=True)
        n_plans += 1; return
    agents = [dict(hist=hist_array(ego), extent=extent_of(ego))] + [dict(hist=hist_array(v), extent=extent_of(v)) for v in chosen]
    t0 = time.time(); batch = model.build_batch(agents, lane_graph, drivable)
    g = guidance_list(len(chosen), ahead); model.set_guidance(g, batch)
    out = model.plan(batch); lat = time.time() - t0; plan_lat.append(lat); n_plans += 1
    W = batch['world_from_agent'][0].cpu().numpy()
    for i, v in enumerate(chosen):
        p = out['positions'][i + 1]; y = out['yaws'][i + 1]; Wi = W[i + 1]
        pw = p @ Wi[:2, :2].T + Wi[:2, 2]; yw = y + math.atan2(Wi[1, 0], Wi[0, 0])
        if v.id not in controlled: take(v)
        controlled[v.id] = dict(actor=v, plan=np.concatenate([pw, yw[:, None]], -1), k=0)
    last_scene = chosen; trouble_idx = ahead
    if n_plans % 4 == 1:
        el = ego.get_location(); dmin = min([v.get_location().distance(el) for v in chosen], default=float('nan'))
        print(f'[plan {n_plans}] {len(chosen)} NPCs under CTG++, latency {lat:.2f}s, guidance={a.guidance}'
              f'{f" on npc{ahead}" if ahead and a.guidance in ("trouble", "cutin", "tailgate") else ""}, nearest {dmin:.1f} m, ego collisions {len(collisions)}', flush=True)

def apply_plans():
    """Controlled NPCs TRACK their plan with velocity commands (physics on): each tick the linear velocity that reaches the
    next plan point and the yaw rate that reaches its heading, both clipped. No teleporting: a teleported car carries an
    unbounded impulse and launches whatever it touches; here a contact is a real-mass collision at the planned speed."""
    el = ego.get_location()
    for c in list(controlled.values()):
        v, plan, k = c['actor'], c['plan'], c['k']
        if k + 1 >= len(plan) or not v.is_alive: continue
        tf = v.get_transform(); cx, cy, cyaw_deg = tf.location.x, tf.location.y, tf.rotation.yaw
        nx, ny, nyaw = (float(q) for q in plan[k + 1]); tx, ty, tyaw_deg = to_carla(nx, ny, nyaw)
        px, py, _ = (float(q) for q in plan[k]); plan_speed = math.hypot(nx - px, ny - py) / DT
        vx, vy = (tx - cx) / DT, (ty - cy) / DT; sp = math.hypot(vx, vy); vmax = max(plan_speed * 1.5, 1.0)
        if sp > vmax: vx, vy = vx / sp * vmax, vy / sp * vmax
        dyaw = (tyaw_deg - cyaw_deg + 180.0) % 360.0 - 180.0; wz = max(-120.0, min(120.0, dyaw / DT))
        v.set_target_velocity(carla.Vector3D(vx, vy, v.get_velocity().z)); v.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, wz))
        c['k'] = k + 1
        if v.id == (last_scene[trouble_idx - 1].id if (trouble_idx and trouble_idx - 1 < len(last_scene)) else -1): col = carla.Color(255, 40, 40)
        else: col = carla.Color(40, 120, 255)
        z = tf.location.z
        for j in range(k, min(len(plan), k + 20), 2):
            qx0, qy0, _ = (float(q) for q in plan[j]); qx, qy, _ = to_carla(qx0, qy0, 0)
            world.debug.draw_point(carla.Location(x=qx, y=qy, z=z + 0.3), size=0.08, color=col, life_time=DT * 1.5)

# ------------------------------------------------------------------ window
Wn, Hn = 1280, 720; img = {}
pygame.init(); screen = pygame.display.set_mode((Wn, Hn)); pygame.display.set_caption('CTG++ drives the NPCs'); font = pygame.font.SysFont('monospace', 17, bold=True)
cams = []; CAMS = [('chase', carla.Location(x=-7, z=3.2), carla.Rotation(pitch=-12)), ('top', carla.Location(z=45), carla.Rotation(pitch=-90)), ('bonnet', carla.Location(x=0.8, z=1.3), carla.Rotation())]
cam_idx = 0
def attach_cam():
    global cams
    for c in cams:
        try: c.stop(); c.destroy()
        except Exception: pass
    cb = lib.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(Wn)); cb.set_attribute('image_size_y', str(Hn)); cb.set_attribute('fov', '95')
    _, loc, rot = CAMS[cam_idx]; cam = world.spawn_actor(cb, carla.Transform(loc, rot), attach_to=ego)
    cam.listen(lambda im: img.__setitem__('cam', pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1))))
    cams = [cam]
attach_cam()
writer = None
if a.record:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v'); writer = cv2.VideoWriter(a.record, fourcc, 10.0, (Wn, Hn))

# ------------------------------------------------------------------ loop
steer = 0.0; ctl = carla.VehicleControl(); clock = pygame.time.Clock(); tick = 0; t_start = time.time(); ms_last = 100
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_TAB: cam_idx = (cam_idx + 1) % len(CAMS); attach_cam()
        keys = pygame.key.get_pressed(); vv = ego.get_velocity(); kmh = 3.6 * math.hypot(vv.x, vv.y)
        if a.ego == 'manual' and not a.test:
            gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
            ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
            inc = 4e-4 * ms_last; smax = 0.7 if kmh < 50 else max(0.25, 0.7 - (kmh - 50) * (0.45 / 70))
            if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-smax, steer - inc)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(smax, steer + inc)
            else: steer = 0.0
            ctl.steer = round(steer, 2); ego.apply_control(ctl)
        ms_last = clock.tick_busy_loop(10) if not a.test else 100
        world.tick(); tick += 1; record_states()
        if tick >= HIST_T and (tick - HIST_T) % a.replan == 0: replan()
        if tick % 20 == 0: recycle_far_npcs()
        apply_plans()
        screen.fill((15, 15, 15))
        if 'cam' in img: screen.blit(img['cam'], (0, 0))
        lat = np.mean(plan_lat[-5:]) if plan_lat else 0.0
        hud = [f'{kmh:5.1f} km/h  CTG++ NPCs {len(controlled)}/{len(npcs)}  plan {lat:.2f}s  guidance {a.guidance}  ego collisions {len(collisions)}',
               'W/S A/D  Shift gentle  Q reverse  Space handbrake  TAB camera  ESC quit   (red dots = the NPC guided towards you, blue = other CTG++ NPCs)']
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 230, 90)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0)); screen.blit(bg, (6, 6 + 22 * i)); screen.blit(lab, (10, 8 + 22 * i))
        pygame.display.flip()
        if writer is not None or (a.snapshot and tick % 50 == 0):
            frame = pygame.surfarray.array3d(screen).swapaxes(0, 1)[:, :, ::-1]
            if writer is not None: writer.write(np.ascontiguousarray(frame))
            if a.snapshot and tick % 50 == 0: cv2.imwrite(f'{a.snapshot}_{tick:05d}.png', frame)
        if a.test and tick * DT >= a.test: break
except KeyboardInterrupt: pass
except Exception:
    import traceback; traceback.print_exc()
finally:
    print(f'\n=== {tick * DT:.0f} s simulated in {time.time() - t_start:.0f} s wall; {n_plans} plans, mean latency {np.mean(plan_lat) if plan_lat else 0:.2f}s; '
          f'ego collisions: {len(collisions)} {[c[1] for c in collisions][:6]}', flush=True)
    if writer is not None: writer.release(); print('video:', a.record)
    for c in cams:
        try: c.stop(); c.destroy()
        except Exception: pass
    try: cs.stop(); cs.destroy()
    except Exception: pass
    for c in controlled.values(): release(c['actor'])
    try:
        tm.set_synchronous_mode(False); st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
    except Exception: pass
    pygame.quit(); os._exit(0)
