"""Scene editor + hand-driven NPC recorder INSIDE the real Bench2Drive episode (CaRL as the ego, real scenario / traffic).
  python npc_record.py --route 24211 --name hand1 [--cars "opp:55,same:55"] [--npc-speed 6] [--horizon 8] [--set "DynamicObjectCrossing.no_cross=1"]
SETUP (everything frozen; top-down view, all vehicles labelled, dots = where CaRL / BridgeDrive were at each moment in their own runs):
   left click     select a car: shows where it ends up at the horizon (hollow marker) + the agents' dots at that time
   + / -  (or up/down)  change the selected car's speed (m/s) - the marker moves
   drag           move the car (snapped to the nearest lane centre)
   right click    remove the car from the rollout
   double click   drive this car -> the scene starts at once (nothing moves before that)
PLAY: GO (t 0) when the ego is --ego-start m from the trigger: W/S/A/D; other cars keep their lane at their speed; recording runs
   until the episode ends (ESC = save early). Output npc_tracks/<name>.json: EVERY car of the scene with its track (the driven one flagged).
REPLAY (any model): SET="DynamicObjectCrossing.npc_track=npc_tracks/<name>.json" bash run_<x>_eval.sh 24211   -> the scenario clears the
   episode's own vehicles around the scene and spawns exactly the recorded cars.   TRACK=npc_tracks/<name>.json bash invader_runs.sh ... (videos)"""
import argparse, glob, json, math, os, subprocess, sys, time, xml.etree.ElementTree as ET
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
ap = argparse.ArgumentParser(); ap.add_argument('--route', required=True); ap.add_argument('--name', required=True)
ap.add_argument('--cars', default='opp:55,same:55', help='staged cars lane:dist (opp = opposite lane, same = ego lane; metres from the trigger along the route)')
ap.add_argument('--npc-speed', type=float, default=6.0); ap.add_argument('--ego-start', type=float, default=50.0); ap.add_argument('--horizon', type=float, default=8.0, help='seconds after GO for the end-position preview')
ap.add_argument('--filter', default='vehicle.audi.tt'); ap.add_argument('--set', default='', help='extra SCENARIO_OVERRIDES for the episode'); ap.add_argument('--scenario', default='DynamicObjectCrossing')
ap.add_argument('--clear-radius', type=float, default=150.0, help="in the REPLAY, the episode's own vehicles within this radius of the trigger are removed before the recorded cars are spawned")
ap.add_argument('--clear-ahead', type=float, default=0.0, help="at setup, auto-remove the episode's own vehicles on the ego road up to N m ahead (default 0 = keep everything; you edit by hand)")
ap.add_argument('--settle', type=float, default=12.0, help='seconds to let the episode spawn its traffic before the scene is frozen (more time = more cars)')
ap.add_argument('--refs', default='auto'); ap.add_argument('--snapshot', default='', help='(testing) scripted clicks + screenshots'); a = ap.parse_args()
WS = '/home/all/carla_ws'; root = ET.parse(f'{WS}/Bench2Drive/leaderboard/data/bench2drive220.xml').getroot(); route = next(r for r in root.findall('route') if r.get('id') == a.route)
sc = next(s for s in route.find('scenarios') if s.get('type') == a.scenario); tp = sc.find('trigger_point'); trig = carla.Location(float(tp.get('x')), float(tp.get('y')), float(tp.get('z')))
route_pts = [carla.Location(float(q.get('x')), float(q.get('y')), float(q.get('z'))) for q in route.find('waypoints')]
FLAG = f'/tmp/npc_record_go_{os.getpid()}'
if os.path.exists(FLAG): os.remove(FLAG)
SETX = f'DynamicObjectCrossing.hold_ego={FLAG}' + (',' + a.set if a.set else '')
BGFREEZE = f'/tmp/npc_record_bgfreeze_{os.getpid()}'   # once this file exists the episode's background-traffic manager stops spawning (patched background_activity.py)
if os.path.exists(BGFREEZE): os.remove(BGFREEZE)
# leftovers from a previous run/replay would steal the CARLA / traffic-manager ports ("bind error") and the window would never appear
subprocess.run(['pkill', '-9', '-f', 'leaderboard_evaluato[r].py'], capture_output=True); subprocess.run(['pkill', '-9', '-f', 'run_carl_eva[l].sh'], capture_output=True)
subprocess.run(['pkill', '-9', '-f', 'CarlaUE4-Linux-Shipp[i]ng'], capture_output=True); time.sleep(2)
env = dict(os.environ, SET=SETX, NPC_RECORD_BG_FREEZE=BGFREEZE); logf = open(f'{WS}/logs/npc_record_episode.log', 'w')
ep = subprocess.Popen(['bash', f'{WS}/run_carl_eval.sh', a.route, 'CaRL_v1_1_PY_01', '2000'], env=env, stdout=logf, stderr=subprocess.STDOUT, cwd=WS)
print('episode starting (CaRL ego, own CARLA on 2000; ~1 min) ...', flush=True)
def find_hero(w):
    for v in w.get_actors().filter('vehicle.*'):
        if v.attributes.get('role_name') == 'hero': return v
hero = None; t_wait = time.time()
while hero is None and time.time() - t_wait < 600:
    try:
        client = carla.Client('localhost', 2000); client.set_timeout(2.0); client.get_server_version(); client.set_timeout(20.0); world = client.get_world()
        try: world.wait_for_tick(2.0)
        except RuntimeError: pass
        hero = find_hero(world)
    except RuntimeError: time.sleep(2.0); continue
    if hero is None: time.sleep(0.5)
assert hero is not None, 'no ego appeared (see logs/npc_record_episode.log)'
cmap = world.get_map(); lib = world.get_blueprint_library(); print('ego found (held by the scenario until you double-click a car)', flush=True)
def wait_tick(max_s=90):
    t_w = time.time()
    while time.time() - t_w < max_s:
        try: return world.wait_for_tick(5.0)
        except RuntimeError: pass
    raise RuntimeError('simulator did not tick for %d s' % max_s)
twp = cmap.get_waypoint(trig, lane_type=carla.LaneType.Driving)
def straightest(opts, ref_yaw): return min(opts, key=lambda q: abs(((q.transform.rotation.yaw - ref_yaw + 180) % 360) - 180))
def along(wp, dist):
    left = abs(dist)
    while left > 0:
        n = wp.next(min(left, 5.0)) if dist >= 0 else wp.previous(min(left, 5.0))
        if not n: break
        wp = straightest(n, wp.transform.rotation.yaw); left -= 5.0
    return wp
def opposite(wp):
    o = wp.get_left_lane()
    if o is None or o.lane_id * wp.lane_id > 0 or o.lane_type != carla.LaneType.Driving: o = wp.get_right_lane()
    assert o is not None and o.lane_id * wp.lane_id < 0, 'no opposite lane here'; return o
def lane_keep(actor, speed):
    wp = cmap.get_waypoint(actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving); ctl = carla.VehicleControl()
    if wp is None or speed <= 0.05: ctl.brake = 1.0; return ctl
    nxt = wp.next(6.0)
    if not nxt: ctl.brake = 1.0; return ctl
    tgt = straightest(nxt, wp.transform.rotation.yaw).transform.location; tf = actor.get_transform(); fwd = tf.get_forward_vector(); d = tgt - tf.location
    lx = d.x * fwd.x + d.y * fwd.y; ly = -d.x * fwd.y + d.y * fwd.x; ctl.steer = max(-0.7, min(0.7, 1.6 * math.atan2(ly, max(lx, 0.1))))
    v = actor.get_velocity(); spd = math.hypot(v.x, v.y)
    if spd < speed: ctl.throttle = min(0.9, 0.45 + 0.25 * (speed - spd))
    elif spd > speed + 1.0: ctl.brake = 0.3
    else: ctl.throttle = 0.35
    return ctl
COLORS = ['255,0,0', '255,255,255', '255,140,0', '0,200,255']; cars = []
DEAD = set()   # ids removed by hand: never re-added by refresh, never respawned
wait_tick()
for i, spec in enumerate([x for x in a.cars.split(',') if x]):
    lane, dist = spec.split(':'); wp = along(twp, float(dist)); wp = opposite(wp) if lane == 'opp' else wp
    bp = lib.find(a.filter); bp.set_attribute('color', COLORS[i % len(COLORS)]); bp.set_attribute('role_name', f'npc{i+1}'); v = None
    for shift in [0, 5, -5, 10, -10, 15, -15, 20, -20]:
        w2 = along(wp, shift) if shift else wp; tf0 = w2.transform; tf0.location.z += 0.5; v = world.try_spawn_actor(bp, tf0)
        if v is not None: break
    if v is None: print(f'car {i+1}: no free spot near {float(dist):+.0f} m, skipped', flush=True); continue
    v.set_autopilot(False); cars.append({'label': f'car {i+1} ({lane} {float(dist):+.0f} m)', 'actor': v, 'tf0': tf0, 'track': [], 'color': COLORS[i % len(COLORS)], 'type_id': v.type_id, 'speed': a.npc_speed, 'managed': True})
def freeze(c):   # setup: nothing moves
    if c.get('walker'):
        try: c['actor'].apply_control(carla.WalkerControl(speed=0.0)); c['actor'].set_simulate_physics(False)
        except Exception: pass
        return
    try: c['actor'].set_autopilot(False, 8000)
    except Exception: pass
    try: c['actor'].set_simulate_physics(False); c['actor'].set_target_velocity(carla.Vector3D())
    except Exception: pass
def refresh_traffic():   # the episode spawns its traffic/pedestrians progressively: keep adding (and freezing) new actors while in SETUP
    cars[:] = [c for c in cars if c['actor'].is_alive and c['actor'].id not in DEAD]; known = {c['actor'].id for c in cars} | DEAD; n = len([c for c in cars if c.get('bg')])
    for v in sorted([v for v in world.get_actors().filter('vehicle.*') if v.id != hero.id and v.id not in known], key=lambda v: v.id):
        n += 1; c = {'label': f"traffic {n} ({v.type_id.split('.')[1]})", 'actor': v, 'tf0': v.get_transform(), 'track': [], 'color': '230,230,0', 'bg': True, 'type_id': v.type_id, 'color_attr': v.attributes.get('color', '200,200,200'), 'speed': 0.0, 'managed': False}
        cars.append(c); freeze(c)
    nw = len([c for c in cars if c.get('walker')])
    for v in sorted([v for v in world.get_actors().filter('walker.pedestrian.*') if v.id not in known], key=lambda v: v.id):
        if v.get_location().z < -10:   # the scenario parks its walker ~50 m below ground until it triggers: put it where it will appear
            try:
                base = cmap.get_waypoint(trig, lane_type=carla.LaneType.Driving); side = base.get_right_lane() or base
                sw = cmap.get_waypoint(side.transform.location, project_to_road=True, lane_type=carla.LaneType.Sidewalk) or side
                off = sw.transform.get_right_vector(); loc = sw.transform.location + carla.Location(off.x * 1.5, off.y * 1.5, 1.0)
                v.set_transform(carla.Transform(loc, carla.Rotation(yaw=sw.transform.rotation.yaw - 90.0)))
                print(f'pedestrian {nw + 1}: lifted from the scenario parking spot to the kerb', flush=True)
            except Exception as e: print('could not lift the walker:', e, flush=True)
        nw += 1; c = {'label': f'pedestrian {nw}', 'actor': v, 'tf0': v.get_transform(), 'track': [], 'color': '255,0,255', 'walker': True, 'bg': True, 'type_id': v.type_id, 'color_attr': '255,0,255', 'speed': 0.0, 'yaw': v.get_transform().rotation.yaw, 'managed': False}
        cars.append(c); freeze(c)
        for ctrl in world.get_actors().filter('controller.ai.walker'):   # stop the scenario's own walker AI: you script them now
            try:
                if ctrl.parent is not None and ctrl.parent.id == v.id: ctrl.stop()
            except Exception: pass
def clear_ahead(n_m):
    if n_m <= 0: return
    ewp = cmap.get_waypoint(hero.get_location(), lane_type=carla.LaneType.Driving); fwd = hero.get_transform().get_forward_vector(); hl = hero.get_location(); n_rm = 0
    for c in list(cars):
        if not c.get('bg'): continue
        wp = cmap.get_waypoint(c['actor'].get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None or wp.road_id != ewp.road_id: continue
        d = c['actor'].get_location() - hl; ahead = d.x * fwd.x + d.y * fwd.y
        if 0 < ahead < n_m:
            try: c['actor'].destroy(); n_rm += 1
            except Exception: pass
            cars.remove(c)
    if n_rm: print(f'cleared {n_rm} episode vehicle(s) from the road ahead (--clear-ahead {n_m:.0f})', flush=True)
for c in cars: freeze(c)
n_settle = max(1, int(a.settle * 20))   # let the episode's traffic wave spawn (each car frozen as it appears), then freeze the spawner
for i in range(n_settle):
    wait_tick(); refresh_traffic(); clear_ahead(a.clear_ahead)
    if i % 40 == 0: print(f'  settling {i/20:.0f}/{a.settle:.0f} s: {len([c for c in cars if c.get("bg")])} episode vehicles so far', flush=True)
open(BGFREEZE, 'w').close(); print(f'scene snapshot taken: {len([c for c in cars if c.get("bg")])} episode vehicles kept (frozen); background spawning is now OFF', flush=True)
def prune_newcomers():   # anything that slips in after the snapshot is removed at once (and hand-removed actors stay removed)
    known = {c['actor'].id for c in cars} | {hero.id}
    for v in list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('walker.pedestrian.*')):
        if v.id not in known:
            try: v.destroy()
            except Exception: pass
    cars[:] = [c for c in cars if c['actor'].is_alive and c['actor'].id not in DEAD]
REFS = []; RCOL = [(0, 255, 0), (40, 120, 255), (255, 0, 255), (255, 200, 0)]
for i, f in enumerate(sorted(glob.glob(f'{WS}/npc_tracks/ref_*_{a.route}.json')) if a.refs == 'auto' else [x for x in a.refs.split(',') if x]):
    r = json.load(open(f)); REFS.append((r['label'], r['track'], RCOL[i % len(RCOL)])); print('reference dot:', r['label'], flush=True)
def ref_pose(track, t):
    if t <= track[0][0]: return track[0]
    for i in range(len(track) - 1):
        if track[i][0] <= t <= track[i + 1][0]:
            p, q = track[i], track[i + 1]; f = (t - p[0]) / max(q[0] - p[0], 1e-3); return [t, p[1] + f * (q[1] - p[1]), p[2] + f * (q[2] - p[2]), p[3] + f * (q[3] - p[3]), p[4], p[5]]
    return track[-1]
W, H = 1250, 700; pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption('scene editor / NPC recorder (real episode)'); font = pygame.font.SysFont('monospace', 15, bold=True); big = pygame.font.SysFont('monospace', 21, bold=True)
imgs = {}; cams = []
def spawn_cam(name, loc, rot, w, h, attach, fov=90):
    b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(w)); b.set_attribute('image_size_y', str(h)); b.set_attribute('fov', str(fov))
    cam = world.spawn_actor(b, carla.Transform(loc, rot), attach_to=attach) if attach is not None else world.spawn_actor(b, carla.Transform(loc, rot))
    cam.listen(lambda img, n=name: imgs.__setitem__(n, (pygame.surfarray.make_surface(np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)), img.transform))); cams.append(cam); return cam
pts = [hero.get_location(), trig] + [c['tf0'].location for c in cars if c.get('managed')]; cx = sum(q.x for q in pts) / len(pts); cy = sum(q.y for q in pts) / len(pts)
span = max(math.hypot(q.x - cx, q.y - cy) for q in pts) + 25.0; TOP_Z = max(60.0, span); top = spawn_cam('top', carla.Location(cx, cy, TOP_Z), carla.Rotation(pitch=-90, yaw=twp.transform.rotation.yaw - 90.0), W, H, None)
def project(cam_tf, w, h, fov, loc):
    M = np.array(cam_tf.get_inverse_matrix()); P = M @ np.array([loc.x, loc.y, loc.z, 1.0]); f = (w / 2.0) / math.tan(math.radians(fov) / 2)
    if P[0] < 0.3: return None
    return int(f * P[1] / P[0] + w / 2.0), int(-f * P[2] / P[0] + h / 2.0)
def unproject(cam_tf, w, h, fov, u, v, z_ground):   # pixel -> world point on the ground plane
    f = (w / 2.0) / math.tan(math.radians(fov) / 2); d_cam = np.array([1.0, (u - w / 2.0) / f, -(v - h / 2.0) / f, 0.0]); M = np.array(cam_tf.get_matrix())
    d = M @ d_cam; o = M @ np.array([0, 0, 0, 1.0]); s = (z_ground - o[2]) / d[2]; return carla.Location(float(o[0] + s * d[0]), float(o[1] + s * d[1]), z_ground)
def car_at(pos):
    if 'top' not in imgs: return None
    _, ctf = imgs['top']; best = None
    for c in cars:
        uv = project(ctf, W, H, 90, c['actor'].get_location())
        if uv:
            d = math.hypot(uv[0] - pos[0], uv[1] - pos[1])
            if d < 30 and (best is None or d < best[0]): best = (d, c)
    return best[1] if best else None
def end_pose(c, T):   # cars: along their lane; walkers: straight along their heading
    loc = c['actor'].get_location()
    if c.get('walker'):
        yaw = math.radians(c.get('yaw', c['actor'].get_transform().rotation.yaw)); d = c['speed'] * T
        return carla.Location(loc.x + d * math.cos(yaw), loc.y + d * math.sin(yaw), loc.z)
    wp = cmap.get_waypoint(loc, lane_type=carla.LaneType.Driving)
    return along(wp, c['speed'] * T).transform.location if wp is not None else loc
mine = None; phase = 'setup'; go = False; t0 = None; steer = 0.0; ctl = carla.VehicleControl(); ended = False; freeze_left = 0
sel = None; drag = None; last_click = (None, 0.0); removed = []
def select_drive(car):
    global mine, phase, freeze_left
    mine = car; car['managed'] = True
    spawn_cam('chase', carla.Location(x=-7.5, z=4.0), carla.Rotation(pitch=-14), 800, 450, car['actor']); spawn_cam('bev', carla.Location(z=48.0), carla.Rotation(pitch=-90), 450, 450, car['actor'])
    start_play(); phase = 'play'; print(f"you drive {car['label']}; scene started - W/A/S/D", flush=True)
def start_play():
    for c in cars:
        try: c['actor'].set_simulate_physics(True)
        except Exception: pass
        if c.get('walker') and c['speed'] <= 0.01: c['managed'] = True   # speed 0 = stays put for the whole scene
        if c.get('bg') and not c.get('managed'):
            try: c['actor'].set_autopilot(True, 8000)   # untouched traffic keeps its traffic-manager behaviour (it is recorded too)
            except Exception: pass
    open(FLAG, 'w').close(); print('ego released (flag written)', flush=True)
def remove_car(c):
    removed.append(c['label']); DEAD.add(c['actor'].id)
    if c.get('walker'):   # a walker's AI controller must go too, or it keeps the walker alive/moving
        for ctrl in world.get_actors().filter('controller.ai.walker'):
            try:
                if ctrl.parent is not None and ctrl.parent.id == c['actor'].id: ctrl.stop(); ctrl.destroy()
            except Exception: pass
    try: c['actor'].destroy()
    except Exception: pass
    if c in cars: cars.remove(c)
    print(f"removed {c['label']} from the rollout ({len(cars)} actors left)", flush=True)
def move_car(c, pos):
    _, ctf = imgs['top']; z0 = c['actor'].get_location().z; gp = unproject(ctf, W, H, 90, pos[0], pos[1], z0)
    if c.get('walker'):   # pedestrians can stand anywhere (sidewalk, verge, road): keep the drop point, snap z to the nearest ground
        wp = cmap.get_waypoint(gp, project_to_road=True, lane_type=carla.LaneType.Driving | carla.LaneType.Sidewalk | carla.LaneType.Shoulder)
        z = (wp.transform.location.z if wp is not None else z0) + 0.5
        tf = carla.Transform(carla.Location(gp.x, gp.y, z), carla.Rotation(yaw=c.get('yaw', c['actor'].get_transform().rotation.yaw)))
    else:
        wp = cmap.get_waypoint(gp, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None: print('no driving lane there', flush=True); return
        tf = wp.transform; tf.location.z += 0.3
    c['actor'].set_transform(tf); c['tf0'] = tf; c['managed'] = True; print(f"moved {c['label']}", flush=True)
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: raise KeyboardInterrupt
            if ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE: raise KeyboardInterrupt
                if phase == 'setup' and sel is not None and sel in cars:
                    if ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS, pygame.K_UP): sel['speed'] = min(25.0, sel['speed'] + 1.0); sel['managed'] = True
                    if ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS, pygame.K_DOWN): sel['speed'] = max(0.0, sel['speed'] - 1.0); sel['managed'] = True
                    if ev.key in (pygame.K_DELETE, pygame.K_BACKSPACE): remove_car(sel); sel = None
                    if sel is not None and sel.get('walker') and ev.key in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET, pygame.K_LEFT, pygame.K_RIGHT):
                        sel['yaw'] = (sel.get('yaw', 0.0) + (-15.0 if ev.key in (pygame.K_LEFTBRACKET, pygame.K_LEFT) else 15.0)) % 360.0; sel['managed'] = True
                        tf_ = sel['actor'].get_transform(); tf_.rotation.yaw = sel['yaw']; sel['actor'].set_transform(tf_)
            if phase == 'setup' and ev.type == pygame.MOUSEBUTTONDOWN:
                c = car_at(ev.pos)
                if ev.button == 3 and c is not None: remove_car(c); sel = None if sel is c else sel
                if ev.button == 1 and c is not None:
                    if last_click[0] is c and time.time() - last_click[1] < 0.45:
                        if c.get('walker'): print('pedestrians cannot be driven - edit speed/direction instead', flush=True); sel = c
                        else: select_drive(c)
                        drag = None
                    else: drag = (c, ev.pos, False); last_click = (c, time.time())
            if phase == 'setup' and ev.type == pygame.MOUSEMOTION and drag is not None and ev.buttons[0]:
                if math.hypot(ev.pos[0] - drag[1][0], ev.pos[1] - drag[1][1]) > 6: drag = (drag[0], drag[1], True)
            if phase == 'setup' and ev.type == pygame.MOUSEBUTTONUP and ev.button == 1 and drag is not None:
                c, p0, moved = drag; drag = None
                if moved: move_car(c, ev.pos); sel = c
                else: sel = c
        try: snap = world.wait_for_tick(5.0)
        except RuntimeError:
            try:
                if not hero.is_alive or find_hero(world) is None: ended = True; print('episode ended', flush=True); break
            except RuntimeError: pass
            continue
        try: alive = hero.is_alive and find_hero(world) is not None
        except RuntimeError: alive = False
        if not alive: ended = True; print('episode ended', flush=True); break
        hloc = hero.get_location(); tsim = snap.timestamp.elapsed_seconds
        if phase != 'play' and snap.frame % 5 == 0: prune_newcomers()
        if phase == 'setup' and not globals().get('_ego_reset'):   # the hold engages a few ticks after the route starts: put the frozen ego back on the route start (= where the reference dots and the replay clock start)
            hv_ = hero.get_velocity()
            if math.hypot(hv_.x, hv_.y) < 0.05 and hloc.distance(route_pts[0]) > 1.5:
                swp = cmap.get_waypoint(route_pts[0], lane_type=carla.LaneType.Driving); stf = carla.Transform(carla.Location(route_pts[0].x, route_pts[0].y, hloc.z), swp.transform.rotation)
                hero.set_transform(stf); globals()['_ego_reset'] = True; print(f'ego moved back to the route start ({hloc.distance(route_pts[0]):.1f} m)', flush=True)
        if phase == 'play' and not go and hloc.distance(trig) <= a.ego_start: go = True; t0 = tsim; print(f'GO: t=0 (ego {hloc.distance(trig):.0f} m from the trigger)', flush=True)
        t = (tsim - t0) if go else 0.0
        if phase == 'play':
            keys = pygame.key.get_pressed(); ctl.throttle = 0.7 if keys[pygame.K_w] else 0.0; ctl.brake = 1.0 if keys[pygame.K_s] else 0.0
            steer = max(-0.7, min(0.7, steer + (0.03 if keys[pygame.K_d] else 0) - (0.03 if keys[pygame.K_a] else 0))) if (keys[pygame.K_a] or keys[pygame.K_d]) else steer * 0.8; ctl.steer = steer
            mine['actor'].apply_control(ctl if go else carla.VehicleControl(brake=1.0))
            for c in cars:
                if c is mine or not c.get('managed'): continue
                if c.get('walker'):
                    yaw = math.radians(c.get('yaw', 0.0)); spd = c['speed'] if go else 0.0
                    c['actor'].apply_control(carla.WalkerControl(direction=carla.Vector3D(math.cos(yaw), math.sin(yaw), 0.0), speed=spd))
                else: c['actor'].apply_control(lane_keep(c['actor'], c['speed']) if go else carla.VehicleControl(brake=1.0))
        for k, (label, track, col) in enumerate(REFS):
            r = ref_pose(track, t); loc = carla.Location(r[1], r[2], r[3] + 1.2)
            world.debug.draw_point(loc, size=0.25, color=carla.Color(*col), life_time=0.06); world.debug.draw_string(loc + carla.Location(z=1.0 + 1.5 * k), f'{label} {r[5]*3.6:.0f} km/h', color=carla.Color(*col), life_time=0.06)
        if go:
            for c in cars:
                if not c['actor'].is_alive: continue
                tf = c['actor'].get_transform(); v = c['actor'].get_velocity(); c['track'].append([round(t, 3), round(tf.location.x, 3), round(tf.location.y, 3), round(tf.location.z, 3), round(tf.rotation.yaw, 2), round(math.hypot(v.x, v.y), 2)])
        screen.fill((20, 20, 20)); hv = hero.get_velocity()
        if phase == 'setup' and 'top' in imgs:
            surf, ctf = imgs['top']; screen.blit(surf, (0, 0))
            for k, c in enumerate(cars + [{'label': 'EGO (CaRL; held until you double-click a car)', 'actor': hero, 'color': '230,230,230', 'ego': True}]):
                uv = project(ctf, W, H, 90, c['actor'].get_location())
                if uv:
                    col = tuple(int(x) for x in c['color'].split(',')); r_ = 8 if c.get('bg') and not c.get('managed') else 12
                    pygame.draw.circle(screen, col, uv, r_, 2 if r_ == 8 else 3)
                    if c is sel: pygame.draw.circle(screen, (255, 255, 0), uv, 18, 2)
                    dy = -34 if (k % 2 == 0 or c.get('ego')) else 18; txt = c['label'] + ('' if c.get('ego') else f"  {c['speed']:.0f} m/s")
                    lab = font.render(txt, True, col); bg = pygame.Surface((lab.get_width() + 6, lab.get_height() + 2)); bg.set_alpha(170); bg.fill((0, 0, 0)); screen.blit(bg, (uv[0] + 11, uv[1] + dy - 1)); screen.blit(lab, (uv[0] + 14, uv[1] + dy))
            if sel is not None and sel in cars:   # end-position preview at the horizon
                ep_ = end_pose(sel, a.horizon); uv0 = project(ctf, W, H, 90, sel['actor'].get_location()); uv1 = project(ctf, W, H, 90, ep_)
                if uv0 and uv1: pygame.draw.line(screen, (255, 255, 0), uv0, uv1, 2); pygame.draw.circle(screen, (255, 255, 0), uv1, 12, 2); screen.blit(font.render(f"{sel['label']} at t={a.horizon:.0f}s ({sel['speed']:.0f} m/s)", True, (255, 255, 0)), (uv1[0] + 14, uv1[1] - 8))
                for label, track, col in REFS:
                    r = ref_pose(track, a.horizon); uvr = project(ctf, W, H, 90, carla.Location(r[1], r[2], r[3]))
                    if uvr: pygame.draw.circle(screen, col, uvr, 9, 2); screen.blit(font.render(f'{label} at t={a.horizon:.0f}s', True, col), (uvr[0] + 12, uvr[1] + 8))
            for k, (label, track, col) in enumerate(REFS):
                uv = project(ctf, W, H, 90, carla.Location(track[0][1], track[0][2], track[0][3]))
                if uv: pygame.draw.circle(screen, col, uv, 7); screen.blit(font.render(label + ' (starts at GO)', True, col), (uv[0] + 10, uv[1] + 6 + 18 * k))
            screen.blit(big.render(f'SETUP (frozen, {len(cars)} actors)   click = select / preview    + / - = speed    [ ] = walker direction    drag = move', True, (255, 255, 0)), (10, 6))
            screen.blit(big.render('right-click or DELETE = remove from rollout    DOUBLE-CLICK = drive this car    ESC = quit', True, (255, 255, 0)), (10, 30))
        elif phase == 'play':
            x = 0
            for n, w in (('chase', 800), ('bev', 450)):
                if n in imgs: screen.blit(imgs[n][0], (x, 0))
                x += w
            if 'top' in imgs: screen.blit(pygame.transform.smoothscale(imgs['top'][0], (W, H - 450)), (0, 450))
            tfm = mine['actor'].get_transform(); vm = mine['actor'].get_velocity()
            hud = [f"YOU = {mine['label']}  {3.6*math.hypot(vm.x, vm.y):5.1f} km/h  steer {steer:+.2f}   EGO {tfm.location.distance(hloc):5.1f} m away, {3.6*math.hypot(hv.x, hv.y):4.1f} km/h, {hloc.distance(trig):4.0f} m to the trigger",
                   (f'REC t={t:.1f}s   ESC = save' if go else f'waiting for GO (ego within {a.ego_start:.0f} m of the trigger)')]
            for i, line in enumerate(hud): screen.blit(font.render(line, True, (255, 255, 0)), (8, 8 + 20 * i))
        pygame.display.flip()
        if a.snapshot:   # scripted test: select car 1, +2 speed, screenshot, remove the first traffic car, drag car 2 20 px right, double-click car 1
            sn = globals().get('_sn', 0) + 1; globals()['_sn'] = sn; _, ctf = imgs.get('top', (None, None))
            if sn == 40 and ctf is not None: sel = cars[0]; sel['speed'] += 2; sel['managed'] = True
            if sn == 60: pygame.image.save(screen, a.snapshot.replace('.png', '_setup.png'))
            if sn == 62:
                bgc = next((c for c in cars if c.get('bg')), None)
                if bgc: remove_car(bgc)
                uv = project(ctf, W, H, 90, cars[1]['actor'].get_location()) if len(cars) > 1 else None
                if uv: move_car(cars[1], (uv[0] + 40, uv[1]))
            if sn == 80: pygame.image.save(screen, a.snapshot.replace('.png', '_setup2.png')); select_drive(cars[0])
            if go and mine is not None and len(mine['track']) == 60: pygame.image.save(screen, a.snapshot.replace('.png', '_play.png')); print('snapshots saved', flush=True)
except KeyboardInterrupt: pass
finally:
    pygame.quit()
    if mine is not None and mine['track']:
        out = f'{WS}/npc_tracks/{a.name}.json'; os.makedirs(os.path.dirname(out), exist_ok=True)
        json.dump({'route': a.route, 'scenario': a.scenario, 'town': route.get('town'), 'trigger': [trig.x, trig.y, trig.z], 'ego_start': a.ego_start, 'set': a.set, 'you': mine['label'], 'clear_radius': a.clear_radius, 'removed': removed,
                   'filter': mine.get('type_id', a.filter), 'color': mine.get('color_attr', mine['color']), 'track': mine['track'],
                   'cars': [{'label': c['label'], 'filter': c.get('type_id', a.filter), 'color': c.get('color_attr', c['color']), 'driven': c is mine, 'speed': c['speed'], 'walker': bool(c.get('walker')), 'track': c['track']} for c in cars if c['track']],
                   'extra': [{'filter': c.get('type_id', a.filter), 'color': c.get('color_attr', c['color']), 'label': c['label'], 'track': c['track']} for c in cars if c is not mine and c['track']]}, open(out, 'w'))
        print(f"saved: you ({mine['label']}) {len(mine['track'])} poses ({mine['track'][-1][0]:.1f} s), {len([c for c in cars if c is not mine and c['track']])} other cars, {len(removed)} removed -> {out}", flush=True)
    else: print('nothing recorded (GO never happened or no car selected)', flush=True)
    alive = False
    try: client.set_timeout(2.0); client.get_server_version(); alive = True
    except Exception: pass
    if alive and not ended:
        for c in cams:
            try: c.stop(); c.destroy()
            except Exception: pass
    subprocess.run(['bash', f'{WS}/stop_all.sh'], capture_output=True); subprocess.run(['pkill', '-9', '-f', 'rpc-port=2000 '], capture_output=True); logf.close()
    for f_ in (FLAG, BGFREEZE):
        try: os.remove(f_)
        except OSError: pass
    os._exit(0)
