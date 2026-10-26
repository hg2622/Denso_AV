"""Pass 3/3: drive the edited scene inside the real episode and record every actor.
  python scene_drive.py --scene scene1 --name hand7 [--ego-start 50]
Spawns exactly the edited actors (cars at their lane, pedestrians where you put them), you drive the one marked 'driven',
every other actor follows its edited plan (cars keep their lane at their speed, walkers walk straight at theirs; speed 0 = still).
No countdown: the world stands still until YOU press any key (or click) in the window; that key press releases the ego and starts
the recording clock (t 0). W/A/S/D drives; recording runs until the episode ends (ESC saves early). Output npc_tracks/<name>.json -> replay with invader_runs.sh / SET=...npc_track=..."""
import argparse, json, math, os, subprocess, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
ap = argparse.ArgumentParser(); ap.add_argument('--scene', required=True); ap.add_argument('--name', required=True)
ap.add_argument('--driver', default='carl', choices=['carl', 'bridgedrive', 'simlingo'], help='which model drives the ego while you drive your car'); ap.add_argument('--ego-start', type=float, default=50.0); ap.add_argument('--snapshot', default=''); ap.add_argument('--autostart', type=float, default=0.0, help='test hook: start after N s as if a key was pressed'); a = ap.parse_args()
WS = '/home/all/carla_ws'; S = json.load(open(f'{WS}/scenes/{a.scene}.json'))
trig = carla.Location(*S['trigger']); route_pts = [carla.Location(*p) for p in S['route']]
driven = next((x for x in S['actors'] if x.get('driven')), None); assert driven is not None, 'no car marked as driven - double-click one in scene_edit.py'
FLAG = f'/tmp/scene_drive_hold_{os.getpid()}'; BG = f'/tmp/scene_drive_bg_{os.getpid()}'
for f in (FLAG, BG):
    if os.path.exists(f): os.remove(f)
open(BG, 'w').close()   # the episode's own traffic never spawns: the scene is exactly what you edited
SETX = f'DynamicObjectCrossing.hold_ego={FLAG},DynamicObjectCrossing.no_cross=1' + (',' + S['set'] if S.get('set') else '')   # the scenario's own pedestrian never crosses: your edited copy does
subprocess.run(['pkill', '-9', '-f', 'leaderboard_evaluato[r].py'], capture_output=True); subprocess.run(['pkill', '-9', '-f', 'CarlaUE4-Linux-Shipp[i]ng'], capture_output=True); time.sleep(2)
logf = open(f'{WS}/logs/scene_drive.log', 'w')
LAUNCH = {'carl': ['bash', f'{WS}/run_carl_eval.sh', S['route_id'], 'CaRL_v1_1_PY_01', '2000'],
          'bridgedrive': ['bash', f'{WS}/run_bridgedrive_eval.sh', S['route_id']],          # its evaluator starts CARLA itself (port 8848, or the next free one)
          'simlingo': ['bash', f'{WS}/run_simlingo_eval.sh', S['route_id']]}[a.driver]
subprocess.Popen(LAUNCH, env=dict(os.environ, SET=SETX, NPC_RECORD_BG_FREEZE=BG), stdout=logf, stderr=subprocess.STDOUT, cwd=WS)
print(f'episode starting with {a.driver} driving the ego (~1-2 min) ...', flush=True)
def live_port():   # the port of the most recently started CARLA server (the Bench2Drive evaluators pick 8848, or the next free one)
    import re
    out = subprocess.run(['pgrep', '-fa', 'CarlaUE4-Linux-Shippin[g]'], capture_output=True, text=True).stdout
    found = [(int(l.split()[0]), int(m.group(1))) for l in out.splitlines() for m in [re.search(r'-carla-rpc-port=(\d+)', l)] if m]
    return max(found)[1] if found else None
def find_hero(w):
    for v in w.get_actors().filter('vehicle.*'):
        if v.attributes.get('role_name') == 'hero': return v
hero = None; t0 = time.time()
while hero is None and time.time() - t0 < 600:
    try:
        port = 2000 if a.driver == 'carl' else live_port()
        if port is None: time.sleep(2.0); continue
        client = carla.Client('localhost', port); client.set_timeout(2.0); client.get_server_version(); client.set_timeout(20.0); world = client.get_world()
        try: world.wait_for_tick(2.0)
        except RuntimeError: pass
        hero = find_hero(world)
    except RuntimeError: time.sleep(2.0); continue
    if hero is None: time.sleep(0.5)
assert hero is not None, 'no ego appeared (see logs/scene_drive.log)'
cmap = world.get_map(); lib = world.get_blueprint_library()
def wait_tick(max_s=90):
    tw = time.time()
    while time.time() - tw < max_s:
        try: return world.wait_for_tick(5.0)
        except RuntimeError: pass
    raise RuntimeError('simulator stopped ticking')
wait_tick()
# clear background traffic that slipped in before the freeze; the scenario's OWN actors (role_name 'scenario': blocker, hidden walker)
# must stay alive - destroying them makes the scenario tree fail and the route stop ~3 s after it starts
for v in list(world.get_actors().filter('vehicle.*')) + list(world.get_actors().filter('walker.pedestrian.*')):
    if v.id == hero.id or v.attributes.get('role_name') in ('hero', 'scenario'): continue
    try: v.destroy()
    except Exception: pass
wait_tick()
swp = cmap.get_waypoint(route_pts[0], lane_type=carla.LaneType.Driving)
hero.set_transform(carla.Transform(carla.Location(route_pts[0].x, route_pts[0].y, hero.get_location().z), swp.transform.rotation))
cars = []
for x in S['actors']:
    bp = lib.find(x['type_id'])
    if bp.has_attribute('color'):
        try: bp.set_attribute('color', x['color'])
        except Exception: pass
    bp.set_attribute('role_name', 'scene')
    tf = carla.Transform(carla.Location(x['x'], x['y'], x['z'] + 0.4), carla.Rotation(yaw=x['yaw']))
    act = world.try_spawn_actor(bp, tf)
    if act is None:
        tf.location.z += 0.6; act = world.try_spawn_actor(bp, tf)
    if act is None: print(f"could not spawn {x['label']}", flush=True); continue
    c = dict(x); c['actor'] = act; c['track'] = []; c['walker'] = (x['kind'] == 'walker'); cars.append(c)
    if c['walker']: act.apply_control(carla.WalkerControl(speed=0.0))
    else: act.set_autopilot(False)
import glob
REFS = []   # baseline reference dots (CaRL green, BridgeDrive blue): where that baseline's ego was t seconds after the route start
for i, f in enumerate(sorted(glob.glob(f"{WS}/npc_tracks/ref_*_{S['route_id']}.json"))):
    r = json.load(open(f)); REFS.append((r['label'], r['track'], [(0, 220, 0), (60, 140, 255), (255, 0, 255), (255, 200, 0)][i % 4]))
def ref_pose(track, t):
    if t <= track[0][0]: return track[0]
    for i in range(len(track) - 1):
        if track[i][0] <= t <= track[i + 1][0]:
            p, q = track[i], track[i + 1]; f = (t - p[0]) / max(q[0] - p[0], 1e-3); return [t, p[1] + f * (q[1] - p[1]), p[2] + f * (q[2] - p[2])] + p[3:]
    return track[-1]
mine = next((c for c in cars if c.get('driven')), None); assert mine is not None, 'the driven car failed to spawn'
print(f"scene spawned: {len(cars)} actors; you drive {mine['label']}", flush=True)
def lane_keep(actor, speed):
    wp = cmap.get_waypoint(actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving); ctl = carla.VehicleControl()
    if wp is None or speed <= 0.05: ctl.brake = 1.0; return ctl
    nxt = wp.next(6.0)
    if not nxt: ctl.brake = 1.0; return ctl
    tgt = min(nxt, key=lambda q: abs(((q.transform.rotation.yaw - wp.transform.rotation.yaw + 180) % 360) - 180)).transform.location
    tf = actor.get_transform(); fwd = tf.get_forward_vector(); d = tgt - tf.location
    lx = d.x * fwd.x + d.y * fwd.y; ly = -d.x * fwd.y + d.y * fwd.x; ctl.steer = max(-0.7, min(0.7, 1.6 * math.atan2(ly, max(lx, 0.1))))
    v = actor.get_velocity(); spd = math.hypot(v.x, v.y)
    if spd < speed: ctl.throttle = min(0.9, 0.45 + 0.25 * (speed - spd))
    elif spd > speed + 1.0: ctl.brake = 0.3
    else: ctl.throttle = 0.35
    return ctl
W, H = 1250, 620; CW, BW = 800, 450; CH = 620; pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption(f'drive the scene - {a.scene} (ego: {a.driver})')
font = pygame.font.SysFont('monospace', 15, bold=True); big = pygame.font.SysFont('monospace', 22, bold=True); imgs = {}; cams = []
for name, (loc, rot, w, h) in {'chase': (carla.Location(x=-7.5, z=4.0), carla.Rotation(pitch=-14), CW, CH), 'bev': (carla.Location(z=48.0), carla.Rotation(pitch=-90), BW, CH)}.items():
    b = lib.find('sensor.camera.rgb'); b.set_attribute('image_size_x', str(w)); b.set_attribute('image_size_y', str(h)); b.set_attribute('fov', '90')
    cam = world.spawn_actor(b, carla.Transform(loc, rot), attach_to=mine['actor'])
    cam.listen(lambda img, n=name: imgs.__setitem__(n, pygame.surfarray.make_surface(np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)))); cams.append(cam)
import time as _t; t_spawn = _t.time(); released = False; go = False; t0 = None; steer = 0.0; ctl = carla.VehicleControl(); ended = False; snap_n = 0
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if not released and ev.type in (pygame.KEYDOWN, pygame.MOUSEBUTTONDOWN) and _t.time() - t_spawn > 1.5:   # YOU start it (a timer cannot work: the held ego stops the world ticking); events from the window opening are ignored
                released = True; open(FLAG, 'w').close(); print('started - you have control', flush=True)
        if not released and a.autostart and _t.time() - t_spawn > a.autostart: released = True; open(FLAG, 'w').close(); print('autostart - you have control', flush=True)
        try: snap = world.wait_for_tick(0.5 if not released else 5.0)
        except RuntimeError:
            try:
                if not hero.is_alive or find_hero(world) is None: ended = True; print('episode ended', flush=True); break
            except RuntimeError: pass
            pygame.event.pump(); continue
        try: alive = hero.is_alive and find_hero(world) is not None
        except RuntimeError: alive = False
        if not alive: ended = True; print('episode ended', flush=True); break
        hloc = hero.get_location(); tsim = snap.timestamp.elapsed_seconds
        if released and not go: go = True; t0 = tsim; print('GO: you have control, recording', flush=True)
        t = (tsim - t0) if go else 0.0
        keys = pygame.key.get_pressed(); ctl.throttle = 0.7 if keys[pygame.K_w] else 0.0; ctl.brake = 1.0 if keys[pygame.K_s] else 0.0
        steer = max(-0.7, min(0.7, steer + (0.03 if keys[pygame.K_d] else 0) - (0.03 if keys[pygame.K_a] else 0))) if (keys[pygame.K_a] or keys[pygame.K_d]) else steer * 0.8; ctl.steer = steer
        driving = released   # your key press released the ego -> you have control (GO only starts the recording clock)
        mine['actor'].apply_control(ctl if driving else carla.VehicleControl(brake=1.0))
        for c in cars:
            if c is mine: continue
            if c['walker']:
                yaw = math.radians(c['yaw']); spd = c['speed'] if driving else 0.0
                c['actor'].apply_control(carla.WalkerControl(direction=carla.Vector3D(math.cos(yaw), math.sin(yaw), 0.0), speed=spd))
            else: c['actor'].apply_control(lane_keep(c['actor'], c['speed']) if driving else carla.VehicleControl(brake=1.0))
        if go:
            for c in cars:
                if not c['actor'].is_alive: continue
                tf = c['actor'].get_transform(); v = c['actor'].get_velocity()
                c['track'].append([round(t, 3), round(tf.location.x, 3), round(tf.location.y, 3), round(tf.location.z, 3), round(tf.rotation.yaw, 2), round(math.hypot(v.x, v.y), 2)])
        for k, (label, track, col) in enumerate(REFS):   # draw the baseline dots in the world (visible in both camera views), stacked so they never hide each other
            r = ref_pose(track, t); world.debug.draw_point(carla.Location(r[1], r[2], r[3] + 1.5 + 1.6 * k), size=0.8, color=carla.Color(*col), life_time=0.12)
        screen.fill((20, 20, 20)); x = 0
        for n, w in (('chase', CW), ('bev', BW)):
            if n in imgs: screen.blit(imgs[n], (x, 0))
            x += w
        vm = mine['actor'].get_velocity(); hv = hero.get_velocity()
        hud = [f"YOU = {mine['label']}  {3.6*math.hypot(vm.x, vm.y):5.1f} km/h  steer {steer:+.2f}   EGO ({a.driver}) {mine['actor'].get_location().distance(hloc):5.1f} m away, {3.6*math.hypot(hv.x, hv.y):4.1f} km/h",
               (f'REC t={t:.1f}s   W/A/S/D to drive   ESC = save' if go else 'press any key to start'),
               'dots = where the baselines were at this time: ' + '  '.join(f"{lab} {'green' if col == (0, 220, 0) else 'blue' if col == (60, 140, 255) else 'magenta' if col == (255, 0, 255) else 'yellow'}" for lab, _, col in REFS)]
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 255, 0)); bgs = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bgs.set_alpha(150); bgs.fill((0, 0, 0))
            screen.blit(bgs, (6, 6 + 20 * i)); screen.blit(lab, (10, 8 + 20 * i))
        if not released: screen.blit(big.render('THIS IS YOUR CAR - press any key (or click) to start', True, (255, 80, 80)), (150, 200))
        pygame.display.flip()
        snap_n += 1
        if a.snapshot:
            if go and len(mine['track']) == 40: pygame.image.save(screen, a.snapshot); print('snapshot saved', flush=True); raise KeyboardInterrupt
except KeyboardInterrupt: pass
finally:
    pygame.quit()
    if mine['track']:
        out = f'{WS}/npc_tracks/{a.name}.json'; os.makedirs(os.path.dirname(out), exist_ok=True)
        json.dump({'route': S['route_id'], 'scenario': S['scenario'], 'town': S['town'], 'ego_driver': a.driver, 'trigger': S['trigger'], 'ego_start': a.ego_start, 'set': S.get('set', ''), 'scene': a.scene,
                   'you': mine['label'], 'filter': mine['type_id'], 'color': mine['color'], 'track': mine['track'], 'clear_radius': 150.0, 'removed': [],
                   'cars': [{'label': c['label'], 'filter': c['type_id'], 'color': c['color'], 'driven': c is mine, 'speed': c['speed'], 'walker': c['walker'], 'track': c['track']} for c in cars if c['track']]}, open(out, 'w'))
        print(f"saved: {len([c for c in cars if c['track']])} actors, {len(mine['track'])} poses ({mine['track'][-1][0]:.1f} s) -> {out}", flush=True)
    else: print('nothing recorded', flush=True)
    subprocess.run(['bash', f'{WS}/stop_all.sh'], capture_output=True); subprocess.run(['pkill', '-9', '-f', 'leaderboard_evaluato[r]'], capture_output=True); logf.close()
    for f in (FLAG, BG):
        try: os.remove(f)
        except OSError: pass
    os._exit(0)
