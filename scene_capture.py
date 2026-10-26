"""Pass 1/3 of the scene pipeline: capture the initial state of a Bench2Drive episode to a JSON scene file (no editing, no driving).
  python scene_capture.py --route 24211 --name scene1 [--settle 12] [--set "..."]
Launches the real evaluator (CaRL ego, own CARLA on 2000), lets the episode spawn its traffic, then writes scenes/<name>.json:
  lanes (map geometry for the 2D editor), route, ego start, every vehicle + pedestrian with its pose, and the scenario trigger.
The scripted scenario walker is parked ~50 m underground until it triggers: its kerb pose is reconstructed here.
Shuts the episode down when done."""
import argparse, json, math, os, subprocess, sys, time, xml.etree.ElementTree as ET
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla
ap = argparse.ArgumentParser(); ap.add_argument('--route', required=True); ap.add_argument('--name', required=True)
ap.add_argument('--settle', type=float, default=12.0); ap.add_argument('--set', default=''); ap.add_argument('--scenario', default='DynamicObjectCrossing')
ap.add_argument('--lane-range', type=float, default=180.0, help='metres of lane geometry around the scene to store for the editor'); a = ap.parse_args()
WS = '/home/all/carla_ws'; root = ET.parse(f'{WS}/Bench2Drive/leaderboard/data/bench2drive220.xml').getroot(); route = next(r for r in root.findall('route') if r.get('id') == a.route)
town = route.get('town'); sc = next(s for s in route.find('scenarios') if s.get('type') == a.scenario); tp = sc.find('trigger_point')
trig = carla.Location(float(tp.get('x')), float(tp.get('y')), float(tp.get('z')))
FLAG = f'/tmp/scene_capture_hold_{os.getpid()}'; BG = f'/tmp/scene_capture_bg_{os.getpid()}'
for f in (FLAG, BG):
    if os.path.exists(f): os.remove(f)
SETX = f'DynamicObjectCrossing.hold_ego={FLAG}' + (',' + a.set if a.set else '')
subprocess.run(['pkill', '-9', '-f', 'leaderboard_evaluato[r].py'], capture_output=True); subprocess.run(['pkill', '-9', '-f', 'CarlaUE4-Linux-Shipp[i]ng'], capture_output=True); time.sleep(2)
logf = open(f'{WS}/logs/scene_capture.log', 'w')
ep = subprocess.Popen(['bash', f'{WS}/run_carl_eval.sh', a.route, 'CaRL_v1_1_PY_01', '2000'], env=dict(os.environ, SET=SETX, NPC_RECORD_BG_FREEZE=BG), stdout=logf, stderr=subprocess.STDOUT, cwd=WS)
print('episode starting (~1 min) ...', flush=True)
def find_hero(w):
    for v in w.get_actors().filter('vehicle.*'):
        if v.attributes.get('role_name') == 'hero': return v
hero = None; t0 = time.time()
while hero is None and time.time() - t0 < 600:
    try:
        client = carla.Client('localhost', 2000); client.set_timeout(2.0); client.get_server_version(); client.set_timeout(20.0); world = client.get_world()
        try: world.wait_for_tick(2.0)
        except RuntimeError: pass
        hero = find_hero(world)
    except RuntimeError: time.sleep(2.0); continue
    if hero is None: time.sleep(0.5)
assert hero is not None, 'no ego appeared (see logs/scene_capture.log)'
cmap = world.get_map(); print('ego found; letting the traffic settle', flush=True)
def wait_tick(max_s=90):
    tw = time.time()
    while time.time() - tw < max_s:
        try: return world.wait_for_tick(5.0)
        except RuntimeError: pass
    raise RuntimeError('simulator stopped ticking')
for i in range(max(1, int(a.settle * 20))):
    wait_tick()
    if i % 60 == 0: print(f'  {i/20:.0f}/{a.settle:.0f} s', flush=True)
open(BG, 'w').close()   # stop further spawning while we read the scene
route_pts = [carla.Location(float(q.get('x')), float(q.get('y')), float(q.get('z'))) for q in route.find('waypoints')]
# ego start = route start (the hold engages a few ticks late)
swp = cmap.get_waypoint(route_pts[0], lane_type=carla.LaneType.Driving)
ego0 = {'x': route_pts[0].x, 'y': route_pts[0].y, 'z': hero.get_location().z, 'yaw': swp.transform.rotation.yaw, 'type_id': hero.type_id}
actors = []
for v in sorted(world.get_actors().filter('vehicle.*'), key=lambda v: v.id):
    if v.id == hero.id or v.attributes.get('role_name') == 'scenario': continue   # the scenario's blocker vehicle is not editable (it stays in the drive pass)
    tf = v.get_transform()
    if math.hypot(tf.location.x - ego0['x'], tf.location.y - ego0['y']) < 6.0: print(f'skip {v.type_id}: sits on the ego start (cannot coexist)', flush=True); continue
    lim = v.get_speed_limit()   # the episode's background traffic drives at the lane's speed limit (tm global_percentage_speed_difference = 0)
    spd = round((lim if lim and lim > 1 else 30.0) / 3.6, 1)
    actors.append({'kind': 'car', 'label': f"car {len([x for x in actors if x['kind']=='car']) + 1} ({v.type_id.split('.')[1]})", 'type_id': v.type_id,
                   'color': v.attributes.get('color', '200,200,200'), 'x': tf.location.x, 'y': tf.location.y, 'z': tf.location.z, 'yaw': tf.rotation.yaw, 'speed': spd})
def kerb_pose():   # reconstruct where the scripted walker will step out
    base = cmap.get_waypoint(trig, lane_type=carla.LaneType.Driving); side = base.get_right_lane() or base
    sw = cmap.get_waypoint(side.transform.location, project_to_road=True, lane_type=carla.LaneType.Sidewalk) or side
    off = sw.transform.get_right_vector(); loc = sw.transform.location + carla.Location(off.x * 1.5, off.y * 1.5, 0.0)
    return loc, sw.transform.rotation.yaw - 90.0
for v in sorted(world.get_actors().filter('walker.pedestrian.*'), key=lambda v: v.id):
    tf = v.get_transform(); loc = tf.location; yaw = tf.rotation.yaw
    if loc.z < -10: loc, yaw = kerb_pose(); print('scenario walker: kerb pose reconstructed', flush=True)
    actors.append({'kind': 'walker', 'label': f"pedestrian {len([x for x in actors if x['kind']=='walker']) + 1}", 'type_id': v.type_id,
                   'color': '255,0,255', 'x': loc.x, 'y': loc.y, 'z': max(loc.z, ego0['z']), 'yaw': yaw, 'speed': 2.0})   # the scenario's crossing speed
# lane geometry around the scene for the 2D editor
cx = sum([ego0['x'], trig.x] + [p['x'] for p in actors]) / (2 + len(actors)); cy = sum([ego0['y'], trig.y] + [p['y'] for p in actors]) / (2 + len(actors))
lanes = []
for wp in cmap.generate_waypoints(2.0):
    L = wp.transform.location
    if math.hypot(L.x - cx, L.y - cy) > a.lane_range: continue
    lanes.append([round(L.x, 2), round(L.y, 2), round(wp.transform.rotation.yaw, 1), round(wp.lane_width, 2), int(wp.lane_id), int(wp.road_id)])
out = f'{WS}/scenes/{a.name}.json'; os.makedirs(os.path.dirname(out), exist_ok=True)
json.dump({'route_id': a.route, 'scenario': a.scenario, 'town': town, 'set': a.set,
           'trigger': [trig.x, trig.y, trig.z], 'route': [[p.x, p.y, p.z] for p in route_pts], 'ego': ego0,
           'actors': actors, 'lanes': lanes, 'centre': [cx, cy]}, open(out, 'w'))
print(f'captured: {len([x for x in actors if x["kind"]=="car"])} cars, {len([x for x in actors if x["kind"]=="walker"])} pedestrians, {len(lanes)} lane points -> {out}', flush=True)
print('default speeds: ' + ', '.join(f"{x['label']} {x['speed']:.0f} m/s" for x in actors), flush=True)
subprocess.run(['bash', f'{WS}/stop_all.sh'], capture_output=True); logf.close()
for f in (FLAG, BG):
    try: os.remove(f)
    except OSError: pass
os._exit(0)
