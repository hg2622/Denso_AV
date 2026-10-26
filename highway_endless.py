"""Endless procedural highway network in CARLA, generated on the run from OpenDRIVE (no Unreal map, no Bench2Drive).
  python highway_endless.py [--lanes 3] [--arm 3000] [--traffic 40] [--seed N]
Every world is a PLUS: four random highway arms (straights + gentle curves, LANES lanes each way, median, shoulders, a fence at
the edge) meeting in a 4-way crossing. You arrive on one arm, cross in any direction (straight / left from the inner lane /
right from the outer lane), and the arm you take leads to the next crossing: near its end the next plus is generated (a few
seconds) and you continue on it - forever, in every direction. Traffic Manager cars drive at highway speed and are recycled
around you; with --chaos (default 1) cars behind overtake and CUT IN front of you, cars ahead brake-check, cars beside drift
into your lane (the HUD announces each). Keys: W/S A/D (Bench2Drive human-agent steering, paced to real time), Shift = gentle, Q = reverse, Space = handbrake,
P = autopilot, ESC = quit.  Bare road on a flat plane: CARLA cannot generate buildings/scenery at runtime."""
import argparse, math, os, random, subprocess, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
ap = argparse.ArgumentParser(); ap.add_argument('--lanes', type=int, default=3); ap.add_argument('--arm', type=float, default=3000.0, help='length of each of the 4 arms (m)')
ap.add_argument('--traffic', type=int, default=60); ap.add_argument('--chaos', type=int, default=1, help='0 off, 1 normal, 2 heavy: cars overtake + cut in front of you, brake-check, drift into your lane'); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--npc-speed', default='90,120', help='NPC desired speed range km/h')
ap.add_argument('--no-server', action='store_true', help='use the CARLA server already running on --port')
ap.add_argument('--test', action='store_true', help='headless check: autopilot ego, short arms, exit after the 2nd world')
ap.add_argument('--snapshot', default=''); a = ap.parse_args()
WS = '/home/all/carla_ws'; LANE_W = 3.5; SHOULDER_W = 2.0; MEDIAN_HALF = 1.25; HBOX = 40.0   # half-size of the crossing box
ARMS = {1: 0.0, 2: math.pi / 2, 3: math.pi, 4: 3 * math.pi / 2}   # road id -> direction the arm points (outward from the crossing)
JUNC = 100; NPC_LO, NPC_HI = (float(x) for x in a.npc_speed.split(','))
if a.test: a.arm = min(a.arm, 1500.0)


def off(j): return MEDIAN_HALF + LANE_W * (j - 0.5)      # lateral offset of driving lane j (1 = inner) from the road centre
def rot(x, y, th): return x * math.cos(th) - y * math.sin(th), x * math.sin(th) + y * math.cos(th)


def make_network(seed, L, lanes):
    rng = random.Random(seed); roads = []; conns = []; cid = [JUNC + 1]
    def lane(i, typ, w, mark, links=''):
        return (f'<lane id="{i}" type="{typ}" level="false"><link>{links}</link><width sOffset="0" a="{w}" b="0" c="0" d="0"/>'
                f'<roadMark sOffset="0" type="{mark}" weight="standard" color="standard" width="0.13" laneChange="both"/></lane>')
    for rid, th in ARMS.items():   # ---- the four arms: random geometry in a local frame (start (HBOX,0) heading 0), rotated by th
        geoms = []; s = 0.0; x, y, hdg = HBOX, 0.0, 0.0
        while s < L:
            gx, gy = rot(x, y, th)
            if rng.random() < 0.45:
                Lg = rng.uniform(250, 800); geoms.append(f'<geometry s="{s:.3f}" x="{gx:.3f}" y="{gy:.3f}" hdg="{hdg + th:.6f}" length="{Lg:.3f}"><line/></geometry>')
                x += Lg * math.cos(hdg); y += Lg * math.sin(hdg)
            else:
                R = rng.uniform(800, 2500) * rng.choice((-1, 1)); k = 1.0 / R; Lg = rng.uniform(300, 900)
                geoms.append(f'<geometry s="{s:.3f}" x="{gx:.3f}" y="{gy:.3f}" hdg="{hdg + th:.6f}" length="{Lg:.3f}"><arc curvature="{k:.8f}"/></geometry>')
                h2 = hdg + k * Lg; x += (math.sin(h2) - math.sin(hdg)) / k; y += -(math.cos(h2) - math.cos(hdg)) / k; hdg = h2
            s += Lg
        left = [lane(1, 'median', MEDIAN_HALF, 'solid')] + [lane(j + 1, 'driving', LANE_W, 'broken' if j < lanes else 'solid') for j in range(1, lanes + 1)] + [lane(lanes + 2, 'shoulder', SHOULDER_W, 'none')]
        right = [lane(-1, 'median', MEDIAN_HALF, 'solid')] + [lane(-(j + 1), 'driving', LANE_W, 'broken' if j < lanes else 'solid') for j in range(1, lanes + 1)] + [lane(-(lanes + 2), 'shoulder', SHOULDER_W, 'none')]
        roads.append(f'<road name="arm{rid}" length="{s:.3f}" id="{rid}" junction="-1"><link><predecessor elementType="junction" elementId="{JUNC}"/></link>'
                     f'<type s="0" type="motorway"><speed max="120" unit="km/h"/></type><planView>{"".join(geoms)}</planView>'
                     f'<elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile><lateralProfile/>'
                     f'<lanes><laneSection s="0"><left>{"".join(reversed(left))}</left><center><lane id="0" type="none" level="false"><link/><roadMark sOffset="0" type="solid" weight="bold" color="yellow" width="0.15"/></lane></center>'
                     f'<right>{"".join(right)}</right></laneSection></lanes></road>')
    def connector(a_id, b_id, j_from, j_to, geom_local, th, length):   # one-lane road inside the crossing: arm a lane +j_from -> arm b lane -j_to
        rid = cid[0]; cid[0] += 1; (x0, y0, h0, kind, k) = geom_local; gx, gy = rot(x0, y0, th)
        g = f'<geometry s="0" x="{gx:.3f}" y="{gy:.3f}" hdg="{h0 + th:.6f}" length="{length:.3f}">' + ('<line/>' if kind == 'line' else f'<arc curvature="{k:.8f}"/>') + '</geometry>'
        roads.append(f'<road name="c{rid}" length="{length:.3f}" id="{rid}" junction="{JUNC}"><link><predecessor elementType="road" elementId="{a_id}" contactPoint="start"/>'
                     f'<successor elementType="road" elementId="{b_id}" contactPoint="start"/></link><planView>{g}</planView>'
                     f'<elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile><lateralProfile/>'
                     f'<lanes><laneSection s="0"><center><lane id="0" type="none" level="false"><link/></lane></center><right>'
                     f'<lane id="-1" type="driving" level="false"><link><predecessor id="{j_from + 1}"/><successor id="{-(j_to + 1)}"/></link><width sOffset="0" a="{LANE_W}" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road>')
        conns.append(f'<connection id="{len(conns)}" incomingRoad="{a_id}" connectingRoad="{rid}" contactPoint="start"><laneLink from="{j_from + 1}" to="-1"/></connection>')
    for a_id, th in ARMS.items():   # incoming traffic on arm a drives toward the crossing on its left lanes (+ids); local frame: at (HBOX, off_j) heading pi
        straight_to = {1: 3, 2: 4, 3: 1, 4: 2}[a_id]; right_to = {1: 2, 2: 3, 3: 4, 4: 1}[a_id]; left_to = {1: 4, 2: 1, 3: 2, 4: 3}[a_id]
        for j in range(1, lanes + 1):
            connector(a_id, straight_to, j, j, (HBOX, off(j), math.pi, 'line', 0.0), th, 2 * HBOX)
        R = HBOX - off(lanes); connector(a_id, right_to, lanes, lanes, (HBOX, off(lanes), math.pi, 'arc', -1.0 / R), th, R * math.pi / 2)   # outer lane turns right
        R = HBOX + off(1); connector(a_id, left_to, 1, 1, (HBOX, off(1), math.pi, 'arc', 1.0 / R), th, R * math.pi / 2)                     # inner lane turns left
    # paved cover under the whole crossing box (the connecting roads alone leave holes); 'parking' lanes so driving-lane queries ignore it
    cover = (f'<road name="cover" length="{2 * HBOX:.1f}" id="99" junction="-1"><link/><planView><geometry s="0" x="{-HBOX:.1f}" y="0" hdg="0" length="{2 * HBOX:.1f}"><line/></geometry></planView>'
             f'<elevationProfile><elevation s="0" a="-0.04" b="0" c="0" d="0"/></elevationProfile><lateralProfile/><lanes><laneSection s="0">'
             f'<left><lane id="1" type="parking" level="false"><link/><width sOffset="0" a="{HBOX:.1f}" b="0" c="0" d="0"/></lane></left><center><lane id="0" type="none" level="false"><link/></lane></center>'
             f'<right><lane id="-1" type="parking" level="false"><link/><width sOffset="0" a="{HBOX:.1f}" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road>')
    roads.append(cover)
    xodr = (f'<?xml version="1.0" standalone="yes"?><OpenDRIVE><header revMajor="1" revMinor="4" name="plus_{seed}" version="1.00" date="2026" north="0" south="0" east="0" west="0"/>'
            + ''.join(roads) + f'<junction id="{JUNC}" name="crossing">' + ''.join(conns) + '</junction></OpenDRIVE>')
    return xodr


# ---------------------------------------------------------------- CARLA server + client
if not a.no_server:
    subprocess.run(['pkill', '-9', '-f', 'CarlaUE4-Linux-Shipp[i]ng'], capture_output=True); time.sleep(2)
    server = subprocess.Popen(['bash', '-c', f'cd {WS}/carla && ./CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port={a.port} -carla-streaming-port=0 -quality-level=Epic'],
                              stdout=open(f'{WS}/logs/highway_server.log', 'w'), stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    print('CARLA starting ...', flush=True)
client = None
for i in range(90):
    try:
        client = carla.Client('localhost', a.port); client.set_timeout(3.0); client.get_server_version(); break
    except RuntimeError: time.sleep(2)
assert client is not None, 'no CARLA server'
client.set_timeout(180.0)
PARAMS = carla.OpendriveGenerationParameters(vertex_distance=2.5, max_road_length=400.0, wall_height=0.9, additional_width=0.6,
                                             smooth_junctions=True, enable_mesh_visibility=True, enable_pedestrian_navigation=False)


def load_world(seed):
    xodr = make_network(seed, a.arm, a.lanes)
    print(f'generating world #{seed}: 4 arms x {a.arm/1000:.1f} km, {a.lanes} lanes each way, crossing in the middle ...', flush=True)
    t0 = time.time(); world = client.generate_opendrive_world(xodr, PARAMS)
    st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = 0.05; world.apply_settings(st)
    world.set_weather(carla.WeatherParameters(cloudiness=30.0, precipitation=0.0, sun_altitude_angle=38.0, sun_azimuth_angle=250.0, fog_density=3.0, wetness=0.0))
    tm = client.get_trafficmanager(a.port + 6000); tm.set_synchronous_mode(True); tm.set_global_distance_to_leading_vehicle(6.0)
    world.tick(); print(f'  built in {time.time() - t0:.1f} s', flush=True); return world, world.get_map(), tm


def wp_at(cmap, road_id, lane_id, s): return cmap.get_waypoint_xodr(road_id, lane_id, s)
def spawn_at(world, cmap, bp, road_id, lane_id, s):
    wp = wp_at(cmap, road_id, lane_id, s)
    if wp is None: return None
    tf = wp.transform; tf.location.z += 0.5; return world.try_spawn_actor(bp, tf)
def where(cmap, loc):
    wp = cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving); return (wp.road_id, wp.lane_id, wp.s) if wp else (0, 0, 0.0)


W, H = 1200, 660; pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption('endless highway network (generated on the run)')
font = pygame.font.SysFont('monospace', 17, bold=True); big = pygame.font.SysFont('monospace', 26, bold=True); img = {}
seed = a.seed or random.randrange(1, 10**6); world_n = 0; km_total = 0.0; autopilot = a.test; steer = 0.0; ctl = carla.VehicleControl()
world = cmap = tm = ego = cam = None; npcs = []; ego_lane_j = 2; ego_kmh = 0.0


def place_npc(v, rng, near=None):
    """put an NPC on a random arm lane; near=(road, lane, s): within 250-1200 m ahead of the ego on its road"""
    for _ in range(6):
        if near:
            road, lane, s = near; j = rng.randint(1, a.lanes); same_dir = rng.random() < 0.75
            lane_id = (lane // abs(lane)) * (j + 1) if same_dir else -(lane // abs(lane)) * (j + 1)
            ahead = rng.uniform(250, 1200); ns = s - ahead if lane > 0 else s + ahead   # +lanes travel toward s=0, -lanes outward
            if not (50 < ns < a.arm - 50): road = rng.choice(list(ARMS)); ns = rng.uniform(100, a.arm - 100)
        else: road = rng.choice(list(ARMS)); lane_id = rng.choice((-1, 1)) * rng.randint(2, a.lanes + 1); ns = rng.uniform(100, a.arm - 100)
        wp = wp_at(cmap, road, lane_id, ns)
        if wp is None: continue
        tf = wp.transform; tf.location.z += 0.3; v.set_transform(tf); v.set_target_velocity(carla.Vector3D(0, 0, 0)); return True
    return False


def build(seed, lane_j, kmh):
    global world, cmap, tm, ego, cam, npcs, world_n
    world, cmap, tm = load_world(seed); lib = world.get_blueprint_library(); world_n += 1; rng = random.Random(seed)
    eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '20,20,20')
    ego = spawn_at(world, cmap, eb, 3, lane_j + 1, a.arm - 150.0)   # arrive on the west arm, driving toward the crossing (left lanes = inbound)
    assert ego is not None, 'ego spawn failed'
    cb = lib.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(W)); cb.set_attribute('image_size_y', str(H)); cb.set_attribute('fov', '95'); cb.set_attribute('exposure_compensation', '-1.2')
    cam = world.spawn_actor(cb, carla.Transform(carla.Location(x=-7.0, z=3.2), carla.Rotation(pitch=-11)), attach_to=ego)
    cam.listen(lambda im: img.__setitem__('cam', pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1))))
    cars = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4 and 'carlacola' not in b.id and 'firetruck' not in b.id and 'ambulance' not in b.id]
    npcs = []
    for i in range(a.traffic):
        b = rng.choice(cars)
        if b.has_attribute('color'): b.set_attribute('color', rng.choice(b.get_attribute('color').recommended_values))
        near = i < (2 * a.traffic) // 3; v = None
        for _ in range(6):
            if near:
                j = rng.randint(1, a.lanes); same_dir = rng.random() < 0.75; lane_id = (j + 1) if same_dir else -(j + 1); ns = a.arm - 150.0 - rng.uniform(150, 1400); road = 3
            else: road = rng.choice(list(ARMS)); lane_id = rng.choice((-1, 1)) * rng.randint(2, a.lanes + 1); ns = rng.uniform(100, a.arm - 100)
            v = spawn_at(world, cmap, b, road, lane_id, ns)
            if v is not None: break
        if v is None: continue
        v.set_autopilot(True, tm.get_port()); tm.set_desired_speed(v, rng.uniform(NPC_LO, NPC_HI)); tm.auto_lane_change(v, True)
        tm.distance_to_leading_vehicle(v, rng.uniform(5, 12)); npcs.append(v)
    for _ in range(3): world.tick()
    ego.set_autopilot(autopilot, tm.get_port())
    if kmh > 1: f = ego.get_transform().get_forward_vector(); ego.set_target_velocity(carla.Vector3D(f.x * kmh / 3.6, f.y * kmh / 3.6, 0.0))
    print(f'world #{world_n} ready: {len(npcs)} NPC cars at {NPC_LO:.0f}-{NPC_HI:.0f} km/h; crossing {a.arm/1000:.1f} km ahead', flush=True)


def recycle_npcs(ego_where):
    rng = random.Random(); road, lane, s = ego_where; eloc = ego.get_location()
    for v in npcs:
        try:
            if v.id in roles: continue
            if v.get_location().distance(eloc) > 2200 or v.get_location().z < -3: place_npc(v, rng, near=ego_where if road in ARMS and lane != 0 else None)
        except RuntimeError: pass


def draw_markings(ego_where):
    """CARLA's generated mesh has no visible paint: draw dashed lane lines + solid edges/median 450 m ahead on the ego's arm (refreshed)."""
    road, lane, s = ego_where
    if road not in ARMS or lane == 0: return
    dbg = world.debug; white = carla.Color(9, 9, 9); yellow = carla.Color(12, 9, 1); life = 6.0   # debug shapes are emissive: dark values render as paint
    inbound = lane > 0; s0, s1 = (max(0.0, s - 460), min(a.arm, s + 20)) if inbound else (max(0.0, s - 20), min(a.arm, s + 460))
    def edge(wp, side):
        yaw = math.radians(wp.transform.rotation.yaw); rx, ry = math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2); w = wp.lane_width / 2.0 * side
        return carla.Location(wp.transform.location.x + rx * w, wp.transform.location.y + ry * w, wp.transform.location.z + 0.05)
    ss = s0
    while ss < s1:
        for lane_id in [-(j + 1) for j in range(1, a.lanes + 1)] + [j + 1 for j in range(1, a.lanes + 1)]:
            outer = abs(lane_id) == a.lanes + 1; inner = abs(lane_id) == 2
            w1 = wp_at(cmap, road, lane_id, ss); w2 = wp_at(cmap, road, lane_id, min(ss + 3.0, s1 - 0.01))
            if w1 is None or w2 is None: continue
            if outer or int(ss // 3) % 2 == 0: dbg.draw_line(edge(w1, 1), edge(w2, 1), thickness=0.07, color=white, life_time=life)
            if inner: dbg.draw_line(edge(w1, -1), edge(w2, -1), thickness=0.08, color=yellow, life_time=life)
        ss += 3.0


# ---------------------------------------------------------------- chaos: scripted hazards around you (ported from traffic_near.py --chaos)
roles = {}; cooldown = {}; EVENT = ['', 0.0]; SIM_T = 0.0
ROLE_LIMIT = {0: 0, 1: 2, 2: 4}[a.chaos]; CHAOS_P = {0: 0.0, 1: 0.5, 2: 0.9}[a.chaos]
def _tm(call):
    try: call()
    except RuntimeError: pass
def _rel(hl, fwd, right, loc):
    d = loc - hl; return d.x * fwd.x + d.y * fwd.y, d.x * right.x + d.y * right.y
def _wp(loc): return cmap.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
def _fs(wp): return wp.s if wp.lane_id < 0 else -wp.s
def _speed(v): vv = v.get_velocity(); return math.hypot(vv.x, vv.y)
def lane_free(allv, wp_lane, s_from, s_to, exclude):
    s0 = _fs(wp_lane)
    for o in allv:
        if o.id in exclude: continue
        wo = _wp(o.get_location())
        if wo is None or wo.road_id != wp_lane.road_id or wo.lane_id != wp_lane.lane_id: continue
        if s_from < _fs(wo) - s0 < s_to: return False
    return True
def neighbour(wp, side):
    nb = wp.get_left_lane() if side < 0 else wp.get_right_lane()
    return nb if (nb is not None and nb.lane_type == carla.LaneType.Driving and nb.lane_id * wp.lane_id > 0) else None
def ahead_blocked(allv, v, dist):
    tf = v.get_transform(); f = tf.get_forward_vector(); r = tf.get_right_vector()
    for o in allv:
        if o.id == v.id: continue
        lon, lat = _rel(tf.location, f, r, o.get_location())
        if 0 < lon < dist and abs(lat) < 2.2: return True
    return False
def steer_to(v, r, target, vspeed, tgt_speed):
    tf = v.get_transform(); f = tf.get_forward_vector(); rt = tf.get_right_vector(); lon, lat = _rel(tf.location, f, rt, target)
    L = max(math.hypot(lon, lat), 4.0); alpha = math.atan2(lat, max(lon, 0.5)); delta = math.atan2(2.0 * 2.9 * math.sin(alpha), L)
    cap = 0.5 if vspeed < 15 else 0.2; st = max(-cap, min(cap, delta)); prev = r.get('steer', 0.0); step = 0.02
    st = max(prev - step, min(prev + step, st)); r['steer'] = st; err = tgt_speed - vspeed
    thr = max(0.0, min(1.0, 0.5 + 0.25 * err)); brk = 1.0 if err < -3.0 else 0.0
    v.apply_control(carla.VehicleControl(throttle=thr if brk == 0 else 0.0, brake=brk, steer=st))
def end_role(v, now, cd):
    roles.pop(v.id, None); cooldown[v.id] = now + cd
    if v.is_alive:
        try: v.set_autopilot(True, tm.get_port()); _tm(lambda: tm.vehicle_lane_offset(v, 0.0))
        except RuntimeError: pass
def say(txt): EVENT[0] = txt; EVENT[1] = SIM_T; print(f'[chaos] {txt}', flush=True)
def drive_actors(h):
    if not roles: return
    htf = h.get_transform(); hl = htf.location; fwd = htf.get_forward_vector(); right = htf.get_right_vector(); now = SIM_T
    hspeed = _speed(h); allv = list(world.get_actors().filter('vehicle.*')); wh = _wp(hl)
    for v in list(npcs):
        r = roles.get(v.id)
        if r is None: continue
        if not v.is_alive or v.get_location().distance(hl) > 250: end_role(v, now, 10); continue
        if r['role'] == 'brake':
            if now - r['t0'] < r['dur']: v.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=r.get('steer', 0.0)))
            else: end_role(v, now, 25)
            continue
        if r['role'] == 'drift':   # TM keeps driving it; we only push its lane offset toward you and back
            ph = (now - r['t0']) / r['dur']
            if ph >= 1.0: end_role(v, now, 25); continue
            k = math.sin(math.pi * min(ph, 1.0)); _tm(lambda: tm.vehicle_lane_offset(v, r['dir'] * 1.9 * k)); continue
        wv = _wp(v.get_location())
        if wv is None: end_role(v, now, 10); continue
        lon, lat = _rel(hl, fwd, right, v.get_location()); vspeed = _speed(v)
        in_hero_lane = wh is not None and wv.road_id == wh.road_id and wv.lane_id == wh.lane_id
        if r['phase'] == 'pass':
            lane = neighbour(wv, r['side']) if in_hero_lane else wv
            if lane is None: end_role(v, now, 15); continue
            if ahead_blocked(allv, v, 8.0 + vspeed * 0.7): v.apply_control(carla.VehicleControl(throttle=0.0, brake=0.8, steer=r.get('steer', 0.0)))
            else:
                nxt = lane.next(max(12.0, vspeed * 1.3)); tgt = nxt[0].transform.location if nxt else lane.transform.location
                steer_to(v, r, tgt, vspeed, min(hspeed + 6.0, 42.0))
            if lon > 6 and not in_hero_lane and wh is not None and lane_free(allv, wh, 1.0, lon + 15.0, {h.id, v.id}):
                r['phase'] = 'cut'; r['t0'] = now; say('CUT-IN: the car ahead swings into your lane')
            elif now - r['t0'] > 25 or lon > 80 or lon < -100: end_role(v, now, 15)
        else:
            lane = wv if in_hero_lane else neighbour(wv, -r['side'])
            if lane is None or ahead_blocked(allv, v, 6.0 + vspeed * 0.5): v.apply_control(carla.VehicleControl(throttle=0.0, brake=0.6, steer=r.get('steer', 0.0)))
            else:
                nxt = lane.next(max(12.0, vspeed * 1.3)); tgt = nxt[0].transform.location if nxt else lane.transform.location
                steer_to(v, r, tgt, vspeed, max(hspeed - 2.0, 4.0))
            if (in_hero_lane and now - r['t0'] > 2.5) or now - r['t0'] > 8: end_role(v, now, 20)
def manage_chaos(h):
    htf = h.get_transform(); hl = htf.location; fwd = htf.get_forward_vector(); right = htf.get_right_vector(); now = SIM_T
    hspeed = _speed(h); wh = _wp(hl); free = ROLE_LIMIT - len(roles)
    if free <= 0 or wh is None or wh.is_junction or hspeed < 3.0: return
    allv = list(world.get_actors().filter('vehicle.*')); rng = random.Random()
    cand = [v for v in npcs if v.is_alive and v.id not in roles and cooldown.get(v.id, 0) < now and v.get_location().distance(hl) < 120]; rng.shuffle(cand)
    for v in cand:
        if free <= 0: break
        lon, lat = _rel(hl, fwd, right, v.get_location()); vf = v.get_transform().get_forward_vector()
        if (vf.x * fwd.x + vf.y * fwd.y) < 0.7 or abs(lat) > 6.0: continue
        in_my_lane = abs(lat) < 2.0; wv = _wp(v.get_location())
        if wv is None or wv.is_junction: continue
        if -70 < lon < (-5 if in_my_lane else 12) and rng.random() < CHAOS_P:        # behind / beside -> overtake then cut in
            if in_my_lane:
                side = -1 if neighbour(wv, -1) is not None else (1 if neighbour(wv, 1) is not None else 0)
                if side == 0: continue
                plane = neighbour(wv, side)
            else:
                side = -1 if lat < 0 else 1; nb = neighbour(wh, side)
                if nb is None or nb.lane_id != wv.lane_id or nb.road_id != wv.road_id: continue
                plane = wv
            if not lane_free(allv, plane, 2.0, 30.0, {v.id}) or not lane_free(allv, wh, 1.0, 25.0, {h.id}): continue
            try: v.set_autopilot(False, tm.get_port())
            except RuntimeError: continue
            roles[v.id] = dict(role='overtake', phase='pass', t0=now, side=side); free -= 1; say(f'a car behind overtakes you on the {"left" if side < 0 else "right"}')
        elif in_my_lane and 25 < lon < 70 and rng.random() < 0.12 * CHAOS_P:           # ahead in my lane -> brake check
            try: v.set_autopilot(False, tm.get_port())
            except RuntimeError: continue
            roles[v.id] = dict(role='brake', phase='brake', t0=now, dur=rng.uniform(1.2, 2.0), steer=v.get_control().steer); free -= 1; say('BRAKE CHECK: the car ahead slams the brakes')
        elif not in_my_lane and -5 < lon < 30 and rng.random() < 0.25 * CHAOS_P:      # beside / just ahead in the next lane -> drifts into your lane and back
            roles[v.id] = dict(role='drift', t0=now, dur=rng.uniform(3.0, 5.0), dir=(1 if lat < 0 else -1)); free -= 1; say(f'DRIFTER: the car on your {"left" if lat < 0 else "right"} drifts into your lane')


def teardown():
    global cam, ego, npcs
    try:
        if cam is not None and cam.is_alive: cam.stop(); time.sleep(0.3)
    except Exception: pass
    for act in [cam, ego] + list(npcs):
        try:
            if act is not None and act.is_alive: act.destroy()
        except Exception: pass
    cam = ego = None; npcs = []


build(seed, ego_lane_j, 0.0); t_start = time.time(); tick_n = 0; build_tick = 0; clock = pygame.time.Clock(); ms_last = 50; prev_loc = None
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_p: autopilot = not autopilot; ego.set_autopilot(autopilot, tm.get_port()); print('autopilot', autopilot, flush=True)
            if ev.type == pygame.KEYUP and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
        keys = pygame.key.get_pressed()
        vv = ego.get_velocity(); ego_kmh = 3.6 * math.hypot(vv.x, vv.y)
        if not autopilot:
            # Bench2Drive human-agent steering: 5e-4 per real ms held, snaps to 0 on release; cap eases above 50 km/h (0.7 -> 0.25 at 120)
            gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
            ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
            smax = 0.7 if ego_kmh < 50 else max(0.25, 0.7 - (ego_kmh - 50) * (0.45 / 70)); inc = 5e-4 * ms_last
            if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-smax, steer - inc)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(smax, steer + inc)
            else: steer = 0.0
            ctl.steer = round(steer, 2); ego.apply_control(ctl)
        ms_last = clock.tick_busy_loop(20)   # real-time 20 Hz like the leaderboard
        world.tick(); tick_n += 1
        SIM_T = world.get_snapshot().timestamp.elapsed_seconds
        if a.chaos: drive_actors(ego)
        if a.chaos and tick_n % 5 == 0: manage_chaos(ego)
        loc = ego.get_location(); ew = where(cmap, loc); road, lane, es = ew
        if prev_loc is not None: km_total += loc.distance(prev_loc) / 1000.0
        prev_loc = loc
        if tick_n % 40 == 0: recycle_npcs(ew)
        if tick_n % 60 == 1: draw_markings(ew)
        at_end = road in ARMS and lane < 0 and es > a.arm - 600   # outbound on an arm, near its far end -> the next plus (the crossing you just used is behind you)
        if at_end or loc.z < -5:
            if a.test and world_n >= 2: break
            lane_j = max(1, min(a.lanes, abs(lane) - 1)) if lane != 0 else 2
            screen.fill((15, 15, 15)); screen.blit(big.render(f'generating the next highway + crossing (#{world_n + 1}) ...', True, (255, 255, 0)), (220, 300)); pygame.display.flip()
            teardown(); roles.clear(); cooldown.clear(); seed += 1; img.clear(); build(seed, lane_j, 0.0 if loc.z < -5 else ego_kmh); build_tick = tick_n; prev_loc = None; continue
        if 'cam' in img: screen.blit(img['cam'], (0, 0))
        if road in ARMS: pos = (f'to the crossing in {es:.0f} m' if lane > 0 else f'crossing behind, next one in {a.arm - es:.0f} m')
        else: pos = 'IN THE CROSSING: straight / left (inner lane) / right (outer lane)'
        hud = [f'{ego_kmh:5.1f} km/h {"REVERSE " if ctl.reverse else ""}  world #{world_n}   {pos}   total {km_total:.2f} km   {len(npcs)} cars',
               'AUTOPILOT (P = manual)' if autopilot else 'W/S A/D  Shift gentle  Q reverse  Space handbrake  P autopilot  ESC quit']
        if EVENT[0] and SIM_T - EVENT[1] < 3.0: hud.append('!! ' + EVENT[0])
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 255, 0)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0))
            screen.blit(bg, (6, 6 + 22 * i)); screen.blit(lab, (10, 8 + 22 * i))
        pygame.display.flip()
        if a.test and a.snapshot and world_n == 1 and road not in ARMS and 'cam' in img and not os.path.exists(a.snapshot): pygame.image.save(screen, a.snapshot); print('snapshot (in the crossing) saved', flush=True)
        if a.test and time.time() - t_start > 900: break
except KeyboardInterrupt: pass
finally:
    print(f'done: {km_total:.2f} km over {world_n} world(s)', flush=True); pygame.quit()
    if not a.no_server: subprocess.run(['pkill', '-9', '-f', 'CarlaUE4-Linux-Shipp[i]ng'], capture_output=True)
    else:
        teardown()
        try:
            st = world.get_settings(); st.synchronous_mode = False; world.apply_settings(st)
        except Exception: pass
    os._exit(0)
