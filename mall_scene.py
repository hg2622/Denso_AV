"""Mall of America inspired scene (from mall_of_america_carla_scenic.zip), rebuilt for our CARLA 0.9.15 without Scenic:
  python mall_scene.py [--cars 24] [--walkers 60] [--parked-stride 2] [--test]
The bundle's authored OpenDRIVE loop (~600 x 450 m, two-way, 3.5 m lanes, 2 m sidewalks) is generated live; the mall massing
(central mall, two garages, north entrance wing) is built from engine cube props as in the bundle's block mode; the north surface
lot gets its parked cars; Traffic Manager cars drive the loop; pedestrians walk the sidewalks (and gather at the entrances);
at the south entrance a pedestrian waits at the kerb and crosses when you come within --trigger m, as in the Scenic program.
You drive the ego (W/S A/D, Shift gentle, Q reverse, Space handbrake, P autopilot, ESC quit), starting on the south side heading east.
Needs the CARLA server on port 2000 (any quality)."""
import argparse, json, math, os, random, sys, time
sys.path.insert(0, '/home/all/carla_ws/carla/PythonAPI/carla'); import carla, numpy as np, pygame
ROOT = '/home/all/carla_ws/mall_of_america/mall_of_america_scene'
ap = argparse.ArgumentParser(); ap.add_argument('--cars', type=int, default=24); ap.add_argument('--walkers', type=int, default=60); ap.add_argument('--parked-stride', type=int, default=2)
ap.add_argument('--trigger', type=float, default=30.0); ap.add_argument('--seed', type=int, default=0); ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--test', action='store_true'); ap.add_argument('--snapshot', default=''); a = ap.parse_args()
rng = random.Random(a.seed or None)
client = carla.Client('localhost', a.port); client.set_timeout(300.0)
print('generating the mall road network ...', flush=True)
world = client.generate_opendrive_world(open(f'{ROOT}/mall.xodr').read(), carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True, enable_pedestrian_navigation=False))
st = world.get_settings(); st.synchronous_mode = True; st.fixed_delta_seconds = 0.05; world.apply_settings(st)
world.set_weather(carla.WeatherParameters(cloudiness=25.0, sun_altitude_angle=45.0, sun_azimuth_angle=150.0, fog_density=2.0))
tm = client.get_trafficmanager(a.port + 6000); tm.set_synchronous_mode(True); tm.set_global_distance_to_leading_vehicle(4.0); tm.global_percentage_speed_difference(-20.0)
lib = world.get_blueprint_library(); cmap = world.get_map(); world.tick()
layout = json.load(open(f'{ROOT}/layout.json'))
def C(x, y, z=0.0): return carla.Location(x=float(x), y=-float(y), z=float(z))   # authored frame (y north) -> CARLA (y south)

# ---- massing from engine cubes (bundle 'blocks' mode): a sunk 100 m cube grid gives the ground, 20 m cubes the buildings
cube = lib.find('static.prop.mesh'); cube.set_attribute('mesh_path', '/Engine/BasicShapes/Cube.Cube'); props = []
def block(size, x, y, z):
    cube.set_attribute('scale', str(size)); p = world.try_spawn_actor(cube, carla.Transform(C(x, y, z)))
    if p is not None: p.set_simulate_physics(False); props.append(p)
for x in range(-350, 351, 100):
    for y in range(-250, 451, 100): block(100, x, y, -50.02)
for b in layout['buildings']:
    nx, ny = math.ceil(b['w'] / 20), math.ceil(b['d'] / 20)
    for ix in range(nx):
        for iy in range(ny): block(20, b['x'] - b['w'] / 2 + 10 + ix * 20, b['y'] - b['d'] / 2 + 10 + iy * 20, 10)
world.tick(); print(f'massing: {len(props)} cube props', flush=True)

cars_bp = [b for b in lib.filter('vehicle.*') if int(b.get_attribute('number_of_wheels')) == 4 and not any(k in b.id for k in ('carlacola', 'firetruck', 'ambulance', 'sprinter', 'truck', 'bus', 'cybertruck', 't2'))]
def car_bp(role):
    b = rng.choice(cars_bp)
    if b.has_attribute('color'): b.set_attribute('color', rng.choice(b.get_attribute('color').recommended_values))
    b.set_attribute('role_name', role); return b
# ---- north surface lot: parked cars from the layout
parked = []
for i, pc in enumerate(layout['parked_cars']):
    if i % a.parked_stride: continue
    v = world.try_spawn_actor(car_bp('parked'), carla.Transform(C(pc['x'], pc['y'], 0.3), carla.Rotation(yaw=-float(pc['heading']))))
    if v is not None: v.apply_control(carla.VehicleControl(hand_brake=True)); parked.append(v)
# ---- ego on the south side of the loop heading east (authored ego: (-80, -225))
def road_tf(x, y):
    wp = cmap.get_waypoint(C(x, y), project_to_road=True, lane_type=carla.LaneType.Driving); tf = wp.transform; tf.location.z += 0.4; return wp, tf
ego_wp, ego_tf = road_tf(-80, -225)
eb = lib.find('vehicle.lincoln.mkz_2020'); eb.set_attribute('role_name', 'hero'); eb.set_attribute('color', '10,10,10')
ego = world.try_spawn_actor(eb, ego_tf); assert ego is not None, 'ego spawn failed'
# ---- NPC cars on the loop (both directions), Traffic Manager
loop_wps = cmap.generate_waypoints(8.0); rng.shuffle(loop_wps); cars = []
for wp in loop_wps:
    if len(cars) >= a.cars: break
    if wp.transform.location.distance(ego_tf.location) < 25: continue
    tf = wp.transform; tf.location.z += 0.4; v = world.try_spawn_actor(car_bp('npc'), tf)
    if v is not None: v.set_autopilot(True, tm.get_port()); tm.auto_lane_change(v, False); cars.append(v)
# ---- sidewalk paths: from each driving waypoint step sideways to the sidewalk lane (both sides)
def sidewalk_polylines():
    sides = {}
    for lane_id in (1, -1):
        pts = []
        wps = [w for w in cmap.generate_waypoints(4.0) if w.lane_id == lane_id]
        wps.sort(key=lambda w: (w.road_id, w.s))
        for w in wps:
            q = w
            for _ in range(3):
                q = q.get_right_lane()
                if q is None: break
                if q.lane_type == carla.LaneType.Sidewalk:   # walk on the OUTER half of the 2 m pavement (0.6 m further from the road) so the Traffic Manager does not brake for them
                    a_, b_ = w.transform.location, q.transform.location; d = b_ - a_; n = max(math.hypot(d.x, d.y), 0.1)
                    pts.append(carla.Location(b_.x + d.x / n * 0.6, b_.y + d.y / n * 0.6, b_.z)); break
        sides[lane_id] = pts
    return sides
SW = sidewalk_polylines(); print('sidewalk points:', {k: len(v) for k, v in SW.items()}, flush=True)
# ---- road paint (the generated mesh has none): yellow centre line, white edges, zebra at the south entrance crossing - drawn once, persistent
dbg = world.debug; Y = carla.Color(14, 10, 1); Wc = carla.Color(10, 10, 10)
for w in sorted([w for w in cmap.generate_waypoints(3.0) if w.lane_id == -1], key=lambda w: (w.road_id, w.s)):
    nx = w.next(3.0)
    if not nx: continue
    l0, l1 = w.transform.location, nx[0].transform.location; yaw = math.radians(w.transform.rotation.yaw); rx, ry = math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2); hw = w.lane_width / 2.0
    dbg.draw_line(carla.Location(l0.x - rx * hw, l0.y - ry * hw, l0.z + 0.05), carla.Location(l1.x - rx * hw, l1.y - ry * hw, l1.z + 0.05), thickness=0.1, color=Y, life_time=0.0)   # centre line (left edge of lane -1)
    dbg.draw_line(carla.Location(l0.x + rx * hw, l0.y + ry * hw, l0.z + 0.05), carla.Location(l1.x + rx * hw, l1.y + ry * hw, l1.z + 0.05), thickness=0.07, color=Wc, life_time=0.0)   # outer edge of lane -1
    dbg.draw_line(carla.Location(l0.x - rx * 3 * hw, l0.y - ry * 3 * hw, l0.z + 0.05), carla.Location(l1.x - rx * 3 * hw, l1.y - ry * 3 * hw, l1.z + 0.05), thickness=0.07, color=Wc, life_time=0.0)   # outer edge of lane +1
cr0 = layout['crossing']
for k in range(-3, 4):   # zebra bars across both lanes at the crossing (bars along x, stacked in y)
    xk = cr0['x'] + k * 1.0; dbg.draw_line(C(xk, cr0['y'] - 4.0, 0.06), C(xk, cr0['y'] + 4.0, 0.06), thickness=0.12, color=carla.Color(6, 6, 6), life_time=0.0)
# ---- pedestrians: walk along a sidewalk polyline, turn around at random; extra crowd at the entrances; one approach-triggered crosser
wbps = lib.filter('walker.pedestrian.*'); walkers = []
def wbp():
    b = rng.choice(wbps)
    if b.has_attribute('is_invincible'): b.set_attribute('is_invincible', 'false')
    return b
for i in range(a.walkers):
    side = rng.choice((1, -1)); pts = SW[side]
    if not pts: continue
    k = rng.randrange(len(pts)); loc = pts[k]
    w = world.try_spawn_actor(wbp(), carla.Transform(carla.Location(loc.x, loc.y, loc.z + 0.6), carla.Rotation(yaw=rng.uniform(0, 360))))
    if w is None: continue
    walkers.append(dict(actor=w, side=side, k=k, dir=rng.choice((1, -1)), speed=rng.uniform(0.8, 1.6), wait=rng.randint(0, 80), kind='stroll'))
cr = layout['crossing']; cross_from = C(cr['x'], cr['y'] - 6.0, 0.6); cross_to = C(cr['x'], cr['y'] + 6.0, 0.6)   # south kerb -> north kerb across the loop at the south entrance
cw = world.try_spawn_actor(wbp(), carla.Transform(cross_from, carla.Rotation(yaw=-90)))
crosser = dict(actor=cw, state='wait') if cw is not None else None
for _ in range(3): world.tick()
print(f'mall ready: {len(parked)} parked, {len(cars)} loop cars, {len(walkers)} walkers, crosser {"yes" if crosser else "no"}', flush=True)

def step_walkers():
    for wk in walkers:
        w = wk['actor']
        if not w.is_alive: continue
        if wk['wait'] > 0: wk['wait'] -= 1; w.apply_control(carla.WalkerControl(speed=0.0)); continue
        pts = SW[wk['side']]; loc = w.get_location(); t = pts[wk['k']]
        if math.hypot(t.x - loc.x, t.y - loc.y) < 1.0:
            wk['k'] += wk['dir']
            if wk['k'] < 0 or wk['k'] >= len(pts) or rng.random() < 0.01: wk['dir'] *= -1; wk['k'] = max(0, min(len(pts) - 1, wk['k'] + wk['dir'])); wk['wait'] = rng.randint(10, 60)
            t = pts[wk['k']]
        d = carla.Vector3D(t.x - loc.x, t.y - loc.y, 0.0); n = max(math.hypot(d.x, d.y), 0.1)
        w.apply_control(carla.WalkerControl(direction=carla.Vector3D(d.x / n, d.y / n, 0.0), speed=wk['speed']))
EVENT = ['', -99.0]
def step_crosser(tsim, ego_loc):
    if crosser is None or not crosser['actor'].is_alive: return
    w = crosser['actor']; loc = w.get_location()
    if crosser['state'] == 'wait':
        if loc.distance(ego_loc) < a.trigger: crosser['state'] = 'cross'; EVENT[0] = 'pedestrian steps off the kerb ahead!'; EVENT[1] = tsim; print('[mall] crossing triggered', flush=True)
        else: w.apply_control(carla.WalkerControl(speed=0.0))
    elif crosser['state'] == 'cross':
        d = carla.Vector3D(cross_to.x - loc.x, cross_to.y - loc.y, 0.0); n = math.hypot(d.x, d.y)
        if n < 0.8: crosser['state'] = 'done'; w.apply_control(carla.WalkerControl(speed=0.0))
        else: w.apply_control(carla.WalkerControl(direction=carla.Vector3D(d.x / n, d.y / n, 0.0), speed=1.3))

W, H = 1250, 660; CW, BW = 800, 450; img = {}
pygame.init(); screen = pygame.display.set_mode((W, H)); pygame.display.set_caption('Mall of America (approximate) - CARLA'); font = pygame.font.SysFont('monospace', 17, bold=True)
cams = []
for name, (loc, rot, w_, h_, fov) in {'chase': (carla.Location(x=-6.5, z=3.0), carla.Rotation(pitch=-12), CW, H, 90), 'bev': (carla.Location(z=120.0), carla.Rotation(pitch=-90), BW, H, 90)}.items():
    cb = lib.find('sensor.camera.rgb'); cb.set_attribute('image_size_x', str(w_)); cb.set_attribute('image_size_y', str(h_)); cb.set_attribute('fov', str(fov))
    cam = world.spawn_actor(cb, carla.Transform(loc, rot), attach_to=ego)
    cam.listen(lambda im, n=name: img.__setitem__(n, pygame.surfarray.make_surface(np.frombuffer(im.raw_data, dtype=np.uint8).reshape((im.height, im.width, 4))[:, :, :3][:, :, ::-1].swapaxes(0, 1)))); cams.append(cam)
clock = pygame.time.Clock(); ms_last = 50; steer = 0.0; ctl = carla.VehicleControl(); tick_n = 0; autopilot = a.test; t_start = time.time()
if autopilot: ego.set_autopilot(True, tm.get_port())
try:
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE): raise KeyboardInterrupt
            if ev.type == pygame.KEYUP and ev.key == pygame.K_q: ctl.reverse = not ctl.reverse; ctl.gear = -1 if ctl.reverse else 1
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_p: autopilot = not autopilot; ego.set_autopilot(autopilot, tm.get_port())
        keys = pygame.key.get_pressed(); vv = ego.get_velocity(); kmh = 3.6 * math.hypot(vv.x, vv.y)
        if not autopilot:
            gentle = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            ctl.throttle = (0.5 if gentle else 1.0) if (keys[pygame.K_w] or keys[pygame.K_UP]) else 0.0
            ctl.brake = 1.0 if (keys[pygame.K_s] or keys[pygame.K_DOWN]) else 0.0; ctl.hand_brake = bool(keys[pygame.K_SPACE])
            inc = 5e-4 * ms_last; smax = 0.7 if kmh < 50 else max(0.25, 0.7 - (kmh - 50) * (0.45 / 70))
            if keys[pygame.K_a] or keys[pygame.K_LEFT]: steer = max(-smax, steer - inc)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: steer = min(smax, steer + inc)
            else: steer = 0.0
            ctl.steer = round(steer, 2); ego.apply_control(ctl)
        ms_last = clock.tick_busy_loop(20) if not a.test else 50; world.tick(); tick_n += 1; tsim = world.get_snapshot().timestamp.elapsed_seconds
        step_walkers(); step_crosser(tsim, ego.get_location())
        screen.fill((15, 15, 15)); x = 0
        for n, w_ in (('chase', CW), ('bev', BW)):
            if n in img: screen.blit(img[n], (x, 0))
            x += w_
        hud = [f'{kmh:5.1f} km/h {"REVERSE " if ctl.reverse else ""}  loop cars {len(cars)}  parked {len(parked)}  walkers {len(walkers)}   ' + ('AUTOPILOT (P = manual)' if autopilot else 'W/S A/D  Shift gentle  Q reverse  Space handbrake  P autopilot  ESC quit')]
        if tsim - EVENT[1] < 3.0: hud.append('!! ' + EVENT[0])
        for i, line in enumerate(hud):
            lab = font.render(line, True, (255, 255, 0)); bg = pygame.Surface((lab.get_width() + 8, lab.get_height() + 2)); bg.set_alpha(150); bg.fill((0, 0, 0)); screen.blit(bg, (6, 6 + 22 * i)); screen.blit(lab, (10, 8 + 22 * i))
        pygame.display.flip()
        if a.test and a.snapshot and tick_n in (120, 400): pygame.image.save(screen, a.snapshot.replace('.png', f'_{tick_n}.png')); print('snapshot', tick_n, flush=True)
        if a.test and tick_n >= 400: break
except KeyboardInterrupt: pass
except Exception:
    import traceback; traceback.print_exc()
finally:
    pygame.quit()
    for cam in cams:
        try: cam.stop()
        except Exception: pass
    time.sleep(0.3)
    try:
        tm.set_synchronous_mode(False); st = world.get_settings(); st.synchronous_mode = False; st.fixed_delta_seconds = None; world.apply_settings(st)
    except Exception: pass
    print('bye', flush=True); os._exit(0)
